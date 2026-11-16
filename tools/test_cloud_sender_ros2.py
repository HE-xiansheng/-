#!/usr/bin/env python3
"""在隔离DDS域运行原生点云发送器；仅合成消息和回环UDP。"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sender', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') == '198'
    out = Path(args.output)
    out.mkdir(exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('cloud_sender_fixture')
    cloud_pub = node.create_publisher(PointCloud2, '/test_cloud', 1)
    pose_pub = node.create_publisher(Odometry, '/test_pose', qos_profile_sensor_data)
    sockets = []
    for _ in range(2):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('127.0.0.1', 0))
        sock.setblocking(False)
        sockets.append(sock)
    log = (out / 'sender.log').open('w')
    child = subprocess.Popen([sys.executable, args.sender, '--pc-ip', '127.0.0.1',
        '--port', str(sockets[0].getsockname()[1]), '--pose-port', str(sockets[1].getsockname()[1]),
        '--topic', '/test_cloud', '--pose-topic', '/test_pose', '--max-hz', '20'], stdout=log, stderr=subprocess.STDOUT)
    checks = []

    def check(name, value):
        checks.append({'name': name, 'passed': bool(value)})
        print(('PASS ' if value else 'FAIL ') + name, flush=True)
        if not value:
            raise AssertionError(name)

    # 两行组织点云含行填充，XYZ为大端float64；另含一项NaN需要过滤。
    cloud = PointCloud2()
    cloud.header.frame_id = 'camera_init'
    cloud.width = cloud.height = 2
    cloud.is_bigendian = True
    cloud.point_step = 24
    cloud.row_step = 56
    cloud.fields = [PointField(name=k, offset=i*8, datatype=PointField.FLOAT64, count=1)
                    for i, k in enumerate(('x', 'y', 'z'))]
    cloud.data = (struct.pack('>6d', 1., 2., 3., 4., 5., 6.) + b'padding!'
                  + struct.pack('>6d', 7., 8., 9., math.nan, 0., 0.) + b'padding!')
    pose = Odometry()
    pose.header.frame_id = 'camera_init'
    pose.child_frame_id = 'body'
    pose.pose.pose.position.x = 1.25
    pose.pose.pose.position.y = -2.5
    pose.pose.pose.position.z = .3
    pose.pose.pose.orientation.z = math.sqrt(.5)
    pose.pose.pose.orientation.w = math.sqrt(.5)

    def drain():
        rows = [[], []]
        for i, sock in enumerate(sockets):
            while True:
                try:
                    rows[i].append(sock.recv(65535))
                except BlockingIOError:
                    break
        return rows

    def send(c, p, duration=.45, offset=0.):
        result = [[], []]
        until = time.monotonic() + duration
        while time.monotonic() < until:
            stamp = node.get_clock().now().to_msg()
            if offset:
                ns = stamp.sec * 1000000000 + stamp.nanosec + int(offset * 1e9)
                stamp.sec, stamp.nanosec = divmod(ns, 1000000000)
            if c is not None:
                c.header.stamp = stamp
                cloud_pub.publish(c)
            if p is not None:
                p.header.stamp = stamp
                pose_pub.publish(p)
            rclpy.spin_once(node, timeout_sec=.025)
            rows = drain()
            for i in range(2):
                result[i].extend(rows[i])
        return result

    try:
        until = time.monotonic() + 8
        while time.monotonic() < until and (not cloud_pub.get_subscription_count() or not pose_pub.get_subscription_count()):
            rclpy.spin_once(node, timeout_sec=.05)
        check('DDS cloud and sensor-data odometry subscriptions match', cloud_pub.get_subscription_count() == 1 and pose_pub.get_subscription_count() == 1)
        rows = send(cloud, pose, .8)
        check('PCD1 and POS1 received via actual UDP', len(rows[0]) > 1 and len(rows[1]) > 1)
        first = rows[0][0]
        magic, seq, index, count, raw_len, packed_len, size = struct.unpack('<IIHHIIH', first[:22])
        check('PCD1 unchanged framing and finite point filtering', magic == 0x50434431 and index == 0 and count == 1 and raw_len == 36 and size == packed_len == len(first)-22)
        (out / 'cloud-header.bin').write_bytes(first)
        (out / 'cloud-compressed.bin').write_bytes(first[22:])
        (out / 'pose.bin').write_bytes(rows[1][0])
        (out / 'pose-new.bin').write_bytes(rows[1][-1])
        fields = struct.unpack('<IId7f', rows[1][0])
        check('POS1 retains meters and XYZW without double coordinate conversion', fields[0] == 0x504f5331 and all(abs(a-b) < 1e-6 for a,b in zip(fields[3:], [1.25,-2.5,.3,0.,0.,math.sqrt(.5),math.sqrt(.5)])))
        time.sleep(.15)
        drain()
        bad_pose = copy.deepcopy(pose)
        bad_pose.child_frame_id = 'wrong'
        check('wrong body frame suppresses pose', not send(cloud, bad_pose)[1])
        bad_pose = copy.deepcopy(pose)
        bad_pose.pose.pose.orientation.w = 0.
        check('nonunit quaternion suppresses pose', not send(cloud, bad_pose)[1])
        check('source stale cloud and odometry suppressed', not any(send(cloud, pose, offset=-2.)))
        check('future cloud and odometry suppressed', not any(send(cloud, pose, offset=2.)))
        bad_cloud = copy.deepcopy(cloud)
        bad_cloud.header.frame_id = 'wrong'
        check('cloud frame mismatch suppresses both outputs', not any(send(bad_cloud, pose)))
        bad_cloud = copy.deepcopy(cloud)
        bad_cloud.row_step = 1
        check('invalid PointCloud2 stride suppresses cloud', not send(bad_cloud, None)[0])
        recovered = send(cloud, pose)
        check('valid input recovers stream', bool(recovered[0]) and bool(recovered[1]))
        time.sleep(.75)
        drain()
        check('fresh odometry with stale cloud suppressed', not send(None, pose)[1])
        check('sender has no control publishers', node.get_publishers_info_by_topic('/mavros/setpoint_raw/local') == [])
    finally:
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()
        log.close()
        if sys.exc_info()[0] is None:
            check('sender normal SIGINT shutdown', child.returncode == 0)
        for sock in sockets:
            sock.close()
        node.destroy_node()
        rclpy.shutdown()
        (out / 'result.json').write_text(json.dumps({'passed': all(c['passed'] for c in checks), 'checks': checks}, indent=2)+'\n')


if __name__ == '__main__':
    main()
