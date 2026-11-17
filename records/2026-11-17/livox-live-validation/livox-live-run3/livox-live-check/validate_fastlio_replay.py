#!/usr/bin/env python3
"""Replay read-only ROS1 Livox samples into isolated native ROS2 FAST-LIO."""
import base64
import gzip
import json
import math
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
from builtin_interfaces.msg import Time
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


def decode(entry):
    raw = base64.b64decode(entry['serialized'])
    _, sec, nanosec, length = struct.unpack_from('<IIII', raw)
    frame = raw[16:16 + length].decode()
    offset = 16 + length
    if entry['topic'] == '/livox/lidar':
        msg = CustomMsg()
        values = struct.unpack_from('<QIB3BI', raw, offset)
        msg.timebase, msg.point_num, msg.lidar_id = values[:3]
        msg.rsvd = values[3:6]
        count = values[6]
        offset += 20
        assert len(raw) - offset == count * 19
        points = []
        for values in struct.iter_unpack('<IfffBBB', raw[offset:]):
            p = CustomPoint()
            p.offset_time, p.x, p.y, p.z, p.reflectivity, p.tag, p.line = values
            points.append(p)
        msg.points = points
        assert msg.point_num == len(points)
    else:
        msg = Imu()
        v = struct.unpack_from('<37d', raw, offset)
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = v[:4]
        msg.orientation_covariance = v[4:13]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = v[13:16]
        msg.angular_velocity_covariance = v[16:25]
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = v[25:28]
        msg.linear_acceleration_covariance = v[28:37]
    msg.header.stamp = Time(sec=sec, nanosec=nanosec)
    msg.header.frame_id = frame
    return entry['topic'], entry['receipt_offset'], msg


def main():
    os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='178')
    with gzip.open('/dev/shm/ros2-livox-replay.jsonl.gz', 'rt') as stream:
        messages = [decode(json.loads(line)) for line in stream]
    # Capture callbacks may block behind large cloud serialization. Replay by
    # measurement time, including cloud point offsets, not observer receipt time.
    imu_stamps = [m.header.stamp.sec + m.header.stamp.nanosec / 1e9 for t, _, m in messages if t == '/livox/imu']
    minimum, maximum = min(imu_stamps), max(imu_stamps)
    ordered = []
    for topic, _, message in messages:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        event_time = stamp + (max((p.offset_time for p in message.points), default=0) / 1e9 if topic == '/livox/lidar' else 0)
        if topic == '/livox/imu' or minimum <= stamp <= event_time <= maximum:
            ordered.append((topic, event_time - minimum, message))
    messages = sorted(ordered, key=lambda item: item[1])
    counts = {topic: sum(t == topic for t, _, _ in messages) for topic in ['/livox/lidar', '/livox/imu']}
    assert all(counts.values()), counts
    with tempfile.TemporaryDirectory(prefix='fastlio-validation-') as log_dir:
        os.environ['ROS_LOG_DIR'] = log_dir
        rclpy.init()
        node = rclpy.create_node('fastlio_replay_validation')
        publishers = {'/livox/lidar': node.create_publisher(CustomMsg, '/livox/lidar', 20),
                      '/livox/imu': node.create_publisher(Imu, '/livox/imu', 200)}
        clock_pub = node.create_publisher(Clock, '/clock', 200)
        odometry, clouds = [], []
        subscriptions = [node.create_subscription(Odometry, '/factory_fastlio/odometry', odometry.append, 200),
                         node.create_subscription(PointCloud2, '/factory_fastlio/cloud_body',
                                                  lambda m: clouds.append((m.width * m.height, m.header.frame_id)), 20)]
        process = subprocess.Popen(['ros2', 'launch', '/home/orangepi/drone_platform/01_apps/fast_lio/launch/localization.launch.py',
                                    'use_sim_time:=true'], stdout=sys.stderr, stderr=sys.stderr,
                                   text=True, start_new_session=True)
        try:
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline and not all(p.get_subscription_count() for p in publishers.values()):
                rclpy.spin_once(node, timeout_sec=.1)
            assert all(p.get_subscription_count() for p in publishers.values()), 'FAST-LIO subscriptions missing'
            first_stamp = messages[0][2].header.stamp
            clock_start = first_stamp.sec * 10**9 + first_stamp.nanosec
            receipt_start = messages[0][1]
            wall_start = time.monotonic()
            for topic, receipt, msg in messages:
                # Half-speed wall replay, original source timestamps/point times.
                target = wall_start + (receipt - receipt_start) * 2
                while time.monotonic() < target:
                    rclpy.spin_once(node, timeout_sec=max(0.0, min(.02, target - time.monotonic())))
                stamp = clock_start + int((receipt - receipt_start) * 1e9)
                clock_pub.publish(Clock(clock=Time(sec=stamp // 10**9, nanosec=stamp % 10**9)))
                publishers[topic].publish(msg)
                rclpy.spin_once(node, timeout_sec=0)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            stamps = [o.header.stamp.sec * 10**9 + o.header.stamp.nanosec for o in odometry]
            valid = all(all(math.isfinite(v) for v in [o.pose.pose.position.x, o.pose.pose.position.y,
                        o.pose.pose.position.z, o.twist.twist.linear.x, o.twist.twist.linear.y,
                        o.twist.twist.linear.z]) and abs(sum(v*v for v in [o.pose.pose.orientation.x,
                        o.pose.pose.orientation.y, o.pose.pose.orientation.z, o.pose.pose.orientation.w]) - 1) < .01
                        for o in odometry)
            checks = {'odometry_received': len(odometry) >= 20, 'clouds_received': len(clouds) >= 20,
                      'finite_pose_velocity_and_unit_quaternion': bool(odometry) and valid,
                      'source_time_monotonic': bool(stamps) and all(b > a for a, b in zip(stamps, stamps[1:])),
                      'expected_frames': bool(odometry) and all(o.header.frame_id == 'camera_init'
                          and o.child_frame_id == 'body' for o in odometry)
                          and all(frame == 'body' for _, frame in clouds)}
            result = {'passed': all(checks.values()), 'checks': checks, 'input_counts': counts,
                      'output_odometry_count': len(odometry), 'output_cloud_count': len(clouds),
                      'source_duration_seconds': messages[-1][1] - receipt_start,
                      'flight_commands_sent': False,
                      'limitations': ['Recorded .235 sensor input trimmed to common IMU/cloud measurement time; isolated DDS, not physical radar test',
                                      'No trajectory reference comparison, long-term drift, FC input or fusion tested']}
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()
        result['shutdown_return_code'] = process.returncode
        result['checks']['clean_shutdown'] = process.returncode == 0
        result['passed'] = all(result['checks'].values())
        print(json.dumps(result, indent=2))
        return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
