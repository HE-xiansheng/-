#!/usr/bin/env python3
"""实采回放：区分停流/排队与真实IMU时钟偏差；独立DDS域，无飞控接口。"""
import base64
import gzip
import json
import math
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import sys
import time

import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from livox_live_protocol import ros1_sensor_to_cdr


def main():
    out = Path(sys.argv[1])
    assert out.parent == Path('/tmp') and out.name.startswith('fastlio-receive-clock-fix-')
    assert os.environ['ROS_DOMAIN_ID'] == '184' and os.environ['ROS_LOCALHOST_ONLY'] == '1'
    root = Path('/home/orangepi/drone_platform')
    inputs = []
    with gzip.open(out / 'livox.jsonl.gz', 'rt') as source:
        for line in source:
            row = json.loads(line)
            kind = 1 if row['topic'] == '/livox/imu' else 2
            raw = base64.b64decode(row['serialized'])
            cdr, stamp, _ = ros1_sensor_to_cdr(kind, raw)
            duration = 0
            if kind == 2:
                offset = 16 + struct.unpack_from('<I', raw, 12)[0] + 20
                count = (len(raw) - offset) // 19
                duration = int(np.ndarray((count,), dtype='<u4', buffer=raw, offset=offset, strides=(19,)).max()) if count else 0
            inputs.append((kind, stamp, duration, cdr))
    first = min(stamp for kind, stamp, _, _ in inputs if kind == 1)
    last = max(stamp for kind, stamp, _, _ in inputs if kind == 1)
    events = sorted((stamp + duration - first, kind, stamp - first, cdr)
                    for kind, stamp, duration, cdr in inputs
                    if kind == 1 or first <= stamp <= stamp + duration <= last)
    period = last - first + 120000000
    rclpy.init()
    node = rclpy.create_node('source_clock_regression')
    publishers = {1: node.create_publisher(Imu, '/livox/imu', 200),
                  2: node.create_publisher(CustomMsg, '/livox/lidar', 20)}
    odometry, clouds = [], []
    def receive(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        odometry.append(dict(at=time.monotonic(), stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                             age=(time.time_ns()-message.header.stamp.sec*10**9-message.header.stamp.nanosec)/1e9,
                             pose=[p.x,p.y,p.z,q.x,q.y,q.z,q.w], frame=message.header.frame_id, child=message.child_frame_id))
    node.create_subscription(Odometry, '/Odometry', receive, 200)
    node.create_subscription(PointCloud2, '/cloud_registered_body', lambda m: clouds.append(m.header.stamp.sec*10**9+m.header.stamp.nanosec), 20)
    installed = root / '01_apps/fast_lio/workspace/install/fast_lio/lib/fast_lio/fastlio_mapping'
    candidate = out / 'install-v3/lib/fast_lio/fastlio_mapping'
    cases = {}
    try:
        for label, binary, gap, shift in [('candidate_gap', candidate, True, 0),
                                          ('candidate_imu_behind', candidate, False, -2000000000),
                                          ('candidate_imu_ahead', candidate, False, 2000000000)]:
            odometry.clear(); clouds.clear()
            path = out / label
            path.mkdir(exist_ok=True)
            with (path / 'node.log').open('w') as log:
                process = subprocess.Popen([str(binary), '--ros-args', '--params-file',
                                            str(out / 'config/lidar.yaml')],
                                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True, cwd=out,
                                           env=dict(os.environ, RMW_IMPLEMENTATION=os.environ.get('FASTLIO_TEST_CHILD_RMW', 'rmw_fastrtps_cpp')))
                try:
                    deadline = time.monotonic() + 12
                    while time.monotonic() < deadline and not all(p.get_subscription_count() for p in publishers.values()):
                        rclpy.spin_once(node, timeout_sec=.02)
                    assert all(p.get_subscription_count() for p in publishers.values()), 'missing subscriptions'
                    begin = time.monotonic() + .15
                    epoch = time.time_ns() + 150000000
                    for cycle in range(3):
                        for event, kind, relative, cdr in events:
                            offset = cycle*period + event
                            target = begin + offset*1e-9
                            while time.monotonic() < target:
                                rclpy.spin_once(node, timeout_sec=max(0, min(.005, target-time.monotonic())))
                            assert process.poll() is None, 'node exited'
                            if gap and kind == 1 and 150000000 <= offset <= 1550000000:
                                continue
                            stamp = epoch + cycle*period + relative + (shift if kind == 1 else 0)
                            data = bytearray(cdr)
                            struct.pack_into('<iI', data, 4, stamp//10**9, stamp%10**9)
                            if kind == 2:
                                frame_end = 16 + struct.unpack_from('<I', data, 12)[0]
                                aligned = 4 + ((frame_end-4+7)//8)*8
                                struct.pack_into('<Q', data, aligned, stamp)
                            publishers[kind].publish(bytes(data))
                            rclpy.spin_once(node, timeout_sec=0)
                    end = time.monotonic() + .6
                    while time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=.01)
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                    try: process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM);process.wait(timeout=3)
            text = (path / 'node.log').read_text()
            offsets = [float(v) for v in re.findall(r'Self sync IMU and LiDAR, time diff is ([-\d.]+)', text)]
            checks = {'clean exit': process.returncode == 0,
                      'clock decision observed': bool(offsets) or 'Source clock alignment verified' in text}
            if label == 'original_gap':
                checks['unmodified node reproduces false offset on missing IMU'] = len(offsets) == 1 and abs(offsets[0]) > 1.
            else:
                stable = [row for row in odometry if row['at'] > begin+5]
                checks.update({'sustained odometry and clouds': len(odometry)>=70 and len(clouds)>=70,
                               'output reaches end of input': bool(odometry) and odometry[-1]['at']-begin > (2*period+events[-1][0])*1e-9-.5,
                               'source stamps strictly increase': bool(odometry) and all(b['stamp']>a['stamp'] for a,b in zip(odometry,odometry[1:])),
                               'finite poses and expected frames': bool(odometry) and all(row['frame']=='camera_init' and row['child']=='body' and all(math.isfinite(v) for v in row['pose']) and abs(sum(v*v for v in row['pose'][3:])-1)<.01 for row in odometry),
                               'settled output age below existing relay threshold': bool(stable) and max(row['age'] for row in stable)<.2})
                checks['true offset preserved with correct sign' if shift else 'no fabricated offset during missing IMU'] = (len(offsets)==1 and abs(offsets[0]+shift*1e-9)<.05) if shift else not offsets
            case = dict(passed=all(checks.values()), checks=checks, measured_offsets_s=offsets,
                        odometry=len(odometry), clouds=len(clouds), returncode=process.returncode,
                        source_shift_ns=shift, startup_imu_gap=gap)
            (path / 'odometry.json').write_text(json.dumps(odometry))
            (path / 'result.json').write_text(json.dumps(case, indent=2)+'\n')
            cases[label]=case
            print(json.dumps({label:case}), flush=True)
            end=time.monotonic()+.5
            while time.monotonic()<end:rclpy.spin_once(node, timeout_sec=.02)
    finally:
        node.destroy_node();rclpy.try_shutdown()
    result=dict(passed=all(case['passed'] for case in cases.values()),cases=cases,
                scope='recorded sensor replay, localhost DDS184, no MAVROS or task/control services',
                candidate_rmw=os.environ.get('FASTLIO_TEST_CHILD_RMW', 'rmw_fastrtps_cpp'))
    (out/'regression-result.json').write_text(json.dumps(result,indent=2)+'\n')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
