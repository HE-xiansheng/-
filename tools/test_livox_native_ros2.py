#!/usr/bin/env python3
"""Observe the physical native LiDAR/LIO/relay in domain183 without any FCU."""
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile
from rclpy.serialization import deserialize_message
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

root = Path('/tmp/livox-native-check')
platform = Path('/home/orangepi/drone_platform')
assert os.environ['ROS_DOMAIN_ID'] == '183' and os.environ['ROS_LOCALHOST_ONLY'] == '1'
rows = {k: [] for k in ['lidar', 'imu', 'odom', 'cloud', 'vision']}
processes = []
logs = []
checks = {}
result = {'passed': False, 'scope': 'physical native LiDAR -> installed FAST-LIO -> pose relay; no MAVROS/FCU or setpoints'}
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = rclpy.create_node('native_livox_ground_check')


def header(raw):
    if raw[:4] != b'\x00\x01\x00\x00':
        raise ValueError('unexpected CDR representation')
    sec, ns, size = struct.unpack_from('<iII', raw, 4)
    end = 16+size
    if not size or end > len(raw) or raw[end-1] != 0 or ns >= 10**9:
        raise ValueError('invalid CDR header')
    return sec*10**9+ns, raw[16:end-1].decode(), end


def raw_receive(kind, raw):
    now = time.monotonic_ns()
    stamp, frame, offset = header(raw)
    extra = {}
    if kind in ('lidar', 'imu'):
        offset += -(offset-4) % 8
    if kind == 'lidar':
        timebase, count, lidar_id, a, b, c, sequence = struct.unpack_from('<QIB3BI', raw, offset)
        body = raw[offset+20:]
        if count != sequence or not count or len(body) not in (count*20-1, count*20):
            raise ValueError('native CustomMsg point schema mismatch')
        if len(body) == count*20-1:
            body += b'\x00'
        xyz = np.ndarray((count, 3), dtype='<f4', buffer=body, offset=4, strides=(20, 4))
        offsets = np.ndarray((count,), dtype='<u4', buffer=body, strides=(20,))
        extra = {'point_count': count, 'timebase': timebase, 'lidar_id': lidar_id,
                 'point_offsets_min_ns': int(offsets.min()), 'point_offsets_max_ns': int(offsets.max()),
                 'finite': bool(np.isfinite(xyz).all())}
    elif kind == 'imu':
        values = struct.unpack_from('<37d', raw, offset)
        if len(raw) != offset+37*8:
            raise ValueError('native Imu schema mismatch')
        extra = {'finite': all(math.isfinite(v) for v in values), 'gyro': values[13:16],
                 'acceleration': values[25:28], 'acceleration_norm': math.sqrt(sum(v*v for v in values[25:28]))}
    else:
        cloud = deserialize_message(raw, PointCloud2)
        fields = {f.name: f for f in cloud.fields}
        if any(k not in fields or fields[k].datatype != 7 or fields[k].count != 1 for k in ['x','y','z']):
            raise ValueError('unexpected body-cloud XYZ fields')
        if not cloud.width or not cloud.height or len(cloud.data) < cloud.row_step*cloud.height:
            raise ValueError('empty or truncated body cloud')
        dtype = np.dtype({'names': ['x','y','z'], 'formats': [('>' if cloud.is_bigendian else '<')+'f4']*3,
                          'offsets': [fields[k].offset for k in ['x','y','z']], 'itemsize': cloud.point_step})
        points = np.ndarray((cloud.height,cloud.width), dtype=dtype, buffer=cloud.data,
                            strides=(cloud.row_step,cloud.point_step))
        extra = {'point_count': cloud.height*cloud.width,
                 'finite': all(bool(np.isfinite(points[k]).all()) for k in ['x','y','z'])}
    if not rows[kind]:
        if kind == 'lidar':
            reference = deserialize_message(raw, CustomMsg)
            expected = np.asarray([(p.x,p.y,p.z) for p in reference.points], dtype='<f4')
            expected_offsets = np.asarray([p.offset_time for p in reference.points], dtype='<u4')
            extra['schema_reference_verified'] = bool(reference.point_num == count and reference.timebase == timebase
                and reference.header.frame_id == frame and np.array_equal(xyz,expected,equal_nan=True)
                and np.array_equal(offsets,expected_offsets))
        elif kind == 'imu':
            reference = deserialize_message(raw, Imu)
            extra['schema_reference_verified'] = bool(reference.header.frame_id == frame
                and tuple(extra['gyro']) == (reference.angular_velocity.x,reference.angular_velocity.y,reference.angular_velocity.z)
                and tuple(extra['acceleration']) == (reference.linear_acceleration.x,reference.linear_acceleration.y,reference.linear_acceleration.z))
        (root/(kind+'-first.cdr')).write_bytes(raw)
    rows[kind].append({'receipt_monotonic_ns': now, 'stamp_ns': stamp,
        'age_s': (time.time_ns()-stamp)/1e9, 'frame': frame, 'bytes': len(raw),
        'sha256': hashlib.sha256(raw).hexdigest(), **extra})


def pose_receive(kind, message):
    stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
    pose = message.pose.pose if kind == 'odom' else message.pose
    p, q = pose.position, pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    rows[kind].append({'receipt_monotonic_ns': time.monotonic_ns(), 'stamp_ns': stamp,
        'age_s': (time.time_ns()-stamp)/1e9, 'frame': message.header.frame_id,
        'child_frame': message.child_frame_id if kind == 'odom' else '', 'pose': values,
        'finite': all(math.isfinite(v) for v in values), 'quaternion_norm_squared': sum(v*v for v in values[3:])})


qos = QoSProfile(depth=200)
subs = [node.create_subscription(typ, topic, lambda m, k=kind: raw_receive(k, m), qos, raw=True)
    for typ, topic, kind in [(CustomMsg, '/livox/lidar', 'lidar'), (Imu, '/livox/imu', 'imu'),
                             (PointCloud2, '/factory_fastlio/cloud_body', 'cloud')]]
subs += [node.create_subscription(typ, topic, lambda m, k=kind: pose_receive(k, m), qos)
    for typ, topic, kind in [(Odometry, '/factory_fastlio/odometry', 'odom'),
                             (PoseStamped, '/mavros/vision_pose/pose', 'vision')]]


def start(name, command):
    log = (root/(name+'.log')).open('w')
    logs.append(log)
    p = subprocess.Popen(command, cwd=root, stdin=subprocess.DEVNULL, stdout=log,
                         stderr=subprocess.STDOUT, start_new_session=True)
    processes.append((name, p))
    (root/'owned-processes.json').write_text(json.dumps([
        {'name': n, 'pid': child.pid, 'start_ticks': Path('/proc/'+str(child.pid)+'/stat').read_text().split()[21]}
        for n, child in processes], indent=2)+'\n')


def check(name, ok):
    checks[name] = bool(ok)
    print(json.dumps({name: bool(ok)}), flush=True)


try:
    start('driver', ['ros2', 'launch', str(platform/'02_drivers/sensors/launch/livox_mid360s.launch.py'),
                     'device_config:='+str(root/'device-config.json')])
    start('lio', ['ros2', 'launch', str(platform/'01_apps/fast_lio/launch/localization.launch.py')])
    start('relay', ['ros2', 'run', 'to_mavros', 'vision2mavros'])
    begin = time.monotonic_ns()
    deadline = time.monotonic()+60
    print('NATIVE_CHECK_RUNNING', flush=True)
    while time.monotonic() < deadline:
        for name, p in processes:
            if p.poll() is not None:
                raise RuntimeError(name+' exited '+str(p.returncode))
        rclpy.spin_once(node, timeout_sec=.01)
    end = time.monotonic_ns()
    summary = {}
    for kind, minimum in [('lidar', 9), ('imu', 180), ('odom', 7), ('cloud', 7), ('vision', 7)]:
        all_rows = rows[kind]
        stable = [r for r in all_rows if begin+15*10**9 <= r['receipt_monotonic_ns'] <= end]
        hz = (len(stable)-1)*1e9/(stable[-1]['receipt_monotonic_ns']-stable[0]['receipt_monotonic_ns']) if len(stable)>1 else 0
        gap = max(((b['receipt_monotonic_ns']-a['receipt_monotonic_ns'])/1e9 for a,b in zip(stable,stable[1:])), default=999)
        summary[kind] = {'messages': len(all_rows), 'stable_messages': len(stable), 'hz': hz,
            'max_gap_s': gap, 'min_age_s': min((r['age_s'] for r in all_rows), default=None),
            'max_age_s': max((r['age_s'] for r in all_rows), default=None),
            'frames': sorted({r['frame'] for r in all_rows})}
        check(kind+'_sustained_rate', hz >= minimum)
        check(kind+'_no_two_second_gap', gap < 2)
        check(kind+'_source_age_bounded', bool(all_rows) and all(-.1 < r['age_s'] < 2 for r in all_rows))
        check(kind+'_source_stamps_increase', bool(all_rows) and all(b['stamp_ns'] > a['stamp_ns'] for a,b in zip(all_rows,all_rows[1:])))
        check(kind+'_finite_data', bool(all_rows) and all(r['finite'] for r in all_rows))
    check('lidar_imu_frames', bool(rows['lidar']) and bool(rows['imu']) and
          all(r['frame'] == 'livox_frame' for k in ['lidar','imu'] for r in rows[k]))
    check('raw_schema_matches_ROS_deserialization', all(rows[k] and rows[k][0].get('schema_reference_verified') for k in ['lidar','imu']))
    check('nonempty_body_cloud', bool(rows['cloud']) and all(r['frame'] == 'body' and r['point_count'] > 0 for r in rows['cloud']))
    check('lio_pose_frames_and_rotation', bool(rows['odom']) and all(r['frame'] == 'camera_init' and r['child_frame'] == 'body'
          and abs(r['quaternion_norm_squared']-1) < .01 for r in rows['odom']))
    odom_stamps = {r['stamp_ns'] for r in rows['odom']}
    check('relay_preserves_actual_lio_stamps', bool(rows['vision']) and {r['stamp_ns'] for r in rows['vision']} <= odom_stamps)
    check('no_FCU_or_setpoint_interfaces', not any(node.get_publishers_info_by_topic(t) for t in
          ['/mavros/setpoint_raw/local','/ego_planner/flight_execution','/Safina_track_stamped']) and
          not any(n in ['/mavros/cmd/arming','/mavros/set_mode'] for n,_ in node.get_service_names_and_types()))
    result.update(streams=summary)
    if rows['imu']:
        norms = [r['acceleration_norm'] for r in rows['imu']]
        result['imu_acceleration_norm'] = {'min': min(norms), 'mean': sum(norms)/len(norms), 'max': max(norms)}
    dump = subprocess.run(['ros2','param','dump','/factory_laserMapping'], stdin=subprocess.DEVNULL,
                          capture_output=True,text=True,timeout=15)
    (root/'lio-parameters.yaml').write_text(dump.stdout+dump.stderr)
    check('lio_parameter_snapshot', dump.returncode == 0)
except Exception as exc:
    result['error'] = repr(exc)
    print(repr(exc), flush=True)
finally:
    for name, p in reversed(processes):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=5)
    for log in logs:
        log.close()
    result['returncodes'] = {n:p.returncode for n,p in processes}
    check('owned_processes_exit_normally', bool(processes) and all(p.returncode == 0 for _,p in processes))
    for kind, data in rows.items():
        with (root/(kind+'-headers.jsonl')).open('w') as f:
            for row in data:
                f.write(json.dumps(row)+'\n')
    result.update(checks=checks, passed=bool(checks) and all(checks.values()) and 'error' not in result)
    (root/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    node.destroy_node()
    rclpy.try_shutdown()
raise SystemExit(0 if result['passed'] else 1)
