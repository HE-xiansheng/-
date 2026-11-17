#!/usr/bin/env python3
"""使用被动C++/Python观察器检查基础服务，不操作生产节点或飞控服务。"""
import collections
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped
from livox_ros_driver2.msg import CustomMsg
from mavros_msgs.msg import State

assert os.environ.get('ROS_DOMAIN_ID', '0') == '0'
rclpy.init()
node = rclpy.create_node('base_autostart_readonly_check')
stats = collections.defaultdict(list)
states = []
subscriptions = []
specification = [
    ('lidar', '/livox/lidar', CustomMsg, 8),
    ('lidar_imu', '/livox/imu', Imu, 150),
    ('color', '/camera/color/image_raw', Image, 20),
    ('depth', '/camera/depth/image_raw', Image, 20),
    ('gimbal_raw', '/rtsp_cam/image_raw', Image, 20),
    ('gimbal_color', '/rtsp_cam/image_color', Image, 10),
    ('fcu_imu', '/mavros/imu/data', Imu, 10),
    ('fcu_pose', '/mavros/local_position/pose', PoseStamped, 1),
]


def receive(key, message):
    stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
    stats[key].append((time.monotonic(), stamp, (time.time_ns() - stamp) / 1e9))


def state(message):
    states.append({'receipt': time.monotonic(), 'connected': message.connected,
                   'armed': message.armed, 'mode': message.mode})
    if message.armed:
        raise RuntimeError('PX4 armed state observed; check stopped')


for key, topic, kind, minimum in specification:
    if key not in ('fcu_imu', 'fcu_pose'):
        continue
    qos = QoSProfile(depth=5) if kind in (Image, CustomMsg) or key == 'lidar_imu' else qos_profile_sensor_data
    subscriptions.append(node.create_subscription(kind, topic,
        lambda message, key=key: receive(key, message), qos))
subscriptions.append(node.create_subscription(State, '/mavros/state', state, 10))
result = {'passed': False, 'scope': 'existing production base services, passive subscriptions only',
          'checks': {}, 'streams': {}, 'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
target = Path(sys.argv[1])
image_csv = target.with_suffix('.images.csv')
source_csv = target.with_suffix('.source.csv')
stop_file = target.with_suffix('.observer.stop')
assert not stop_file.exists()
observer_log = target.with_suffix('.observer.log').open('w')
observer = subprocess.Popen([str(target.parent/'native-observer'), '--ros-args',
    '-p', 'output:='+str(image_csv), '-p', 'sensor_output:='+str(source_csv),
    '-p', 'include_camera:=true', '-p', 'stop_file:='+str(stop_file)],
    stdout=observer_log, stderr=subprocess.STDOUT)


def stop_observer():
    stop_file.touch()
    if observer.poll() is None:
        observer.wait(timeout=10)
    observer_log.close()


begin = time.monotonic()
sample_begin = begin + 20
end = sample_begin + 30
last_report = begin
try:
    while time.monotonic() < end:
        if observer.poll() is not None:
            raise RuntimeError('native observer unexpectedly stopped')
        rclpy.spin_once(node, timeout_sec=.02)
        now = time.monotonic()
        if now - last_report >= 5:
            print(json.dumps({'elapsed_s': round(now-begin, 1),
                              'counts': {key: len(rows) for key, rows in stats.items()},
                              'last_fcu': states[-1] if states else None}), flush=True)
            last_report = now
    stop_observer()
    result['checks']['native_observer_normal_exit'] = observer.returncode == 0
    with image_csv.open() as source:
        for row in csv.DictReader(source):
            key = {'stereo': 'color', 'depth': 'depth', 'raw': 'gimbal_raw',
                   'color': 'gimbal_color'}.get(row['key'])
            if row['qos'] == 'reliable' and key:
                stats[key].append((int(row['receipt_ns'])/1e9,
                                   int(row['stamp_ns']), int(row['age_ns'])/1e9))
    with source_csv.open() as source:
        for row in csv.DictReader(source):
            key = {'lidar': 'lidar', 'imu': 'lidar_imu'}.get(row['kind'])
            if key:
                stats[key].append((int(row['callback_monotonic_ns'])/1e9, int(row['stamp_ns']),
                                   (int(row['callback_wall_ns'])-int(row['stamp_ns']))/1e9))
    for key, topic, kind, minimum in specification:
        rows = [row for row in stats[key] if row[0] >= sample_begin]
        rate = (len(rows)-1)/(rows[-1][0]-rows[0][0]) if len(rows)>1 else 0
        result['streams'][key] = {'messages': len(rows), 'hz': rate,
            'min_age_s': min((row[2] for row in rows), default=None),
            'max_age_s': max((row[2] for row in rows), default=None)}
        result['checks'][key + '_rate'] = rate >= minimum
        result['checks'][key + '_timestamps'] = bool(rows) and all(
            b[1] > a[1] for a, b in zip(rows, rows[1:]))
        result['checks'][key + '_age'] = bool(rows) and all(-.1 < row[2] < 1 for row in rows)
    result['checks']['PX4_connected_disarmed'] = bool(states) and states[-1]['connected'] and all(
        not row['armed'] for row in states) and time.monotonic()-states[-1]['receipt'] < 3
    # 任务及外部定位保持手动，不应该有对应的发布者。
    for topic in ['/mavros/vision_pose/pose', '/mavros/setpoint_position/local',
                  '/mavros/setpoint_raw/local', '/mavros/setpoint_raw/attitude']:
        result['checks']['no_task_publisher:' + topic] = not node.get_publishers_info_by_topic(topic)
    for unit in ['Safina_sensor.service', 'gimbal-rtsp-relay.service']:
        values = subprocess.check_output(['systemctl', 'show', unit, '-p', 'ActiveState',
                                           '-p', 'UnitFileState', '-p', 'NRestarts'], text=True)
        result[unit] = dict(line.split('=', 1) for line in values.splitlines())
        result['checks'][unit] = (result[unit]['ActiveState'] == 'active' and
                                  result[unit]['UnitFileState'] == 'enabled')
    result['passed'] = all(result['checks'].values())
except Exception as error:
    result['error'] = repr(error)
finally:
    stop_observer()
    result['last_states'] = states[-5:]
    node.destroy_node()
    rclpy.shutdown()
    Path(sys.argv[1]).write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(result, ensure_ascii=False), flush=True)
raise SystemExit(0 if result['passed'] else 1)
