#!/usr/bin/env python3
"""Observe camera and MAVROS data; never publish or call control services."""
import datetime
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState
from tf2_msgs.msg import TFMessage


def main():
    rclpy.init()
    node = rclpy.create_node('peripheral_data_audit')
    records = {}
    subscriptions = []
    transforms = {}

    def callback(topic, msg):
        now = time.monotonic()
        rec = records.setdefault(topic, {'count': 0, 'first_receipt': now,
                                         'last_stamp': None, 'stamp_regressions': 0})
        rec['count'] += 1
        rec['last_receipt'] = now
        if hasattr(msg, 'header'):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if rec['last_stamp'] is not None and stamp < rec['last_stamp']:
                rec['stamp_regressions'] += 1
            rec['last_stamp'] = stamp
            rec['frame_id'] = msg.header.frame_id
            rec['age_seconds'] = node.get_clock().now().nanoseconds * 1e-9 - stamp
        if isinstance(msg, Image):
            rec.update(width=msg.width, height=msg.height, encoding=msg.encoding,
                       step=msg.step, data_bytes=len(msg.data))
            if 'sample' not in rec and msg.encoding in ('16UC1', '32FC1'):
                dtype = np.dtype('>u2' if msg.is_bigendian else '<u2') if msg.encoding == '16UC1' else np.dtype('>f4' if msg.is_bigendian else '<f4')
                values = np.ndarray((msg.height, msg.width), dtype=dtype,
                                    buffer=msg.data, strides=(msg.step, dtype.itemsize))
                valid = values[np.isfinite(values) & (values > 0)]
                rec['sample'] = {'valid_fraction': float(valid.size / values.size),
                                 'raw_median': float(np.median(valid)) if valid.size else None,
                                 'unit': 'millimetres' if msg.encoding == '16UC1' else 'metres'}
        elif isinstance(msg, CameraInfo):
            rec.update(width=msg.width, height=msg.height, k=list(msg.k),
                       distortion_model=msg.distortion_model, d=list(msg.d))
        elif isinstance(msg, PointCloud2):
            rec.update(width=msg.width, height=msg.height, point_step=msg.point_step,
                       fields=[f.name for f in msg.fields])
        elif isinstance(msg, State):
            rec.update(connected=msg.connected, armed=msg.armed, mode=msg.mode,
                       system_status=msg.system_status)
        elif isinstance(msg, ExtendedState):
            rec.update(landed_state=msg.landed_state, vtol_state=msg.vtol_state)
        elif isinstance(msg, PoseStamped):
            rec['position'] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    expected = {
        '/mavros/state': State, '/mavros/extended_state': ExtendedState,
        '/mavros/imu/data': Imu, '/mavros/local_position/pose': PoseStamped,
    }
    for stream in ('color', 'depth', 'left_ir', 'right_ir'):
        expected[f'/camera/{stream}/image_raw'] = Image
        expected[f'/camera/{stream}/camera_info'] = CameraInfo
    expected['/camera/depth/points'] = PointCloud2
    expected['/camera/depth_registered/points'] = PointCloud2
    expected['/camera/depth/color/points'] = PointCloud2
    for topic, cls in expected.items():
        # Camera publishes RELIABLE. Large raw images/clouds lose samples with
        # best-effort readers on this platform; retain sensor QoS for MAVROS.
        qos = QoSProfile(depth=5) if cls in (Image, PointCloud2) else qos_profile_sensor_data
        subscriptions.append(node.create_subscription(
            cls, topic, lambda msg, topic=topic: callback(topic, msg), qos))

    def on_tf(msg):
        for tf in msg.transforms:
            transforms[tf.child_frame_id] = {'parent': tf.header.frame_id,
                'translation': [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z],
                'rotation_xyzw': [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]}

    subscriptions.append(node.create_subscription(TFMessage, '/tf_static', on_tf,
        QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    subscriptions.append(node.create_subscription(TFMessage, '/tf', on_tf, qos_profile_sensor_data))
    start = time.monotonic()
    while time.monotonic() - start < 35:
        rclpy.spin_once(node, timeout_sec=0.2)
    for rec in records.values():
        elapsed = rec['last_receipt'] - rec['first_receipt']
        rec['receipt_hz'] = (rec['count'] - 1) / elapsed if elapsed > 0 else 0
        rec['stale_seconds'] = time.monotonic() - rec['last_receipt']
    graph = {}
    for topic, types in node.get_topic_names_and_types():
        pubs = node.get_publishers_info_by_topic(topic)
        if pubs:
            graph[topic] = {'types': types, 'publishers': [
                {'node': p.node_namespace + '/' + p.node_name,
                 'reliability': str(p.qos_profile.reliability),
                 'durability': str(p.qos_profile.durability)} for p in pubs]}
    images = [f'/camera/{s}/image_raw' for s in ('color', 'depth', 'left_ir', 'right_ir')]
    checks = {
        'flight_controller_connected': records.get('/mavros/state', {}).get('connected') is True,
        'flight_controller_disarmed': records.get('/mavros/state', {}).get('armed') is False,
        'four_image_streams': all(records.get(t, {}).get('count', 0) >= 10 for t in images),
        'four_camera_calibrations': all(records.get(t.replace('image_raw', 'camera_info'), {}).get('k', [0])[0] > 0 for t in images),
        'depth_has_valid_samples': records.get('/camera/depth/image_raw', {}).get('sample', {}).get('valid_fraction', 0) > 0,
        'point_cloud_received': any(r.get('count', 0) >= 5 for t, r in records.items() if 'points' in t),
        'camera_tf_received': any('camera' in f for f in transforms),
        'timestamps_monotonic': all(r['stamp_regressions'] == 0 for r in records.values()),
        'image_timestamps_current': all(abs(records.get(t, {}).get('age_seconds', 999)) < 2 for t in images),
    }
    result = {'checked_at': datetime.datetime.now().astimezone().isoformat(),
              'duration_seconds': time.monotonic() - start, 'checks': checks,
              'passed': all(checks.values()), 'topics': records, 'tf': transforms,
              'graph': graph, 'flight_commands_sent': False,
              'limitations': ['Short bench sample only; no external pose fusion or flight validation',
                              'Depth units follow ROS encoding; not independently measured against a target',
                              'No camera-to-airframe extrinsic calibration assumed']}
    directory = Path.home() / 'drone_platform/04_data/logs/ros2_migration'
    filename = directory / datetime.datetime.now().strftime('peripherals-%Y%m%d-%H%M%S.json')
    filename.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'result_file': str(filename), 'passed': result['passed'], 'checks': checks}, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
