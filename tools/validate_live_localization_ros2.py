#!/usr/bin/env python3
"""Passively validate an existing ground localization session; no control outputs."""
import argparse
from collections import Counter
import json
import math
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor, await_or_execute
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, EstimatorStatus


class MetadataExecutor(SingleThreadedExecutor):
    # Preserve the native MessageInfo in the installed early Humble executor.
    def _take_subscription(self, sub):
        with sub.handle:
            return sub.handle.take_message(sub.msg_type, sub.raw)

    async def _execute_subscription(self, sub, value):
        if value is not None:
            await await_or_execute(sub.callback, *value)


def distribution(values):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    return {name: values[round(q*(len(values)-1))] for name, q in
            [('min', 0), ('median', .5), ('p95', .95), ('max', 1)]} if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=120)
    args = parser.parse_args()
    assert 30 <= args.seconds <= 300
    definitions = {
        'imu': (Imu, '/livox/imu'),
        'lio': (Odometry, '/factory_fastlio/odometry'),
        'relay': (PoseStamped, '/mavros/vision_pose/pose'),
        'fc_odom': (Odometry, '/mavros/local_position/odom'),
        'state': (State, '/mavros/state'),
        'ekf': (EstimatorStatus, '/mavros/estimator_status'),
    }
    data = {key: [] for key in definitions}
    rclpy.init()
    node = rclpy.create_node('localization_acceptance_observer', enable_rosout=False,
                             start_parameter_services=False)
    executor = MetadataExecutor()
    executor.add_node(node)
    began = time.monotonic()

    def callback(key):
        def receive(message, info):
            now = time.time_ns()
            stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
            published = info.get('source_timestamp', 0) if isinstance(info, dict) else info.source_timestamp
            row = {'at': time.monotonic()-began, 'stamp': stamp, 'age_s': (now-stamp)*1e-9,
                   'prepublish_age_s': (published-stamp)*1e-9 if published else None,
                   'publish_to_callback_s': (now-published)*1e-9 if published else None}
            if key in ('lio', 'relay', 'fc_odom'):
                pose = message.pose if key == 'relay' else message.pose.pose
                p, q = pose.position, pose.orientation
                row.update(pose=[p.x, p.y, p.z, q.x, q.y, q.z, q.w],
                           frame=message.header.frame_id)
            elif key == 'state':
                row.update(connected=message.connected, armed=message.armed, mode=message.mode)
                if message.armed or message.mode == 'OFFBOARD':
                    raise RuntimeError('ground observation requires disarmed non-OFFBOARD state')
            elif key == 'ekf':
                row['flags'] = {f: getattr(message, f) for f in message.get_fields_and_field_types()
                                if f.endswith('_flag')}
            data[key].append(row)
        return receive

    try:
        # A depth-five observer can itself discard IMU samples during a short
        # scheduling delay. Retain the same bounded two-second sensor history as
        # the candidate; the strict source-age check still detects old inputs.
        imu_qos = QoSProfile(depth=400, reliability=ReliabilityPolicy.BEST_EFFORT)
        subscriptions = [node.create_subscription(typ, topic, callback(key),
                                                  imu_qos if key == 'imu' else qos_profile_sensor_data)
                         for key, (typ, topic) in definitions.items()]
        while time.monotonic()-began < args.seconds:
            executor.spin_once(timeout_sec=.01)
        stable = {key: [r for r in rows if r['at'] >= 5] for key, rows in data.items()}
        gaps = lambda rows: [b['at']-a['at'] for a, b in zip(rows, rows[1:])]
        checks = {}
        for key, min_hz, max_age in [('imu', 180, .1), ('lio', 8, .2),
                                      ('relay', 8, .2), ('fc_odom', 25, .2)]:
            rows = stable[key]
            checks[key+' sustained measurements'] = len(rows) >= (args.seconds-5)*min_hz
            checks[key+' source freshness'] = bool(rows) and all(-.02 <= r['age_s'] <= max_age for r in rows)
            checks[key+' monotonic source stamps'] = bool(rows) and all(b['stamp'] > a['stamp'] for a, b in zip(rows, rows[1:]))
            checks[key+' no 0.4 second receipt gap'] = bool(gaps(rows)) and max(gaps(rows)) < .4
        lio_stamps = {r['stamp'] for r in data['lio']}
        checks['relay preserves actual LIO source stamps'] = bool(stable['relay']) and all(r['stamp'] in lio_stamps for r in stable['relay'])
        for key, frame in [('lio', 'camera_init'), ('relay', 'map')]:
            rows = stable[key]
            checks[key+' finite normalized pose and frame'] = bool(rows) and all(
                r['frame'] == frame and all(math.isfinite(v) for v in r['pose'])
                and abs(sum(v*v for v in r['pose'][3:])-1) < .01 for r in rows)
        states = stable['state']
        checks['FCU stays connected and disarmed'] = bool(states) and all(r['connected'] and not r['armed'] and r['mode'] != 'OFFBOARD' for r in states)
        ekf = stable['ekf']
        checks['fresh sustained EKF status'] = len(ekf) >= (args.seconds-5)*.8 and all(-.02 <= r['age_s'] <= 1.5 for r in ekf)
        for flag in ('attitude_status_flag', 'pos_horiz_rel_status_flag', 'pos_vert_abs_status_flag'):
            checks['EKF '+flag] = bool(ekf) and all(r['flags'].get(flag) is True for r in ekf)
        summary = {key: {'count': len(rows), 'age_s': distribution([r['age_s'] for r in rows]),
                         'prepublish_age_s': distribution([r['prepublish_age_s'] for r in rows]),
                         'publish_to_callback_s': distribution([r['publish_to_callback_s'] for r in rows]),
                         'gaps_s': distribution(gaps(rows)),
                         'hz': (len(rows)-1)/(rows[-1]['at']-rows[0]['at']) if len(rows)>1 else None}
                   for key, rows in stable.items()}
        summary['ekf']['flags'] = dict(Counter(json.dumps(r['flags'], sort_keys=True) for r in ekf))
        print(json.dumps({'passed': all(checks.values()), 'checks': checks, 'summary': summary,
                          'data': data, 'scope': 'existing ROS2 ground session, passive subscriptions only; battery and RC excluded'}, allow_nan=False), flush=True)
        return 0 if all(checks.values()) else 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
