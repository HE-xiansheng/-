#!/usr/bin/env python3
"""Read existing small ROS2 topics only; print observations to the SSH stdout."""
import argparse
from collections import Counter
import json
import math
import time
import uuid

import rclpy
from rclpy.executors import SingleThreadedExecutor, await_or_execute
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, BatteryState
from mavros_msgs.msg import State, EstimatorStatus, RCIn


class MetadataExecutor(SingleThreadedExecutor):
    """Installed Humble discards MessageInfo before calling subscriptions.

    Keep its native take result for this observer only. No installed rclpy or
    running application is changed. Both hooks match the inspected installation.
    """
    def _take_subscription(self, sub):
        with sub.handle:
            return sub.handle.take_message(sub.msg_type, sub.raw)

    async def _execute_subscription(self, sub, msg_info):
        if msg_info is not None:
            await await_or_execute(sub.callback, *msg_info)


def distribution(values):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    if not values:
        return None
    return {name: values[round((len(values) - 1) * q)]
            for name, q in [('min', 0), ('median', .5), ('p95', .95), ('max', 1)]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 5 <= args.seconds <= 60:
        parser.error('seconds must be between 5 and 60')
    definitions = {
        'imu': ('/livox/imu', Imu),
        'lio': ('/factory_fastlio/odometry', Odometry),
        'relay': ('/mavros/vision_pose/pose', PoseStamped),
        'fc_odom': ('/mavros/local_position/odom', Odometry),
        'fc': ('/mavros/state', State),
        'ekf': ('/mavros/estimator_status', EstimatorStatus),
        'rc': ('/mavros/rc/in', RCIn),
        'battery': ('/mavros/battery', BatteryState),
    }
    rclpy.init()
    node = rclpy.create_node('preflight_small_readonly_' + uuid.uuid4().hex[:8],
                             enable_rosout=False, start_parameter_services=False)
    rows = {key: [] for key in definitions}
    snapshots = []
    began = time.monotonic()
    wall = time.time()

    def callback(key):
        def receive(message, info):
            now = node.get_clock().now().nanoseconds
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            source = info.get('source_timestamp', 0) if isinstance(info, dict) else info.source_timestamp
            received = info.get('received_timestamp', 0) if isinstance(info, dict) else info.received_timestamp
            row = dict(at=time.monotonic() - began, stamp_ns=stamp,
                       source_age_s=(now - stamp) * 1e-9,
                       publish_measurement_age_s=(source - stamp) * 1e-9 if source else None,
                       publish_to_callback_s=(now - source) * 1e-9 if source else None,
                       publish_to_dds_receive_s=(received - source) * 1e-9 if source and received else None,
                       dds_receive_to_callback_s=(now - received) * 1e-9 if received else None)
            if key == 'ekf':
                row['values'] = {field: getattr(message, field)
                                 for field in message.get_fields_and_field_types() if field.endswith('_flag')}
            elif key == 'fc':
                row['values'] = dict(connected=message.connected, armed=message.armed, mode=message.mode)
            elif key == 'battery':
                row['values'] = {field: (getattr(message, field) if math.isfinite(getattr(message, field)) else None)
                                 for field in ('voltage', 'percentage')}
            elif key == 'rc':
                row['values'] = dict(rssi=message.rssi, channels=len(message.channels))
            rows[key].append(row)
        return receive

    try:
        subscriptions = [node.create_subscription(typ, topic, callback(key), qos_profile_sensor_data)
                         for key, (topic, typ) in definitions.items()]
        executor = MetadataExecutor()
        executor.add_node(node)
        next_snapshot = 3.
        while time.monotonic() - began < args.seconds:
            executor.spin_once(timeout_sec=.05)
            at = time.monotonic() - began
            if at >= next_snapshot:
                now = node.get_clock().now().nanoseconds
                latest = {}
                for key in ('imu', 'lio', 'relay', 'fc_odom'):
                    if rows[key]:
                        last = rows[key][-1]
                        latest[key] = dict(source_age_s=(now - last['stamp_ns']) * 1e-9,
                                           receipt_age_s=at - last['at'])
                snapshots.append(dict(at=at, topics=latest))
                next_snapshot = at + .2
        summary = {}
        for key, samples in rows.items():
            summary[key] = dict(topic=definitions[key][0], count=len(samples))
            if samples:
                summary[key].update(
                    received_hz=(len(samples)-1)/(samples[-1]['at']-samples[0]['at']) if len(samples)>1 else None,
                    source_age_s=distribution([r['source_age_s'] for r in samples]),
                    publish_measurement_age_s=distribution([r['publish_measurement_age_s'] for r in samples]),
                    publish_to_callback_s=distribution([r['publish_to_callback_s'] for r in samples]),
                    publish_to_dds_receive_s=distribution([r['publish_to_dds_receive_s'] for r in samples]),
                    dds_receive_to_callback_s=distribution([r['dds_receive_to_callback_s'] for r in samples]),
                    receipt_gap_s=distribution([b['at']-a['at'] for a,b in zip(samples,samples[1:])]),
                    age_over_0_2_s=sum(r['source_age_s']>.2 for r in samples),
                    non_increasing_stamp=sum(b['stamp_ns']<=a['stamp_ns'] for a,b in zip(samples,samples[1:])),
                    values=dict(Counter(json.dumps(r['values'],sort_keys=True) for r in samples if 'values' in r)))
            if key in ('imu', 'lio', 'relay', 'fc_odom'):
                latest = [s['topics'].get(key, {}) for s in snapshots]
                summary[key]['display_fresh_0_4_s'] = sum(
                    -.1 <= r.get('source_age_s', math.inf) <= .4 and 0 <= r.get('receipt_age_s', math.inf) <= .4
                    for r in latest)
                summary[key]['display_snapshots'] = len(latest)
            summary[key]['endpoints'] = [dict(node=e.node_namespace.rstrip('/')+'/'+e.node_name,
                                             reliability=str(e.qos_profile.reliability),
                                             durability=str(e.qos_profile.durability),
                                             depth=e.qos_profile.depth)
                                         for e in node.get_publishers_info_by_topic(definitions[key][0])]
        print(json.dumps(dict(started_wall=wall, duration_s=time.monotonic()-began,
                              summary=summary, observations=rows, snapshots=snapshots), allow_nan=False))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
