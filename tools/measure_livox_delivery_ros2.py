#!/usr/bin/env python3
"""只读采样原始Livox交付与正式里程计；不启动/停止定位或接触飞控串口。"""
import argparse
import json
import struct
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor, await_or_execute
from rclpy.qos import QoSProfile, ReliabilityPolicy
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class MetadataExecutor(SingleThreadedExecutor):
    def _take_subscription(self, sub):
        with sub.handle:
            return sub.handle.take_message(sub.msg_type, sub.raw)

    async def _execute_subscription(self, sub, value):
        if value is not None:
            await await_or_execute(sub.callback, *value)


def quantiles(values):
    values = sorted(values)
    return {name: values[round(q * (len(values) - 1))]
            for name, q in [('min', 0), ('median', .5), ('p95', .95), ('max', 1)]} if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=40)
    args = parser.parse_args()
    assert 20 <= args.seconds <= 120
    rclpy.init()
    node = rclpy.create_node('livox_readonly_delivery_probe', enable_rosout=False)
    executor = MetadataExecutor()
    executor.add_node(node)
    data = {key: [] for key in ['lidar_be', 'lidar_reliable', 'imu', 'formal_odometry']}
    errors = []
    start = time.monotonic()

    def receive(key):
        def callback(raw, info):
            now_ns, at = time.time_ns(), time.monotonic() - start
            try:
                assert raw[:4] == b'\x00\x01\x00\x00'
                sec, ns, length = struct.unpack_from('<iII', raw, 4)
                assert ns < 10**9 and 0 < length < 256
                stamp = sec * 10**9 + ns
                points, duration = None, 0
                if key.startswith('lidar'):
                    base = 4 + ((16 + length - 4 + 7) // 8) * 8
                    points = struct.unpack_from('<I', raw, base + 8)[0]
                    assert struct.unpack_from('<I', raw, base + 16)[0] == points
                    assert len(raw) == base + 20 + max(0, points * 20 - 1)
                    duration = int(np.ndarray((points,), dtype='<u4', buffer=raw,
                                              offset=base + 20, strides=(20,)).max()) if points else 0
                end = stamp + duration
                pub = info.get('source_timestamp', 0) if isinstance(info, dict) else info.source_timestamp
                data[key].append(dict(at=at, stamp=stamp, end_stamp=end, points=points,
                                      serialized_bytes=len(raw), age_s=(now_ns-end)*1e-9,
                                      prepublish_age_s=(pub-end)*1e-9 if pub else None,
                                      publish_to_callback_s=(now_ns-pub)*1e-9 if pub else None))
            except Exception as error:
                errors.append(dict(key=key, at=at, error=repr(error)))
        return callback

    reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    best_effort = QoSProfile(depth=400, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = [
        node.create_subscription(CustomMsg, '/livox/lidar', receive('lidar_be'), best_effort, raw=True),
        node.create_subscription(CustomMsg, '/livox/lidar', receive('lidar_reliable'), reliable, raw=True),
        node.create_subscription(Imu, '/livox/imu', receive('imu'), best_effort, raw=True),
        node.create_subscription(Odometry, '/factory_fastlio/odometry', receive('formal_odometry'), best_effort, raw=True),
    ]
    try:
        while time.monotonic() - start < args.seconds + 5:
            executor.spin_once(timeout_sec=.01)
        endpoints = []
        for entry in node.get_publishers_info_by_topic('/livox/lidar'):
            endpoints.append(dict(node=entry.node_name, reliability=str(entry.qos_profile.reliability),
                                  depth=entry.qos_profile.depth))
        summary = {}
        for key, rows in data.items():
            rows = [row for row in rows if row['at'] >= 5]
            summary[key] = dict(count=len(rows), hz=(len(rows)-1)/(rows[-1]['at']-rows[0]['at']) if len(rows)>1 else 0,
                                age_s=quantiles([r['age_s'] for r in rows]),
                                gaps_s=quantiles([b['at']-a['at'] for a,b in zip(rows,rows[1:])]),
                                source_gaps_s=quantiles([(b['end_stamp']-a['end_stamp'])*1e-9 for a,b in zip(rows,rows[1:])]),
                                prepublish_age_s=quantiles([r['prepublish_age_s'] for r in rows if r['prepublish_age_s'] is not None]),
                                publish_to_callback_s=quantiles([r['publish_to_callback_s'] for r in rows if r['publish_to_callback_s'] is not None]))
        print(json.dumps(dict(scope='read-only existing inputs; no clone or control publishers',
                              seconds=args.seconds, summary=summary, endpoints=endpoints,
                              malformed=errors, data=data)), flush=True)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
