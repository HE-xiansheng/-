#!/usr/bin/env python3
"""Check private real-input LIO output and recovery without touching the FCU."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor, await_or_execute
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from rcl_interfaces.srv import GetParameters


class MetadataExecutor(SingleThreadedExecutor):
    # The installed early Humble executor discards existing MessageInfo. Forward
    # it only in this observer; the installed ROS libraries are unchanged.
    def _take_subscription(self, sub):
        with sub.handle:
            return sub.handle.take_message(sub.msg_type, sub.raw)

    async def _execute_subscription(self, sub, value):
        if value is not None:
            await await_or_execute(sub.callback, *value)


def distribution(values):
    values = sorted(values)
    return {name: values[round(q*(len(values)-1))]
            for name, q in [('min', 0), ('median', .5), ('p95', .95), ('max', 1)]} if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    assert args.root.parent == Path('/tmp') and args.root.name.startswith('fastlio-receive-clock-test-')
    assert args.binary.is_file() and args.config.is_file()
    assert os.environ.get('ROS_DOMAIN_ID', '0') == '0'
    assert json.loads(Path('/tmp/drone-localization-1000/localization.json').read_text())['pid'] == 73086
    assert Path('/proc/73150').exists() and Path('/proc/73273').exists()
    args.root.mkdir(exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('receive_clock_candidate_observer', enable_rosout=False)
    executor = MetadataExecutor()
    executor.add_node(node)
    phase = {'name': 'waiting', 'start': time.monotonic()}
    data = {key: [] for key in ['imu', 'odometry', 'odometry_reliable', 'cloud_world', 'cloud_body', 'state']}
    malformed, subscriptions, results = [], [], {}
    process = None

    def metadata(info, stamp):
        now = time.time_ns()
        pub = info.get('source_timestamp', 0) if isinstance(info, dict) else info.source_timestamp
        received = info.get('received_timestamp', 0) if isinstance(info, dict) else info.received_timestamp
        return {'published_ns': pub, 'rmw_received_ns': received,
                'prepublish_age_s': (pub-stamp)*1e-9 if pub else None,
                'publish_to_callback_s': (now-pub)*1e-9 if pub else None}

    def record(key, message, info):
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        row = {'at': time.monotonic(), 'phase': phase['name'],
               'phase_at': time.monotonic()-phase['start'], 'stamp': stamp,
               'age_s': (time.time_ns()-stamp)*1e-9}
        row.update(metadata(info, stamp))
        if key in ('odometry', 'odometry_reliable'):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            row.update(pose=[p.x, p.y, p.z, q.x, q.y, q.z, q.w],
                       frame=message.header.frame_id, child=message.child_frame_id)
        data[key].append(row)

    def cloud(key):
        def receive(raw, info):
            try:
                assert raw[:4] == b'\x00\x01\x00\x00'
                sec, nsec, length = struct.unpack_from('<iII', raw, 4)
                assert 0 < length < 256 and nsec < 10**9
                frame = raw[16:16+length-1].decode()
                height, width = struct.unpack_from('<II', raw, (16+length+3)&~3)
                stamp = sec*10**9+nsec
                data[key].append({'at': time.monotonic(), 'phase': phase['name'],
                                  'phase_at': time.monotonic()-phase['start'], 'stamp': stamp,
                                  'age_s': (time.time_ns()-stamp)*1e-9,
                                  'frame': frame, 'points': height*width, **metadata(info, stamp)})
            except Exception as error:
                malformed.append({'topic': key, 'error': str(error)})
        return receive

    def state(message, info):
        data['state'].append({'at': time.monotonic(), 'connected': message.connected, 'armed': message.armed})
        if message.armed:
            raise RuntimeError('FCU armed; stopping only our private experiment')

    # Verify complete output delivery with reliable readers. The real-time IMU
    # input policy is independent and is checked on the candidate's endpoint.
    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
    subscriptions += [
        node.create_subscription(Imu, '/livox/imu', lambda m, i: record('imu', m, i), qos_profile_sensor_data),
        node.create_subscription(Odometry, '/gcs_receive_fix/odometry', lambda m, i: record('odometry', m, i), qos_profile_sensor_data),
        node.create_subscription(Odometry, '/gcs_receive_fix/odometry', lambda m, i: record('odometry_reliable', m, i), qos),
        node.create_subscription(PointCloud2, '/gcs_receive_fix/cloud_world', cloud('cloud_world'), qos, raw=True),
        node.create_subscription(PointCloud2, '/gcs_receive_fix/cloud_body', cloud('cloud_body'), qos, raw=True),
        node.create_subscription(State, '/mavros/state', state, qos_profile_sensor_data),
    ]
    try:
        deadline = time.monotonic()+8
        while time.monotonic() < deadline and not (data['state'] and data['imu']):
            executor.spin_once(timeout_sec=.02)
        assert data['state'] and data['state'][-1]['connected'] and not data['state'][-1]['armed']
        assert data['imu'] and data['imu'][-1]['age_s'] < .1
        for label, seconds, pause in [('baseline', 90, False), ('stall_recovery', 60, True)]:
            command = [str(args.binary), '--ros-args', '--params-file', str(args.config),
                       '-r', '__ns:=/gcs_receive_fix', '-r', '__node:=receive_mapping']
            for source, target in {'/Odometry': 'odometry', '/cloud_registered': 'cloud_world',
                                   '/cloud_registered_body': 'cloud_body', '/cloud_effected': 'effect',
                                   '/Laser_map': 'map', '/path': 'path', '/tf': 'tf', '/tf_static': 'tf_static'}.items():
                command += ['-r', source+':=/gcs_receive_fix/'+target]
            phase.update(name=label, start=time.monotonic())
            stopped, paused, endpoints = False, False, []
            parameters, parameter_future = {}, None
            parameter_client = node.create_client(GetParameters, '/gcs_receive_fix/receive_mapping/get_parameters')
            with (args.root/(label+'.log')).open('w') as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True, cwd=args.root)
                assert process.pid not in (73086, 73150, 73273)
                print(json.dumps({'event': 'started', 'phase': label, 'pid': process.pid}), flush=True)
                try:
                    while time.monotonic()-phase['start'] < seconds:
                        at = time.monotonic()-phase['start']
                        if pause and at >= 15 and not paused:
                            os.killpg(process.pid, signal.SIGSTOP)
                            stopped, paused = True, True
                        if stopped and at >= 16.4:
                            os.killpg(process.pid, signal.SIGCONT)
                            stopped = False
                        executor.spin_once(timeout_sec=.01)
                        assert process.poll() is None, 'candidate exited early'
                        if not endpoints and at >= 3:
                            endpoints = [{'node': i.node_name, 'namespace': i.node_namespace,
                                          'reliability': str(i.qos_profile.reliability), 'depth': i.qos_profile.depth}
                                         for i in node.get_subscriptions_info_by_topic('/livox/imu')
                                         if i.node_name == 'receive_mapping' and i.node_namespace == '/gcs_receive_fix']
                        if at >= 3 and parameter_future is None and parameter_client.service_is_ready():
                            request = GetParameters.Request()
                            request.names = ['common.imu_queue_depth', 'common.time_sync_en', 'common.time_offset_lidar_to_imu']
                            parameter_future = parameter_client.call_async(request)
                        if parameter_future is not None and parameter_future.done() and not parameters:
                            response = parameter_future.result()
                            if response is not None:
                                parameters = {name: {'type': value.type, 'integer': value.integer_value,
                                                     'boolean': value.bool_value, 'double': value.double_value}
                                              for name, value in zip(request.names, response.values)}
                finally:
                    if stopped and process.poll() is None:
                        os.killpg(process.pid, signal.SIGCONT)
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=12)
            node.destroy_client(parameter_client)
            text = (args.root/(label+'.log')).read_text()
            def settled(row):
                return row['phase'] == label and row['phase_at'] >= 5 and (not pause or row['phase_at'] < 15 or row['phase_at'] >= 20)
            rows = [r for r in data['odometry'] if settled(r)]
            stamps = {r['stamp'] for key in ('odometry', 'odometry_reliable') for r in data[key] if r['phase'] == label}
            clouds = {key: [r for r in data[key] if settled(r)] for key in ['cloud_world', 'cloud_body']}
            gaps = [b['at']-a['at'] for a, b in zip(rows, rows[1:])
                    if not pause or not (a['phase_at'] < 15 and b['phase_at'] >= 20)]
            recovered = [r for r in data['odometry'] if r['phase'] == label and r['phase_at'] >= 16.4 and r['age_s'] <= .2]
            checks = {
                'clock alignment locked without fabricated shift': 'Source clock alignment verified' in text and 'Self sync IMU and LiDAR' not in text,
                'best effort endpoint and runtime configured depth 400': (
                    any(e['reliability'] == str(ReliabilityPolicy.BEST_EFFORT) and e['depth'] in (0, 400) for e in endpoints)
                    and parameters.get('common.imu_queue_depth', {}).get('integer') == 400
                    and 'IMU sensor QoS: best_effort, keep_last 400' in text),
                'original timing parameters unchanged': (
                    parameters.get('common.time_sync_en', {}).get('boolean') is True
                    and parameters.get('common.time_offset_lidar_to_imu', {}).get('double') == 0),
                'sustained new source measurements': len(rows) >= (seconds-10-(5 if pause else 0))*8,
                'stable output meets original 0.2s relay gate': bool(rows) and all(-.02 <= r['age_s'] <= .2 for r in rows),
                'stable receipt gaps below 0.4s display and PX4 timeout': bool(gaps) and max(gaps) < .4,
                'measurement stamps strictly increase': bool(rows) and all(b['stamp'] > a['stamp'] for a, b in zip(rows, rows[1:])),
                'finite poses and original coordinate frames': bool(rows) and all(r['frame'] == 'camera_init' and r['child'] == 'body' and all(math.isfinite(v) for v in r['pose']) and abs(sum(v*v for v in r['pose'][3:])-1) < .01 for r in rows),
                'both cloud streams match odometry source stamps': all(len(v) >= len(rows)-2 and all(c['stamp'] in stamps and c['points'] > 0 and c['frame'] == ('body' if key == 'cloud_body' else 'camera_init') for c in v) for key, v in clouds.items()),
                'both cloud streams meet original 0.4s display age': all(v and all(-.02 <= c['age_s'] <= .4 for c in v) for v in clouds.values()),
                'clean exit': process.returncode == 0,
                'no malformed raw cloud headers': not malformed,
            }
            if pause:
                checks['fresh output recovers within 1 second after private SIGCONT'] = bool(recovered) and recovered[0]['phase_at']-16.4 < 1
            result = {'passed': all(checks.values()), 'checks': checks, 'count': len(rows),
                      'ages_s': distribution([r['age_s'] for r in rows]), 'gaps_s': distribution(gaps),
                      'recovery_s': recovered[0]['phase_at']-16.4 if pause and recovered else None,
                      'endpoints': endpoints, 'parameters': parameters, 'returncode': process.returncode,
                      'publication_mode': os.environ.get('RMW_FASTRTPS_PUBLICATION_MODE', 'unset')}
            result['streams'] = {key: {'count': len(v), 'age_s': distribution([r['age_s'] for r in v]),
                                       'prepublish_age_s': distribution([r['prepublish_age_s'] for r in v if r['prepublish_age_s'] is not None]),
                                       'publish_to_callback_s': distribution([r['publish_to_callback_s'] for r in v if r['publish_to_callback_s'] is not None])}
                                 for key in ['odometry', 'odometry_reliable', 'cloud_world', 'cloud_body']
                                 for v in [[r for r in data[key] if settled(r)]]}
            results[label] = result
            (args.root/'result.json').write_text(json.dumps(results, indent=2)+'\n')
            print(json.dumps({'event': 'phase_result', 'phase': label, 'result': result}), flush=True)
        return 0 if all(v['passed'] for v in results.values()) else 1
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGCONT)
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=12)
        (args.root/'observations.json').write_text(json.dumps({'data': data, 'malformed': malformed,
            'binary_sha256': hashlib.sha256(args.binary.read_bytes()).hexdigest()}))
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
