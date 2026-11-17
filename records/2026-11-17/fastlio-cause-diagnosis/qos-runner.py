#!/usr/bin/env python3
"""Compare IMU QoS in private real-input LIO clones; pause only each clone.

Existing nodes, services, FCU parameters and outputs are untouched. Every clone
output and TF topic is private. SIGSTOP/CONT/INT target only our new process group.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import uuid

import rclpy
from rclpy.executors import SingleThreadedExecutor, await_or_execute
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, EstimatorStatus


class MetadataExecutor(SingleThreadedExecutor):
    # Match the installed early Humble executor, forwarding existing MessageInfo.
    def _take_subscription(self, sub):
        with sub.handle:
            return sub.handle.take_message(sub.msg_type, sub.raw)

    async def _execute_subscription(self, sub, value):
        if value is not None:
            await await_or_execute(sub.callback, *value)


def distribution(values):
    values = sorted(v for v in values if math.isfinite(v))
    return {name: values[round(q * (len(values)-1))]
            for name, q in [('min', 0.), ('median', .5), ('p95', .95), ('max', 1.)]} if values else None


def proc_sample(pid):
    tasks = {}
    for path in Path('/proc').joinpath(str(pid), 'task').glob('*/schedstat'):
        try:
            tasks[path.parent.name] = [int(v) for v in path.read_text().split()]
        except (OSError, ValueError):
            pass
    return tasks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=45.)
    args = parser.parse_args()
    if args.root.parent != Path('/tmp') or not args.root.name.startswith('fastlio-cause-'):
        parser.error('unique /tmp/fastlio-cause-* directory required')
    if not 25 <= args.seconds <= 60:
        parser.error('phase duration must be 25..60 seconds')
    args.root.mkdir(exist_ok=False)
    platform = Path.home()/'drone_platform'
    original = platform/'01_apps/fast_lio/workspace/install/fast_lio/lib/fast_lio/fastlio_mapping'
    assert hashlib.sha256(original.read_bytes()).hexdigest() == '7707f011a357aa241b11eb098ce86fdbe6e9a73de762c36a960db6937810cbcd'
    active = json.loads(Path('/tmp/drone-localization-1000/localization.json').read_text())
    assert active['pid'] == 73086 and Path('/proc/73150/exe').resolve() == original
    rclpy.init()
    node = rclpy.create_node('fastlio_cause_observer_'+uuid.uuid4().hex[:8], enable_rosout=False)
    executor = MetadataExecutor()
    executor.add_node(node)
    began = time.monotonic()
    wall = time.time()
    observations = {k: [] for k in ('imu', 'actual', 'actual_relay', 'clone', 'fc', 'ekf')}
    system = []
    phase = {'label': 'waiting', 'begin': began, 'gap': False}

    def receive(key):
        def callback(message, info):
            at = time.monotonic()
            now = node.get_clock().now().nanoseconds
            stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
            published = info.get('source_timestamp', 0) if isinstance(info, dict) else info.source_timestamp
            row = dict(at=at-began, phase=phase['label'], phase_at=at-phase['begin'],
                       stamp_ns=stamp, age_s=(now-stamp)*1e-9,
                       prepublish_age_s=(published-stamp)*1e-9 if published else None,
                       pub_callback_s=(now-published)*1e-9 if published else None)
            if key in ('actual', 'clone'):
                p, q = message.pose.pose.position, message.pose.pose.orientation
                row['pose'] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            elif key == 'fc':
                row['values'] = dict(connected=message.connected, armed=message.armed, mode=message.mode)
                if message.armed:
                    raise RuntimeError('FCU armed; ending only the experiment')
            elif key == 'ekf':
                row['values'] = {f: getattr(message, f) for f in message.get_fields_and_field_types() if f.endswith('_flag')}
            if key == 'imu':
                paused = phase['gap'] and 10. <= row['phase_at'] <= 11.4
                row['forwarded'] = False
            observations[key].append(row)
        return callback

    # PoseStamped has pose directly; Odometry has pose.pose.
    def relay_callback(message, info):
        now = node.get_clock().now().nanoseconds
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        published = info.get('source_timestamp', 0) if isinstance(info, dict) else info.source_timestamp
        at = time.monotonic()
        observations['actual_relay'].append(dict(at=at-began, phase=phase['label'], phase_at=at-phase['begin'],
                                                 stamp_ns=stamp, age_s=(now-stamp)*1e-9,
                                                 prepublish_age_s=(published-stamp)*1e-9 if published else None,
                                                 pub_callback_s=(now-published)*1e-9 if published else None))

    subscriptions = [
        node.create_subscription(Imu, '/livox/imu', receive('imu'), qos_profile_sensor_data),
        node.create_subscription(Odometry, '/factory_fastlio/odometry', receive('actual'), qos_profile_sensor_data),
        node.create_subscription(PoseStamped, '/mavros/vision_pose/pose', relay_callback, qos_profile_sensor_data),
        node.create_subscription(Odometry, '/gcs_cause_diag/odometry', receive('clone'), qos_profile_sensor_data),
        node.create_subscription(State, '/mavros/state', receive('fc'), qos_profile_sensor_data),
        node.create_subscription(EstimatorStatus, '/mavros/estimator_status', receive('ekf'), qos_profile_sensor_data),
    ]
    process = None
    results = {}
    last_system = -1.
    try:
        ready = time.monotonic()+8
        while time.monotonic()<ready:
            executor.spin_once(timeout_sec=.02)
            if observations['fc'] and observations['imu']:
                break
        if not observations['fc'] or not observations['fc'][-1]['values']['connected'] or not observations['imu']:
            raise RuntimeError('live sensors and connected/disarmed FCU required')
        binary = Path('/tmp/fastlio-cause-20261117-180200-7e3e69/probe_build/fastlio_mapping')
        for label, best_effort in [('reliable_1', False), ('best_effort', True), ('reliable_2', False)]:
            phase.update(label=label, begin=time.monotonic(), gap=False)
            command = [str(binary), '--ros-args', '--params-file', str(platform/'01_apps/fast_lio/config/lidar.yaml'),
                       '-r', '__ns:=/gcs_cause_diag', '-r', '__node:=cause_mapping', '-p', 'common.imu_topic:=/livox/imu', '-p', 'diag_imu_best_effort:='+str(best_effort).lower()]
            for source, target in {'/Odometry':'odometry', '/cloud_registered':'cloud_world',
                                   '/cloud_registered_body':'cloud_body', '/cloud_effected':'effect',
                                   '/Laser_map':'map', '/path':'path', '/tf':'tf', '/tf_static':'tf_static'}.items():
                command += ['-r', source+':=/gcs_cause_diag/'+target]
            with (args.root/(label+'.log')).open('w') as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True, cwd=args.root)
                assert process.pid not in (73086, 73150, 73273)
                print(json.dumps(dict(event="clone_start", label=label, pid=process.pid, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())), flush=True)
                clone_stopped = False
                stalled_once = False
                try:
                    while time.monotonic()-phase['begin']<args.seconds:
                        elapsed = time.monotonic()-phase['begin']
                        if elapsed >= 10. and not stalled_once:
                            os.killpg(process.pid,signal.SIGSTOP)
                            clone_stopped=True;stalled_once=True
                            print(json.dumps(dict(event='pause_private_clone',pid=process.pid,at=elapsed)),flush=True)
                        if elapsed >= 11.4 and clone_stopped:
                            os.killpg(process.pid,signal.SIGCONT)
                            clone_stopped=False
                            print(json.dumps(dict(event='resume_private_clone',pid=process.pid,at=elapsed)),flush=True)
                        executor.spin_once(timeout_sec=.02)
                        if process.poll() is not None:
                            raise RuntimeError('clone exited unexpectedly')
                        at = time.monotonic()-began
                        if at-last_system>=.5:
                            system.append(dict(at=at, phase=label, clone_pid=process.pid,
                                               original_schedstat=proc_sample(73150),
                                               clone_schedstat=proc_sample(process.pid),
                                               temperatures={p.parent.name:int(p.read_text()) for p in Path('/sys/class/thermal').glob('thermal_zone*/temp')},
                                               scaling_max={p.parent.name:int(p.read_text()) for p in Path('/sys/devices/system/cpu/cpufreq').glob('policy*/scaling_max_freq')},
                                               load=Path('/proc/loadavg').read_text().strip()))
                            last_system=at
                finally:
                    if clone_stopped and process.poll() is None:
                        os.killpg(process.pid,signal.SIGCONT)
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=3)
            text = (args.root/(label+'.log')).read_text()
            result = dict(imu_qos='best_effort' if best_effort else 'reliable', depth=10, exit_code=process.returncode,
                          detected_offsets=[float(v) for v in re.findall(r'Self sync IMU and LiDAR, time diff is ([-\d.]+)', text)],
                          verified_alignment='Source clock alignment verified' in text, windows={})
            for window, low, high in [('before', 3., 10.), ('after', 16., args.seconds)]:
                stream = [r for r in observations['clone'] if r['phase']==label and low<=r['phase_at']<high]
                result['windows'][window] = dict(count=len(stream),
                    hz=(len(stream)-1)/(stream[-1]['at']-stream[0]['at']) if len(stream)>1 else None,
                    age_s=distribution([r['age_s'] for r in stream]),
                    prepublish_age_s=distribution([r['prepublish_age_s'] for r in stream if r['prepublish_age_s'] is not None]),
                    receipt_gap_s=distribution([b['at']-a['at'] for a,b in zip(stream,stream[1:])]),
                    over_0_2_s=sum(r['age_s']>.2 for r in stream))
            results[label]=result
            (args.root/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
            print(json.dumps(dict(event='phase', label=label, result=result)),flush=True)
            phase.update(label='waiting', begin=time.monotonic(), gap=False)
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.02)
        (args.root/'observations.json').write_text(json.dumps(dict(started_wall=wall,observations=observations,system=system)))
        print(json.dumps(dict(event='finished',root=str(args.root))),flush=True)
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=8)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
