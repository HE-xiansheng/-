#!/usr/bin/env python3
"""Real Livox/LIO/relay -> installed MAVROS -> disarmed physical PX4, ground only."""
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, TimesyncStatus
import sensor_readiness
from px4_ground_link import GroundLink, TOPICS

root = Path('/tmp/px4-ev-ground-check')
platform = Path('/home/orangepi/drone_platform')
assert os.environ['ROS_DOMAIN_ID'] == '187' and os.environ['ROS_LOCALHOST_ONLY'] == '1'
processes = []
logs = []
link = None
checks = {}
states = []
poses = {key: [] for key in ('odom', 'vision', 'local')}
sync = []
report = {'passed': False, 'scope': 'real native sensors and physical PX4, static disarmed EV ground check'}
extended = '--extended-sync' in sys.argv
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = rclpy.create_node('physical_ev_ground_observer')


def pose(kind, message):
    p = message.pose.pose if kind == 'odom' else message.pose
    stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
    poses[kind].append({'stamp_ns': stamp, 'receipt_ns': time.time_ns(),
                        'position': [p.position.x, p.position.y, p.position.z],
                        'q_xyzw': [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w],
                        'frame': message.header.frame_id})


subs = [node.create_subscription(typ, topic, lambda m, k=kind: pose(k, m),
                                qos_profile_sensor_data if kind == 'local' else 200)
        for typ, topic, kind in [(Odometry, '/factory_fastlio/odometry', 'odom'),
                                 (PoseStamped, '/mavros/vision_pose/pose', 'vision'),
                                 (PoseStamped, '/mavros/local_position/pose', 'local')]]
subs.append(node.create_subscription(State, '/mavros/state', lambda m: states.append({
    'receipt_ns': time.time_ns(), 'connected': m.connected, 'armed': m.armed, 'mode': m.mode}), qos_profile_sensor_data))
subs.append(node.create_subscription(TimesyncStatus, '/mavros/timesync_status', lambda m: sync.append({
    slot.lstrip('_'): getattr(m, slot) for slot in m.__slots__ if slot != '_header'}), qos_profile_sensor_data))


def start(name, command):
    log = (root/(name+'.log')).open('w')
    logs.append(log)
    child = subprocess.Popen(command, cwd=root, stdin=subprocess.DEVNULL, stdout=log,
                             stderr=subprocess.STDOUT, start_new_session=True)
    processes.append((name, child))
    (root/'owned-processes.json').write_text(json.dumps([
        {'name': n, 'pid': p.pid, 'start_ticks': Path('/proc/'+str(p.pid)+'/stat').read_text().split()[21]}
        for n, p in processes], indent=2)+'\n')


def check(name, value):
    checks[name] = bool(value)
    print(json.dumps({name: bool(value)}), flush=True)


def fields(query):
    text = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', query['text']).replace('\r', '\n')
    if 'TOPIC: '+query['topic'] not in text:
        return {}
    output = {}
    for key, value in re.findall(r'^\s*([a-zA-Z_][a-zA-Z_0-9]*):\s*([^\n]+)', text, re.M):
        value = value.strip()
        if value.lower() in ('true', 'false'):
            output[key] = value.lower() == 'true'
        elif re.match(r'^-?\d+(?:\.\d+)?', value):
            output[key] = float(re.match(r'^-?\d+(?:\.\d+)?', value).group())
        else:
            output[key] = value
    return output


try:
    start('driver', ['ros2', 'launch', str(platform/'02_drivers/sensors/launch/livox_mid360s.launch.py')])
    readiness = sensor_readiness.wait_for_sensors()
    (root/'sensor-readiness.json').write_text(json.dumps(readiness, indent=2)+'\n')
    start('lio', ['ros2', 'launch', str(platform/'01_apps/fast_lio/launch/localization.launch.py')])
    start('relay', ['ros2', 'run', 'to_mavros', 'vision2mavros'])
    link = GroundLink('/dev/serial/by-id/usb-Matek_HKUST_UAV_NxtPX4_0-if00', root/'common.py')
    start('mavros', ['ros2', 'launch', str(platform/'02_drivers/sensors/launch/mavros_platform.launch.py'),
                     'fcu_url:=udp://127.0.0.1:14650@127.0.0.1:14651'])
    begin = time.monotonic()
    query_schedule = [(offset+2*i, topic) for offset in ((100, 125) if extended else (25, 50)) for i, topic in enumerate(TOPICS)]
    query_index = 0
    print('PHYSICAL_EV_GROUND_RUNNING', flush=True)
    while time.monotonic()-begin < (145 if extended else 75):
        for name, child in processes:
            if child.poll() is not None:
                raise RuntimeError(name+' exited '+str(child.returncode))
        link.step()
        if states and states[-1]['armed']:
            raise RuntimeError('armed ROS state; stopping ground test')
        elapsed = time.monotonic()-begin
        if query_index < len(query_schedule) and elapsed >= query_schedule[query_index][0]:
            link.send_query(query_schedule[query_index][1])
            query_index += 1
        rclpy.spin_once(node, timeout_sec=.001)
    check('physical_PX4_connected_disarmed', bool(states) and any(s['connected'] for s in states)
          and all(not s['armed'] and s['mode'] != 'OFFBOARD' for s in states))
    wire = [m for m in link.outgoing if m['mavpackettype'] == 'VISION_POSITION_ESTIMATE']
    sources = {p['stamp_ns']//1000: p for p in poses['vision']}
    check('sustained_actual_vision_packets', len(wire) >= 400)
    check('all_wire_timestamps_match_actual_relay', bool(wire) and all(m['usec'] in sources for m in wire))
    check('all_wire_NED_positions_match_ROS_ENU', bool(wire) and all(
        m['usec'] in sources and all(abs(a-b) < 1e-5 for a, b in zip(
            [m['x'], m['y'], m['z']], [sources[m['usec']]['position'][1],
            sources[m['usec']]['position'][0], -sources[m['usec']]['position'][2]])) for m in wire))
    odom_stamps = {p['stamp_ns'] for p in poses['odom']}
    check('relay_keeps_real_LIO_source_stamp', bool(poses['vision']) and
          all(p['stamp_ns'] in odom_stamps for p in poses['vision']))
    check('wire_stamps_increase', bool(wire) and all(a['usec'] < b['usec'] for a, b in zip(wire, wire[1:])))
    check('wire_poses_fresh', bool(wire) and all(-.1 < (m['wall_ns']//1000-m['usec'])/1e6 < .5 for m in wire))
    check('physical_feedback_and_ROS_local_pose', link.counts['fcu_ODOMETRY'] > 300 and len(poses['local']) > 300)
    check('actual_MAVROS_timesync_received', len(sync) > 100)
    check('only_allowlisted_MAVROS_packets', not link.forbidden)
    snapshots = [{**q, 'fields': fields(q)} for q in link.queries]
    report['snapshots'] = snapshots
    def snapshots_for(topic):
        return [q['fields'] for q in snapshots if q['topic'] == topic]
    visual = snapshots_for('vehicle_visual_odometry')
    check('PX4_visual_odometry_sample_received', len(visual) == 2 and all(
        v.get('pose_frame') == 1 and 0 <= v.get('timestamp', 0)-v.get('timestamp_sample', 10**18) < 500000 for v in visual))
    flags = snapshots_for('estimator_status_flags')
    check('PX4_EV_position_height_yaw_active', len(flags) == 2 and all(
        all(v.get(key) is True for key in ('cs_ev_pos', 'cs_ev_hgt', 'cs_ev_yaw')) for v in flags))
    for topic in ('estimator_aid_src_ev_pos', 'estimator_aid_src_ev_hgt', 'estimator_aid_src_ev_yaw'):
        values = snapshots_for(topic)
        check('PX4_'+topic+'_fused', len(values) == 2 and all(v.get('fused') is True
              and v.get('innovation_rejected') is False and v.get('time_last_fuse', 0) > 0 for v in values))
    report.update(vision_packets=len(wire), ros_pose_counts={k: len(v) for k, v in poses.items()},
                  physical_counts=dict(link.counts), states=states, timesync=sync)
except Exception as exc:
    report['error'] = repr(exc)
    print(repr(exc), flush=True)
finally:
    for name, child in reversed(processes):
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)
    for log in logs:
        log.close()
    check('all_owned_processes_exit_normally', bool(processes) and all(p.returncode == 0 for _, p in processes))
    check('all_launched_children_exit_normally', not any('process has died' in (root/(name+'.log')).read_text()
                                                       for name, _ in processes))
    if link is not None:
        try:
            link.close()
            report['serial_closed_and_restored'] = True
        except Exception as exc:
            report['serial_close_error'] = repr(exc)
        (root/'wire.json').write_text(json.dumps({'outgoing': link.outgoing, 'incoming': link.incoming,
            'queries': link.queries, 'forbidden': link.forbidden, 'counts': dict(link.counts),
            'serial_noise': link.serial_noise}, indent=2)+'\n')
    (root/'poses.json').write_text(json.dumps(poses)+'\n')
    report['returncodes'] = {name: p.returncode for name, p in processes}
    report['checks'] = checks
    report['passed'] = bool(checks) and all(checks.values()) and 'error' not in report and report.get('serial_closed_and_restored', False)
    (root/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    node.destroy_node()
    rclpy.try_shutdown()
raise SystemExit(0 if report['passed'] else 1)
