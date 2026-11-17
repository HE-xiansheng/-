#!/usr/bin/env python3
"""Consume live read-only ROS1 input into isolated installed ROS2 LIO/relay."""
import base64
from collections import Counter, defaultdict
import csv
import faulthandler
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import struct
import subprocess
import sys
import threading
import time

from livox_live_protocol import read_frame, TOPICS
from validate_fastlio_replay import decode
import rclpy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import PoseStamped
from livox_ros_driver2.msg import CustomMsg

out = Path(sys.argv[1])
faulthandler.enable()
assert out == Path('/tmp/livox-live-check')
assert os.environ['ROS_DOMAIN_ID'] == '183' and os.environ['ROS_LOCALHOST_ONLY'] == '1'
root = Path('/home/orangepi/drone_platform')
pending = queue.Queue(maxsize=1024)
done = threading.Event()
abort = threading.Event()
errors = []
source = None
checks = {}
inputs = Counter()
input_rows = []
outputs = defaultdict(list)
processes = []
logs = []
result = {'scope': 'live .235 sensor subscription over SSH to installed .224 LIO/relay; domain183 localhost; no physical FC input, no native .224 LiDAR driver', 'passed': False}
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = rclpy.create_node('livox_live_validation')
pubs = {1: node.create_publisher(Imu, TOPICS[1], 200),
        2: node.create_publisher(CustomMsg, TOPICS[2], 20)}


def stamp(m):
    return m.header.stamp.sec*10**9 + m.header.stamp.nanosec


def receive(key, m):
    now = time.monotonic_ns()
    row = [now, stamp(m), (time.time_ns()-stamp(m))/1e9, m.header.frame_id]
    if key == 'odom':
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        row += [p.x, p.y, p.z, q.x, q.y, q.z, q.w, m.child_frame_id]
    elif key == 'cloud':
        row += [m.width*m.height]
    outputs[key].append(row)


subs = [node.create_subscription(typ, topic, lambda m, k=key: receive(k, m), 200)
        for typ, topic, key in [(Odometry, '/factory_fastlio/odometry', 'odom'),
            (PointCloud2, '/factory_fastlio/cloud_body', 'cloud'),
            (PoseStamped, '/mavros/vision_pose/pose', 'vision')]]


def check(name, ok):
    checks[name] = bool(ok)
    print(json.dumps({name: bool(ok)}), flush=True)


def start(name, args):
    f = (out/(name+'.log')).open('w')
    logs.append(f)
    p = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=f, stderr=subprocess.STDOUT, start_new_session=True, cwd=out)
    processes.append((name, p))
    (out/'owned-processes.json').write_text(json.dumps([
        {'name': n, 'pid': child.pid, 'start_ticks': Path('/proc/'+str(child.pid)+'/stat').read_text().split()[21]}
        for n, child in processes], indent=2)+'\n')


def alive():
    for name, p in processes:
        if p.poll() is not None:
            raise RuntimeError(name+' exited '+str(p.returncode))


def reader():
    try:
        while not abort.is_set():
            frame = read_frame(sys.stdin.buffer)
            if frame is None:
                raise ValueError('source EOF without terminal metadata')
            while not abort.is_set():
                try:
                    pending.put(frame, timeout=.2)
                    break
                except queue.Full:
                    continue
            if frame[0] == 255:
                if sys.stdin.buffer.read(1):
                    raise ValueError('data after terminal metadata')
                break
    except Exception as exc:
        errors.append(repr(exc))
    finally:
        done.set()


first = last = None
thread = None
try:
    start('lio', ['ros2', 'launch', str(root/'01_apps/fast_lio/launch/localization.launch.py')])
    start('relay', ['ros2', 'run', 'to_mavros', 'vision2mavros'])
    deadline = time.monotonic()+30
    ready = False
    while time.monotonic() < deadline:
        alive()
        rclpy.spin_once(node, timeout_sec=.05)
        ready = all(any(ep.node_name == 'factory_laserMapping'
                    for ep in node.get_subscriptions_info_by_topic(topic)) for topic in TOPICS.values())
        if ready:
            break
    check('actual_lio_consumers_ready', ready)
    if not ready:
        raise RuntimeError('actual LIO subscriptions missing')
    snapshot = subprocess.run(['ros2', 'param', 'dump', '/factory_laserMapping'],
                            stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=15)
    (out/'lio-parameters.yaml').write_text(snapshot.stdout+snapshot.stderr)
    if snapshot.returncode:
        raise RuntimeError('LIO parameter snapshot failed')
    print('LIVE_INPUT_READY', flush=True)
    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    deadline = time.monotonic()+150
    while time.monotonic() < deadline and not (done.is_set() and pending.empty()):
        alive()
        if errors:
            raise RuntimeError(errors)
        try:
            kind, raw, source_receipt = pending.get(timeout=.01)
        except queue.Empty:
            rclpy.spin_once(node, timeout_sec=.001)
            continue
        if kind == 255:
            source = json.loads(raw)
            continue
        begin = time.monotonic_ns()
        topic, _, message = decode({'topic': TOPICS[kind], 'receipt_offset': 0,
                                   'serialized': base64.b64encode(raw).decode('ascii')})
        _, sec, ns, _ = struct.unpack_from('<IIII', raw)
        if stamp(message) != sec*10**9+ns:
            raise ValueError('source header stamp changed')
        pubs[kind].publish(message)
        now = time.monotonic_ns()
        first = first or now
        last = now
        input_rows.append([kind, now, stamp(message), source_receipt, time.time_ns(),
                           (time.time_ns()-stamp(message))/1e9, (now-begin)/1e6,
                           pending.qsize(), len(raw), hashlib.sha256(raw).hexdigest()])
        inputs[kind] += 1
        rclpy.spin_once(node, timeout_sec=0)
    if not done.is_set() or not pending.empty() or errors:
        raise RuntimeError('input did not finish cleanly: '+repr(errors))
    deadline = time.monotonic()+3
    while time.monotonic() < deadline:
        alive()
        rclpy.spin_once(node, timeout_sec=.01)
    check('source_terminal_metadata_received', source is not None)
    check('source_observer_creates_no_publishers', bool(source) and source.get('publishers_created') == 0)
    check('source_bridge_queue_no_drops', bool(source) and not any(source['dropped'].values()))
    check('source_observed_expected_message_types', bool(source) and not source.get('errors')
          and source.get('types') == {'1': 'sensor_msgs/Imu', '2': 'livox_ros_driver2/CustomMsg'})
    check('all_observed_source_inputs_published', bool(source) and all(
        inputs[k] == source['sent'][str(k)] == source['received'][str(k)] for k in TOPICS))
    check('both_sensor_streams_present', inputs[1] > 200 and inputs[2] > 30)
    check('source_frames_crc_and_schema_valid', not errors)
    stream_results = {}
    for kind in TOPICS:
        rows = [r for r in input_rows if r[0] == kind]
        check(TOPICS[kind]+'_source_stamp_increases', bool(rows) and all(b[2] > a[2] for a,b in zip(rows, rows[1:])))
        check(TOPICS[kind]+'_source_age_bounded', bool(rows) and all(-.1 < r[5] < 2 for r in rows))
    stable_begin = first+15*10**9 if first else 0
    for key, min_hz in [('odom', 7), ('cloud', 7), ('vision', 7)]:
        rows = [r for r in outputs[key] if stable_begin <= r[0] <= (last or 0)]
        hz = (len(rows)-1)*1e9/(rows[-1][0]-rows[0][0]) if len(rows)>1 else 0
        gap = max(((b[0]-a[0])/1e9 for a,b in zip(rows, rows[1:])), default=999)
        stream_results[key] = {'messages': len(rows), 'hz': hz, 'max_gap_s': gap,
            'min_age_s': min((r[2] for r in rows), default=None), 'max_age_s': max((r[2] for r in rows), default=None)}
        check(key+'_sustained_rate', hz >= min_hz)
        check(key+'_no_long_gap', gap < 2)
        check(key+'_source_age_bounded', bool(rows) and all(-.1 < r[2] < 2 for r in rows))
        check(key+'_source_stamp_increases', bool(rows) and all(b[1]>a[1] for a,b in zip(rows,rows[1:])))
    check('lio_valid_frames_and_pose', bool(outputs['odom']) and all(r[3]=='camera_init' and r[11]=='body'
        and all(math.isfinite(v) for v in r[4:11]) and abs(sum(v*v for v in r[7:11])-1)<.01 for r in outputs['odom']))
    check('body_cloud_frames', bool(outputs['cloud']) and all(r[3]=='body' and r[4]>0 for r in outputs['cloud']))
    odom_stamps = {r[1] for r in outputs['odom']}
    check('relay_preserves_actual_lio_stamps', bool(outputs['vision']) and {r[1] for r in outputs['vision']} <= odom_stamps)
    check('no_control_or_fcu_interfaces', not any(node.get_publishers_info_by_topic(t) for t in
        ['/mavros/setpoint_raw/local','/ego_planner/flight_execution','/Safina_track_stamped'])
        and not any(n in ['/mavros/set_mode','/mavros/cmd/arming'] for n,_ in node.get_service_names_and_types()))
    result.update(source=source, stream_results=stream_results, passed=all(checks.values()))
except Exception as exc:
    result['error'] = repr(exc)
    print(repr(exc), file=sys.stderr, flush=True)
finally:
    abort.set()
    for name, p in reversed(processes):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=5)
    for f in logs:
        f.close()
    result.update(checks=checks, input_counts=dict(inputs), reader_errors=errors,
                  returncodes={n:p.returncode for n,p in processes})
    with (out/'input-headers.csv').open('w') as f:
        writer = csv.writer(f)
        writer.writerow(['kind','monotonic_ns','source_stamp_ns','source_receipt_ns','publish_wall_ns',
                         'source_age_s','decode_publish_ms','queue_depth','raw_bytes','raw_sha256'])
        writer.writerows(input_rows)
    for key, rows in outputs.items():
        with (out/(key+'-headers.jsonl')).open('w') as f:
            for row in rows:
                f.write(json.dumps(row)+'\n')
    (out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    node.destroy_node()
    rclpy.try_shutdown()
raise SystemExit(0 if result['passed'] else 1)
