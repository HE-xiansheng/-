#!/usr/bin/env python3
"""Isolated DDS test of foreground launcher with actual native ArUco."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

base = Path(sys.argv[1])
checks = []
processes = []
logs = []
rclpy.init()
node = rclpy.create_node('vision_launcher_fixture')
raw = None
preview = None
age = 0.0
running = True
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
raw_msg = Image()
raw_msg.width = 1920
raw_msg.height = 1080
raw_msg.encoding = 'mono8'
raw_msg.step = 1920
raw_msg.data = bytes([255]) * (1920 * 1080)
raw_msg.header.frame_id = 'fixture_optical'
preview_msg = Image()
preview_msg.width = 8
preview_msg.height = 8
preview_msg.step = 8
preview_msg.encoding = 'mono8'
preview_msg.data = bytes(64)
preview_msg.header.frame_id = 'fixture_optical'


def pump():
    next_frame = 0
    while running:
        rclpy.spin_once(node, timeout_sec=.02)
        if time.monotonic() >= next_frame:
            stamp = node.get_clock().now().nanoseconds - int(age * 1e9)
            for pub, msg in [(raw, raw_msg), (preview, preview_msg)]:
                if pub is not None:
                    msg.header.stamp.sec = stamp // 1000000000
                    msg.header.stamp.nanosec = stamp % 1000000000
                    pub.publish(msg)
            next_frame = time.monotonic() + .15


thread = threading.Thread(target=pump)
thread.start()


def check(ok, label):
    checks.append({'check': label, 'passed': bool(ok)})
    print(('PASS ' if ok else 'FAIL ') + label, flush=True)
    assert ok, label


def start(label, *args):
    path = base / (label + '.log')
    log = path.open('wb')
    logs.append(log)
    env = dict(os.environ, DISPLAY='')
    proc = subprocess.Popen([sys.executable, str(base/'aruco_vision.py'), '--no-view', '--timeout', '5'] + list(args), stdout=log, stderr=subprocess.STDOUT, env=env)
    processes.append((proc, path))
    return proc, path


def await_text(proc, path, text, timeout=14):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if text in path.read_text(errors='replace'):
            return True
        if proc.poll() is not None:
            return False
        time.sleep(.1)
    return False


def stop(proc):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=12)


def no_owned_groups(path):
    for line in path.read_text(errors='replace').splitlines():
        if not line.startswith('Started owned process '):
            continue
        try:
            os.killpg(int(line.split()[-1]), 0)
            return False
        except ProcessLookupError:
            pass
    return True


try:
    raw = node.create_publisher(Image, '/rtsp_cam/image_raw', qos)
    time.sleep(1)
    p, log = start('native-owned-detector')
    check(await_text(p, log, 'Vision is ready'), 'existing raw image drives actual native detector readiness')
    check('Reusing RTSP publisher' in log.read_text(), 'existing camera is reused')
    check(log.read_text().count('Started owned process ') == 1, 'only detector is owned')
    conflict, conflict_log = start('concurrent-launch')
    check(conflict.wait(timeout=6) == 1 and 'Another foreground' in conflict_log.read_text(), 'second launcher is refused within the same domain')
    check(p.poll() is None, 'refused concurrent launcher leaves first running')
    stop(p)
    check(p.returncode == 0 and no_owned_groups(log), 'Ctrl+C reaps owned detector group')
    check(node.count_publishers('/rtsp_cam/image_raw') == 1, 'Ctrl+C leaves external raw publisher alive')
    preview = node.create_publisher(Image, '/aruco_imgresult', qos)
    time.sleep(2)
    p, log = start('reuse-both')
    check(p.wait(timeout=12) == 0, 'reusing both publishers returns normally')
    check('Reusing ArUco publisher' in log.read_text() and 'Started owned process' not in log.read_text(), 'no duplicate detector or camera when both exist')
    check(node.count_publishers('/aruco_imgresult') == 1, 'existing detector publisher survives launcher exit')
    node.destroy_publisher(preview)
    preview = None
    time.sleep(2)
    age = 8.0
    p, log = start('stale-source')
    check(p.wait(timeout=12) == 1 and 'No fresh image' in log.read_text(), 'stale image stream fails readiness')
    check('Started owned process' not in log.read_text(), 'stale reused stream does not spawn replacement or detector')
    age = -8.0
    p, log = start('future-source')
    check(p.wait(timeout=12) == 1 and 'No fresh image' in log.read_text(), 'future image stream fails readiness')
    check(node.count_publishers('/rtsp_cam/image_raw') == 1, 'failed readiness leaves external source alive')
    node.destroy_publisher(raw)
    raw = None
    age = 0.0
    time.sleep(2)
    p, log = start('failed-owned-camera', 'rtsp_url:=rtsp://127.0.0.1:1/unavailable')
    check(p.wait(timeout=14) == 1, 'owned camera connection failure is reported')
    check('Started owned process' in log.read_text() and no_owned_groups(log), 'failed startup reaps owned camera group')
    print(json.dumps({'passed': True, 'checks': checks}), flush=True)
finally:
    for proc, _ in processes:
        stop(proc)
    running = False
    thread.join(timeout=3)
    for log in logs:
        log.close()
    (base/'result.json').write_text(json.dumps({'passed': all(x['passed'] for x in checks), 'checks': checks}, indent=2))
    node.destroy_node()
    rclpy.shutdown()
