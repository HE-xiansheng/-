#!/usr/bin/env python3
"""Real rqt offscreen subscription and owned viewer cleanup, isolated DDS only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

base = Path(sys.argv[1])
rclpy.init()
node = rclpy.create_node('aruco_viewer_fixture')
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
raw = node.create_publisher(Image, '/rtsp_cam/image_raw', qos)
preview = node.create_publisher(Image, '/aruco_imgresult', qos)
msg = Image()
msg.width = 64
msg.height = 64
msg.encoding = 'mono8'
msg.step = 64
msg.data = bytes([128]) * 4096
msg.header.frame_id = 'fixture_optical'
checks = []
proc = None

def check(ok, label):
    checks.append({'check': label, 'passed': bool(ok)})
    print(('PASS ' if ok else 'FAIL ') + label, flush=True)
    assert ok, label

try:
    log = (base / 'viewer.log').open('wb')
    env = dict(os.environ, DISPLAY=':offscreen', QT_QPA_PLATFORM='offscreen', XDG_CONFIG_HOME=str(base/'qt-config'), QT_XCB_GL_INTEGRATION='none', QT_OPENGL='software', LIBGL_ALWAYS_SOFTWARE='1')
    proc = subprocess.Popen([sys.executable, str(base/'aruco_vision.py'), '--timeout', '6'], env=env, stdout=log, stderr=subprocess.STDOUT)
    deadline = time.monotonic() + 22
    subscribed = False
    while time.monotonic() < deadline and proc.poll() is None:
        stamp = node.get_clock().now().to_msg()
        msg.header.stamp = stamp
        raw.publish(msg)
        preview.publish(msg)
        rclpy.spin_once(node, timeout_sec=.08)
        # The launcher's temporary readiness subscriber has been removed at READY.
        content = (base/'viewer.log').read_text(errors='replace')
        if 'Vision is ready' in content and node.count_subscribers('/aruco_imgresult') > 0:
            subscribed = True
            break
    check(subscribed, 'real rqt subscribes to requested preview topic offscreen')
    check(proc.poll() is None, 'viewer session stays alive')
    check((base/'viewer.log').read_text().count('Started owned process ') == 1, 'only viewer is owned when both streams already exist')
    proc.send_signal(signal.SIGINT)
    proc.wait(timeout=12)
    check(proc.returncode == 0, 'viewer session exits normally on Ctrl+C')
    gone = True
    for line in (base/'viewer.log').read_text(errors='replace').splitlines():
        if line.startswith('Started owned process '):
            try:
                os.killpg(int(line.split()[-1]), 0)
                gone = False
            except ProcessLookupError:
                pass
    check(gone, 'owned viewer process group is reaped')
    check(node.count_publishers('/rtsp_cam/image_raw') == 1 and node.count_publishers('/aruco_imgresult') == 1, 'external image producers survive viewer shutdown')
finally:
    if proc is not None and proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=12)
    (base/'viewer-result.json').write_text(json.dumps({'passed': len(checks) == 6 and all(x['passed'] for x in checks), 'checks': checks}, indent=2))
    node.destroy_node()
    rclpy.shutdown()
