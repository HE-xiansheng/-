#!/usr/bin/env python3
"""Actual USB/Gemini/direct gimbal -> installed ROS2 drivers; no flight interfaces."""
import csv
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import yaml
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Empty

root = Path('/tmp/new-camera-ground-check')
platform = Path('/home/orangepi/drone_platform')
assert os.environ['ROS_DOMAIN_ID'] == '189' and os.environ['ROS_LOCALHOST_ONLY'] == '1'
processes = []
logs = []
checks = {}
infos = []
statuses = []
actions = []
report = {'passed': False, 'scope': 'actual independent USB, Gemini335L and physical gimbal direct Ethernet; no camera/gimbal motion commands'}
usb_module = platform/'02_drivers/usb_camera' if os.environ.get('CAMERA_CHECK_INSTALLED') == '1' else root/'candidate-usb'
include_usb = os.environ.get('CAMERA_CHECK_NO_USB') != '1'
if not include_usb:
    report['scope'] = 'actual Gemini335L and physical gimbal only; independent USB absent/untested; no motion commands'
report['software_paths'] = {'usb_module': str(usb_module), **{
    name: get_package_prefix(name) for name in ('usb_cam', 'rtsp_test', 'orbbec_camera')}}
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = rclpy.create_node('new_physical_camera_observer')
sub = node.create_subscription(CameraInfo, '/usb_cam/camera_info', infos.append, 50)
status_sub = node.create_subscription(String, '/usb_cam/capture_status', lambda m: statuses.append(json.loads(m.data)),
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
clients = {action: node.create_client(Empty, '/usb_cam/'+action+'_capture') for action in ('start', 'stop')}


def start(name, command):
    log = (root/(name+'.log')).open('w')
    logs.append(log)
    child = subprocess.Popen(command, cwd=root, stdin=subprocess.DEVNULL, stdout=log,
                             stderr=subprocess.STDOUT, start_new_session=True)
    processes.append((name, child))
    (root/'owned-processes.json').write_text(json.dumps([
        {'name': n, 'pid': p.pid, 'start_ticks': Path('/proc/'+str(p.pid)+'/stat').read_text().split()[21]}
        for n, p in processes], indent=2)+'\n')
    return child


def pump(seconds, predicate=None):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        exited = {name: p.poll() for name, p in processes if p.poll() is not None}
        if exited:
            raise RuntimeError('owned camera process exited: '+repr(exited))
        rclpy.spin_once(node, timeout_sec=.01)
        if predicate and predicate():
            return True
    return bool(predicate and predicate())


def received(key):
    path = root/'headers.csv'
    return path.exists() and any(row['key'] == key for row in csv.DictReader(path.open()))


def check(name, ok):
    checks[name] = bool(ok)
    print(json.dumps({name: bool(ok)}), flush=True)


def capture(action):
    client = clients[action]
    if not pump(8, client.service_is_ready):
        raise RuntimeError('capture service missing: '+action)
    count = len(statuses)
    future = client.call_async(Empty.Request())
    if not pump(5, lambda: future.done() and len(statuses) > count):
        raise RuntimeError('capture response/status timeout: '+action)
    actions.append({'action': action, 'receipt_monotonic_ns': time.monotonic_ns(), 'status': statuses[-1]})
    check('actual_'+action+'_capture_accepted', future.result() is not None and statuses[-1]['success']
          and statuses[-1]['requested_capture'] == (action == 'start'))


def stop(child):
    if child.poll() is None:
        os.killpg(child.pid, signal.SIGINT)
    try:
        child.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid, signal.SIGTERM)
        child.wait(timeout=5)


try:
    source = 'rtsp://192.168.144.108:554'
    probe = subprocess.run(['ffprobe', '-v', 'error', '-rtsp_transport', 'tcp', '-select_streams', 'v:0',
                            '-show_entries', 'stream=codec_name,width,height,avg_frame_rate', '-of', 'json', source],
                           capture_output=True, text=True, timeout=20)
    (root/'source-probe.json').write_text(probe.stdout)
    (root/'source-probe.stderr').write_text(probe.stderr)
    probe.check_returncode()
    video = json.loads(probe.stdout)['streams'][0]
    check('direct_physical_gimbal_h264', video['codec_name'] == 'h264')
    config = yaml.safe_load((platform/'03_tools/gimbal_relay/gimbal.yml').read_text())
    config.update(rtspAddress='127.0.0.1:18554', rtspTransports=['tcp'], rtpAddress='127.0.0.1:18000', rtcpAddress='127.0.0.1:18001')
    (root/'relay.yml').write_text(yaml.safe_dump(config))
    start('relay', ['bash', str(platform/'03_tools/gimbal_relay/start_relay.sh'), str(root/'relay.yml')])
    if not pump(15, lambda: 'stream is available and online' in (root/'relay.log').read_text()):
        raise RuntimeError('direct gimbal relay not online')
    print('PHYSICAL_GIMBAL_RELAY_READY', flush=True)
    reader = start('observer', [str(root/'observer-build/observer'), '--ros-args', '-p', 'include_usb:='+str(include_usb).lower(),
        '-p', 'include_camera:=true', '-p', 'output:='+str(root/'headers.csv'), '-p', 'stop_file:='+str(root/'observer.stop')])
    start('gemini', ['ros2', 'launch', str(platform/'02_drivers/orbbec/launch/gemini335l.launch.py')])
    if not pump(25, lambda: received('stereo') and received('depth')):
        raise RuntimeError('Gemini did not produce real color/depth frames')
    print('PHYSICAL_GEMINI_FRAMES_READY', flush=True)
    if include_usb:
        start('usb', ['ros2', 'launch', str(usb_module/'launch/usb_camera.launch.py')])
        if not pump(12, lambda: received('usb')):
            raise RuntimeError('independent USB camera did not produce real frames')
        print('PHYSICAL_USB_FRAMES_READY', flush=True)
    start('mpp', ['ros2', 'run', 'rtsp_test', 'rtsp_hw_pub_ultra', '--ros-args', '-p',
                  'rtsp_url:=rtsp://127.0.0.1:18554/gimbal', '--log-level', 'rtsp_hw_pub_ultra.frames:=debug'])
    begin = time.monotonic_ns()
    if not pump(15, lambda: received('raw') and received('color')):
        raise RuntimeError('gimbal MPP did not produce real mono/color frames')
    pump(17)
    if include_usb:
        capture('stop')
    pause = time.monotonic_ns()
    pump(3)
    resume = time.monotonic_ns()
    if include_usb:
        capture('start')
    pump(17)
    # End publishers, drain the reader, then close its context via stop marker.
    for name, child in reversed(processes):
        if name != 'observer':
            stop(child)
    deadline = time.monotonic()+1
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.01)
    (root/'observer.stop').touch()
    reader.wait(timeout=10)
    rows = list(csv.DictReader((root/'headers.csv').open()))
    streams = {}
    for key, minimum in [('usb', 22), ('stereo', 25), ('depth', 25), ('raw', 20), ('color', 10)]:
        if key == 'usb' and not include_usb:
            continue
        qs = ('reliable', 'best_effort') if key in ('raw', 'color') else ('reliable',)
        for qos in qs:
            data = [r for r in rows if r['key'] == key and r['qos'] == qos]
            stable = [r for r in data if begin+5*10**9 < int(r['receipt_ns']) < pause]
            hz = (len(stable)-1)*1e9/(int(stable[-1]['receipt_ns'])-int(stable[0]['receipt_ns'])) if len(stable) > 1 else 0
            name = qos+'_'+key
            streams[name] = {'messages': len(data), 'stable_hz': hz,
                'geometry': sorted({(int(r['width']), int(r['height']), r['encoding'], r['frame']) for r in data}),
                'max_age_ms': max((int(r['age_ns'])/1e6 for r in data), default=None)}
            check(name+'_live_rate', hz >= minimum)
            check(name+'_geometry_and_fresh_stamps', bool(data) and all(
                int(r['bytes']) == int(r['step'])*int(r['height']) > 0 and -.1e9 < int(r['age_ns']) < 1e9 for r in data))
            check(name+'_stamps_increase', bool(data) and all(int(b['stamp_ns']) > int(a['stamp_ns']) for a, b in zip(data, data[1:])))
            check(name+'_changing_pixels', len({r['hash'] for r in data}) > 10)
    usb = [r for r in rows if r['key'] == 'usb']
    if include_usb:
        check('USB_exact_configuration', bool(usb) and all((r['width'],r['height'],r['encoding'],r['frame']) ==
              ('1280','720','rgb8','head_camera') for r in usb))
        check('USB_capture_pause_stops_real_images', not any(pause+.7e9 < int(r['receipt_ns']) < resume-.2e9 for r in usb))
        check('USB_capture_resume_restores_real_images', sum(int(r['receipt_ns']) > resume+2e9 for r in usb) > 200)
        check('USB_camera_info_matches_images', len(infos) > 200 and all(m.width == 1280 and m.height == 720 and m.header.frame_id == 'head_camera' for m in infos))
    trace = (root/'mpp.log').read_text()
    emitted = {key: {int(v) for v in re.findall('published_'+key+r' stamp_ns=([0-9]+)', trace)} for key in ('raw', 'color')}
    for key in ('raw', 'color'):
        for qos in ('reliable', 'best_effort'):
            received = {int(r['stamp_ns']) for r in rows if r['key'] == key and r['qos'] == qos}
            check(qos+'_'+key+'_all_published_frames_received', bool(emitted[key]) and received == emitted[key])
    check('gimbal_color_source_stamps_preserved', bool(emitted['color']) and emitted['color'] <= emitted['raw'])
    report.update(streams=streams, capture_actions=actions, source_probe=video,
        USB_calibration={'camera_info_count': len(infos), 'K': list(infos[-1].k) if infos else [],
                         'tested': include_usb, 'calibrated': bool(infos and infos[-1].k[0] > 0)})
except Exception as exc:
    report['error'] = repr(exc)
    print(repr(exc), flush=True)
finally:
    for name, child in reversed(processes):
        if name == 'observer' and child.poll() is None:
            (root/'observer.stop').touch()
        stop(child)
    for log in logs:
        log.close()
    check('all_owned_camera_processes_exit_normally', bool(processes) and all(p.returncode == 0 for _, p in processes))
    check('no_launched_camera_child_crash', not any('process has died' in (root/(name+'.log')).read_text() for name, _ in processes))
    report.update(checks=checks, returncodes={n: p.returncode for n, p in processes},
                  passed=bool(checks) and all(checks.values()) and 'error' not in report)
    (root/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    node.destroy_node()
    rclpy.try_shutdown()
raise SystemExit(0 if report['passed'] else 1)
