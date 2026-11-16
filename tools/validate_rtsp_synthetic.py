#!/usr/bin/env python3
"""Validate RK3588 MPP decoding through a localhost RTSP test fixture."""
import json
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from sensor_msgs.msg import Image


def main():
    os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='177')
    processes = []
    logs = []
    with tempfile.TemporaryDirectory(prefix='rtsp-validation-') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        config = os.path.join(directory, 'fixture.yml')
        with open(config, 'w') as stream:
            stream.write('''logDestinations: [stdout]
rtspAddress: 127.0.0.1:18554
rtspTransports: [tcp]
rtmp: false
hls: false
webrtc: false
srt: false
moq: false
api: false
paths:
  fixture:
    source: publisher
''')
        def start(command):
            log = tempfile.TemporaryFile(mode='w+')
            logs.append(log)
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(process)
            return process
        rclpy.init()
        node = rclpy.create_node('rtsp_synthetic_validation')
        records = {}
        subscriptions = []
        def callback(topic, message):
            entry = records.setdefault(topic, {'count': 0, 'first': time.monotonic(), 'last': None,
                                                'stamp_regressions': 0})
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            if entry['last'] is not None and stamp < entry['last']:
                entry['stamp_regressions'] += 1
            entry.update(count=entry['count'] + 1, last=stamp,
                         width=message.width, height=message.height, encoding=message.encoding,
                         frame_id=message.header.frame_id, bytes=len(message.data), step=message.step,
                         age_seconds=(node.get_clock().now().nanoseconds - stamp) / 1e9,
                         last_receipt=time.monotonic())
        for topic in ['/migration_validation/gray', '/migration_validation/color']:
            subscriptions.append(node.create_subscription(Image, topic, lambda m, t=topic: callback(t, m), 1))
        try:
            server = start(['/home/orangepi/drone_platform/03_tools/gimbal_relay/mediamtx', config])
            time.sleep(1)
            encoder = start(['ffmpeg', '-hide_banner', '-loglevel', 'warning', '-re', '-f', 'lavfi',
                             '-i', 'testsrc2=size=1920x1080:rate=15', '-an', '-c:v', 'libx264',
                             '-preset', 'ultrafast', '-tune', 'zerolatency', '-threads', '2',
                             '-pix_fmt', 'yuv420p', '-g', '15', '-f', 'rtsp', '-rtsp_transport', 'tcp',
                             'rtsp://127.0.0.1:18554/fixture'])
            time.sleep(3)
            decoder_command = [os.environ['RTSP_DECODER_EXECUTABLE']] if os.environ.get('RTSP_DECODER_EXECUTABLE') else ['ros2', 'run', 'rtsp_test', 'rtsp_hw_pub_ultra']
            decoder = start(decoder_command + ['--ros-args',
                             '-p', 'rtsp_url:=rtsp://127.0.0.1:18554/fixture',
                             '-r', '/rtsp_cam/image_raw:=/migration_validation/gray',
                             '-r', '/rtsp_cam/image_color:=/migration_validation/color'])
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError('A fixture or decoder process exited unexpectedly')
            for entry in records.values():
                duration = entry.pop('last_receipt') - entry.pop('first')
                entry['observed_hz'] = (entry['count'] - 1) / duration if duration else 0
            checks = {
                'gray_full_resolution': records.get('/migration_validation/gray', {}).get('width') == 1920
                    and records.get('/migration_validation/gray', {}).get('height') == 1080,
                'gray_mono8': records.get('/migration_validation/gray', {}).get('encoding') == 'mono8',
                'color_preview': records.get('/migration_validation/color', {}).get('width') == 960
                    and records.get('/migration_validation/color', {}).get('encoding') == 'bgr8',
                'streams_received': len(records) == 2 and all(r['count'] >= 30 for r in records.values()),
                'current_monotonic_stamps': len(records) == 2 and all(r['stamp_regressions'] == 0
                    and 0 <= r['age_seconds'] < 2 for r in records.values()),
            }
            result = {'passed': all(checks.values()), 'checks': checks, 'streams': records,
                      'flight_commands_sent': False,
                      'limitations': ['Local synthetic H264 only; physical gimbal network/reconnect not validated',
                                      'Image timestamps describe decoder receipt, not camera exposure time']}
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            for log in logs:
                log.seek(0)
                print(log.read(), file=__import__('sys').stderr)
                log.close()
            node.destroy_node()
            rclpy.shutdown()
        result['shutdown_return_codes'] = [p.returncode for p in processes]
        result['checks']['clean_shutdown'] = processes[0].returncode == 0 and processes[1].returncode in (0, 255) and processes[2].returncode == 0
        result['passed'] = all(result['checks'].values())
        print(json.dumps(result, indent=2))
        return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
