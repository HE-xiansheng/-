#!/usr/bin/env python3
"""Temporary native signal backtraces, comparing direct and launch shutdown."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu

root = Path('/tmp/livox-shutdown-debug')
sdk_dir = Path('/tmp/livox-sdk-symbol-check/build/sdk_core') if '--candidate-sdk' in sys.argv else None
if '--control-sdk' in sys.argv:
    assert sdk_dir is None
    sdk_dir = Path('/tmp/livox-sdk-symbol-check/control')
if sdk_dir:
    assert (sdk_dir/'liblivox_lidar_sdk_shared.so').is_file()
assert os.environ['ROS_DOMAIN_ID'] == '185' and os.environ['ROS_LOCALHOST_ONLY'] == '1'
source = r'''
#define _GNU_SOURCE
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>
static void trace(int sig) {
  const char text[] = "NATIVE_SIGSEGV_BACKTRACE\n";
  write(STDERR_FILENO, text, sizeof(text)-1);
  void *frames[64];
  int n = backtrace(frames, 64);
  backtrace_symbols_fd(frames, n, STDERR_FILENO);
  signal(sig, SIG_DFL);
  raise(sig);
}
__attribute__((constructor)) static void setup(void) {
  void *frame[1]; backtrace(frame, 1);
  struct sigaction action = {0};
  action.sa_handler = trace;
  sigemptyset(&action.sa_mask);
  sigaction(SIGSEGV, &action, 0);
}
'''
(root/'sigtrace.c').write_text(source)
subprocess.run(['gcc', '-shared', '-fPIC', '-g', '-o', str(root/'sigtrace.so'), str(root/'sigtrace.c')], check=True)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = rclpy.create_node('livox_shutdown_observer')
counts = {'lidar': 0, 'imu': 0}
def receive(kind, data):
    counts[kind] += 1
subs = [node.create_subscription(typ, topic, lambda m, k=kind: receive(k, m), 100, raw=True)
    for kind, typ, topic in [('lidar', CustomMsg, '/livox/lidar'), ('imu', Imu, '/livox/imu')]]
platform = Path('/home/orangepi/drone_platform')
binary = platform/'02_drivers/sensors/workspace/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node'
direct = [str(binary), '--ros-args', '-r', '__node:=livox_lidar_publisher2']
for key, value in {'xfer_format': '1', 'multi_topic': '0', 'data_src': '0', 'output_data_type': '0',
                   'publish_freq': '10.0', 'frame_id': 'livox_frame',
                   'user_config_path': str(platform/'02_drivers/sensors/config/MID360s_config.json')}.items():
    direct += ['-p', key+':='+value]
launch = ['ros2', 'launch', str(platform/'02_drivers/sensors/launch/livox_mid360s.launch.py')]
cases = []
owned = []
try:
    commands = [('direct-single-signal', direct), ('launch-group-signal-1', launch), ('launch-group-signal-2', launch)]
    if '--direct-only' in sys.argv:
        commands = commands[:1]
    for name, command in commands:
        counts = dict.fromkeys(counts, 0)
        with (root/(name+'.log')).open('w') as log:
            env = dict(os.environ, LD_PRELOAD=str(root/'sigtrace.so'))
            if sdk_dir:
                env['LD_LIBRARY_PATH'] = str(sdk_dir)+':'+env.get('LD_LIBRARY_PATH', '')
            child = subprocess.Popen(command, env=env, cwd=root, stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            owned.append({'name': name, 'pid': child.pid,
                'start_ticks': Path('/proc/'+str(child.pid)+'/stat').read_text().split()[21]})
            (root/'owned-processes.json').write_text(json.dumps(owned, indent=2)+'\n')
            try:
                until = time.monotonic()+10
                while time.monotonic() < until and child.poll() is None:
                    rclpy.spin_once(node, timeout_sec=.01)
            finally:
                loaded = set()
                for proc in Path('/proc').iterdir():
                    if not proc.name.isdigit():
                        continue
                    try:
                        if os.getpgid(int(proc.name)) == child.pid:
                            loaded.update(line.split()[-1] for line in (proc/'maps').read_text().splitlines()
                                          if 'liblivox_lidar_sdk_shared.so' in line)
                    except (FileNotFoundError, ProcessLookupError, PermissionError):
                        pass
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGINT)
                    try:
                        child.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGTERM)
                        child.wait(timeout=5)
        text = (root/(name+'.log')).read_text()
        result = {'case': name, 'returncode': child.returncode, 'messages': counts,
                  'loaded_sdk': sorted(loaded),
                  'segfault': child.returncode == -11 or 'exit code -11' in text,
                  'backtrace': 'NATIVE_SIGSEGV_BACKTRACE' in text}
        cases.append(result)
        print(json.dumps(result), flush=True)
        until = time.monotonic()+.5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.01)
finally:
    (root/'result.json').write_text(json.dumps({'scope': 'unchanged installed driver; temporary preload only; no FCU',
        'candidate_sdk': str(sdk_dir) if sdk_dir else None, 'cases': cases}, indent=2)+'\n')
    node.destroy_node()
    rclpy.try_shutdown()
