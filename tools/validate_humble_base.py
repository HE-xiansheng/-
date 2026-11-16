#!/usr/bin/env python3
"""Validate installed C++ to Python ROS2 messaging using localhost-only DDS.

Run as orangepi after sourcing Humble. Only starts the official demo nodes.
"""
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    if os.getuid() == 0:
        raise RuntimeError('Run as the application user, not root')
    env = os.environ.copy()
    env.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='174')
    directory = Path(tempfile.mkdtemp(prefix='ros2-base-check-', dir='/tmp')) / datetime.datetime.now().strftime('base-dds-%Y%m%d-%H%M%S')
    directory.mkdir(parents=True, exist_ok=False)
    processes = []
    try:
        for executable, filename in [('demo_nodes_py listener', 'listener.log'),
                                     ('demo_nodes_cpp talker', 'talker.log')]:
            package, node = executable.split()
            with (directory / filename).open('w') as log:
                processes.append(subprocess.Popen(['ros2', 'run', package, node],
                                 env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + 18
        messages = []
        while time.monotonic() < deadline:
            messages = [line for line in (directory / 'listener.log').read_text().splitlines()
                        if 'I heard' in line and 'Hello World' in line]
            if len(messages) >= 5:
                break
            if any(process.poll() is not None for process in processes):
                break
            time.sleep(0.5)
        result = {'checked_at': datetime.datetime.now().astimezone().isoformat(),
                  'passed': len(messages) >= 5, 'received_messages': len(messages),
                  'publisher': 'official demo_nodes_cpp/talker',
                  'subscriber': 'official demo_nodes_py/listener',
                  'ROS_LOCALHOST_ONLY': '1', 'ROS_DOMAIN_ID': '174',
                  'log_directory': str(directory), 'hardware_validation': False}
        (directory / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
        if not result['passed']:
            raise RuntimeError('Fewer than five C++ messages received by Python; inspect logs')
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)


if __name__ == '__main__':
    main()
