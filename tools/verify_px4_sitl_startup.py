#!/usr/bin/env python3
"""Run official SIH SITL in a loopback-only network namespace, without arming."""
import argparse
import json
import os
import socket
from pathlib import Path
import subprocess
import time

from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--runtime', type=Path, required=True)
    args = parser.parse_args()
    interfaces = {name for _, name in socket.if_nameindex()}
    if interfaces != {'lo'}:
        raise RuntimeError('This test requires a network namespace with only lo')
    subprocess.run(['ip', 'link', 'set', 'lo', 'up'], check=True)
    args.runtime.mkdir(parents=True, exist_ok=False)
    build = args.build.resolve()
    env = dict(os.environ, PX4_SIM_MODEL='sihsim_quadx', PX4_SIMULATOR='sihsim',
               PX4_NO_FOLLOW_MODE='1')
    connection = mavutil.mavlink_connection('udpin:127.0.0.1:14540')
    messages = {}
    heartbeats = []
    result = {'interfaces': sorted(interfaces), 'commands_sent': [], 'checks': {}}
    process = None
    try:
        with (args.runtime / 'px4.log').open('w') as log:
            process = subprocess.Popen([str(build / 'bin/px4'), '-s',
                                        str(build / 'etc/init.d-posix/rcS'),
                                        str(build / 'etc')], cwd=args.runtime,
                                       env=env, stdin=subprocess.PIPE,
                                       stdout=log, stderr=subprocess.STDOUT,
                                       text=True, start_new_session=True)
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError('PX4 exited: %s' % process.returncode)
                message = connection.recv_match(blocking=True, timeout=1)
                if message is None:
                    continue
                kind = message.get_type()
                messages[kind] = message.to_dict()
                if kind == 'HEARTBEAT':
                    heartbeats.append(message.to_dict())
                    if message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        raise RuntimeError('SITL unexpectedly armed')
                if (len(heartbeats) >= 5 and
                        {'LOCAL_POSITION_NED', 'ATTITUDE', 'HIGHRES_IMU'} <= messages.keys()):
                    break
            checks = result['checks']
            checks['heartbeat'] = len(heartbeats) >= 5
            checks['all_disarmed'] = bool(heartbeats) and all(
                not hb['base_mode'] & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                for hb in heartbeats)
            checks['px4_autopilot'] = bool(heartbeats) and all(
                hb['autopilot'] == mavutil.mavlink.MAV_AUTOPILOT_PX4 for hb in heartbeats)
            for kind in ['LOCAL_POSITION_NED', 'ATTITUDE', 'HIGHRES_IMU']:
                checks[kind] = kind in messages
            for command in ['ver all', 'commander status', 'ekf2 status',
                            'listener vehicle_local_position 1', 'shutdown']:
                process.stdin.write(command + '\n')
                process.stdin.flush()
                result['commands_sent'].append(command)
                time.sleep(0.3)
            process.wait(timeout=10)
            checks['clean_exit'] = process.returncode == 0
    finally:
        if process is not None and process.poll() is None:
            import signal
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        connection.close()
        result['heartbeats'] = heartbeats
        result['latest_messages'] = messages
        def json_value(value):
            if isinstance(value, (bytes, bytearray)):
                return list(value)
            raise TypeError('Unsupported result value: %r' % type(value))
        (args.runtime / 'result.json').write_text(
            json.dumps(result, indent=2, default=json_value) + '\n')
    print(json.dumps(result['checks'], indent=2))
    return 0 if all(result['checks'].values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
