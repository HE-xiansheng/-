#!/usr/bin/env python3
"""Loopback UDP over SSH stdio; PX4 side requires its own network namespace."""
import argparse
import json
import os
from pathlib import Path
import selectors
import signal
import socket
import struct
import subprocess
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['relay', 'px4'])
    ap.add_argument('--build', type=Path)
    ap.add_argument('--runtime', type=Path, required=True)
    ap.add_argument('--seconds', type=float, default=100)
    ap.add_argument('--diagnostics-at', type=float, nargs='*', default=[])
    args = ap.parse_args()
    args.runtime.mkdir(parents=True, exist_ok=False)
    interfaces = [name for _, name in socket.if_nameindex()]
    process = None
    log = None
    packets = []
    counts = {'udp_to_stdio': 0, 'stdio_to_udp': 0}
    if args.mode == 'px4':
        if set(interfaces) != {'lo'}:
            raise RuntimeError('PX4 requires a loopback-only namespace')
        subprocess.run(['ip', 'link', 'set', 'lo', 'up'], check=True)
        from pymavlink.dialects.v20 import common
        parser = common.MAVLink(None)
        log = (args.runtime / 'px4.log').open('w')
        build = args.build.resolve()
        process = subprocess.Popen([str(build/'bin/px4'), '-s',
            str(build/'etc/init.d-posix/rcS'), str(build/'etc')], cwd=args.runtime,
            env=dict(os.environ, PX4_SIM_MODEL='sihsim_quadx', PX4_SIMULATOR='sihsim'),
            stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)
        local_port, peer_port = 14540, 14580
    else:
        local_port, peer_port = 14641, 14640
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', local_port))
    selector = selectors.DefaultSelector()
    selector.register(sock, selectors.EVENT_READ)
    selector.register(sys.stdin.buffer, selectors.EVENT_READ)
    buffer = bytearray()
    started = time.monotonic()
    configured = False
    diagnostics = []
    try:
        while time.monotonic() - started < args.seconds:
            if process and process.poll() is not None:
                raise RuntimeError('PX4 terminated early')
            if process and not configured and time.monotonic() - started > 8:
                # Parameters affect only this disposable simulator, never hardware.
                process.stdin.write('param set EKF2_EV_CTRL 3\n')
                process.stdin.flush()
                configured = True
            if process and len(diagnostics) < len(args.diagnostics_at):
                due = args.diagnostics_at[len(diagnostics)]
                if time.monotonic() - started >= due:
                    for command in ['listener vehicle_visual_odometry 1',
                                    'listener estimator_aid_src_ev_pos 1',
                                    'listener estimator_aid_src_ev_hgt 1']:
                        process.stdin.write(command+'\n')
                    process.stdin.flush()
                    diagnostics.append(time.monotonic()-started)
            for key, _ in selector.select(.1):
                if key.fileobj is sock:
                    data, address = sock.recvfrom(65536)
                    if address != ('127.0.0.1', peer_port):
                        raise RuntimeError('Unexpected UDP sender: %r' % (address,))
                    sys.stdout.buffer.write(struct.pack('!I',len(data)) + data)
                    sys.stdout.buffer.flush()
                    counts['udp_to_stdio'] += 1
                else:
                    data = os.read(sys.stdin.fileno(), 65536)
                    if not data:
                        return
                    buffer.extend(data)
                    while len(buffer) >= 4:
                        size = struct.unpack('!I',buffer[:4])[0]
                        if not 0 < size <= 65535:
                            raise RuntimeError('Invalid framed packet length')
                        if len(buffer) < size + 4:
                            break
                        packet = bytes(buffer[4:4+size]);del buffer[:4+size]
                        if process:
                            for message in parser.parse_buffer(packet) or []:
                                kind=message.get_type()
                                if kind in ['SET_MODE', 'COMMAND_INT'] or (
                                    kind=='COMMAND_LONG' and message.command in [400,22]):
                                    raise RuntimeError('Forbidden flight command: '+kind)
                                if kind in ['VISION_POSITION_ESTIMATE',
                                            'SET_POSITION_TARGET_LOCAL_NED']:
                                    packets.append(message.to_dict())
                        sock.sendto(packet, ('127.0.0.1',peer_port))
                        counts['stdio_to_udp'] += 1
    finally:
        if process:
            try:
                for command in ['listener vehicle_visual_odometry 1',
                                'listener estimator_aid_src_ev_pos 1',
                                'listener estimator_aid_src_ev_hgt 1',
                                'commander status', 'shutdown']:
                    process.stdin.write(command+'\n');process.stdin.flush();time.sleep(.4)
                process.wait(timeout=10)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
                log.close()
        sock.close();selector.close()
        (args.runtime/'transport.json').write_text(json.dumps({
            'interfaces':interfaces, 'counts':counts, 'packets':packets,
            'diagnostic_offsets_seconds':diagnostics,
            'simulator_parameter_commands':['param set EKF2_EV_CTRL 3'] if configured else [],
            'process_returncode':process.returncode if process else None},indent=2)+'\n')


if __name__ == '__main__':
    main()
