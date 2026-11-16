#!/usr/bin/env python3
"""Read physical PX4 firmware/EV parameters over an exclusively opened serial port.

No ROS nodes, persistent output, parameter writes, mode changes or setpoints.
The standalone generated pymavlink common dialect is supplied via --dialect.
"""
import argparse
from collections import Counter
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import select
import struct
import termios
import time
import tty

PARAMETERS = [
    'EKF2_EV_CTRL', 'EKF2_AID_MASK', 'EKF2_EV_DELAY', 'EKF2_HGT_REF',
    'EKF2_HGT_MODE', 'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z',
    'EKF2_EV_NOISE_MD', 'EKF2_EVP_NOISE', 'EKF2_EVV_NOISE', 'EKF2_EVA_NOISE',
    'EKF2_EV_QMIN', 'EKF2_EV_P_NOISE', 'EKF2_EV_V_NOISE', 'EKF2_EV_Y_NOISE',
]


def allowed_read(message):
    kind = message.get_type()
    if kind == 'HEARTBEAT':
        return message.type == 6 and message.autopilot == 8 and message.base_mode == 0
    if kind == 'PARAM_REQUEST_READ':
        key = message.param_id
        if isinstance(key, bytes):
            key = key.decode('ascii').rstrip('\0')
        return key in PARAMETERS and message.param_index == -1
    if kind == 'COMMAND_LONG':
        return (message.command == 512 and message.param1 == 148
                and all(getattr(message, 'param'+str(i)) == 0 for i in range(2, 8)))
    return False


def inspect(device, dialect, seconds):
    spec = importlib.util.spec_from_file_location('px4_readonly_common', dialect)
    mav = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mav)
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original = None
    exclusive = False
    report = {'scope': 'physical PX4 firmware/EV settings read only; no estimator input or flight commands',
              'device': str(device), 'duration_s': seconds, 'parameters_requested': PARAMETERS,
              'parameters': {}, 'heartbeat': None, 'firmware': None, 'outgoing': [],
              'first_message_wall_ns': None}
    counts = Counter()
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        original = termios.tcgetattr(fd)
        tty.setraw(fd, termios.TCSANOW)
        attrs = termios.tcgetattr(fd)
        attrs[4] = attrs[5] = termios.B57600
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        parser = mav.MAVLink(None)
        sender = mav.MAVLink(None, srcSystem=245, srcComponent=190)

        def send(message):
            if not allowed_read(message):
                raise RuntimeError('Forbidden outgoing MAVLink: '+message.get_type())
            data = message.pack(sender)
            sender.seq = (sender.seq + 1) % 256
            while data:
                _, writable, _ = select.select([], [fd], [], 2)
                if not writable:
                    raise TimeoutError('serial write readiness')
                try:
                    data = data[os.write(fd, data):]
                except BlockingIOError:
                    continue
            report['outgoing'].append(message.to_dict())

        begin = time.monotonic()
        deadline = begin + seconds
        next_heartbeat = next_request = begin
        target = None
        request_index = 0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                send(mav.MAVLink_heartbeat_message(6, 8, 0, 0, 0, 3))
                next_heartbeat = now + 1
            if target and now >= next_request:
                if request_index % (len(PARAMETERS)+1) == 0:
                    send(mav.MAVLink_command_long_message(*target, 512, 0, 148, 0, 0, 0, 0, 0, 0))
                else:
                    key = PARAMETERS[(request_index-1) % (len(PARAMETERS)+1)]
                    if key not in report['parameters']:
                        send(mav.MAVLink_param_request_read_message(*target, key.encode('ascii'), -1))
                request_index += 1
                next_request = now + .2
            readable, _, _ = select.select([fd], [], [], .05)
            if not readable:
                continue
            try:
                data = os.read(fd, 65536)
            except BlockingIOError:
                continue
            if not data:
                raise RuntimeError('serial device disconnected')
            for message in parser.parse_buffer(data) or []:
                kind = message.get_type()
                counts[kind] += 1
                if report['first_message_wall_ns'] is None:
                    report['first_message_wall_ns'] = time.time_ns()
                if kind == 'HEARTBEAT' and message.autopilot == 12:
                    target = (message.get_srcSystem(), message.get_srcComponent())
                    report['heartbeat'] = {**message.to_dict(), 'system_id': target[0],
                                           'component_id': target[1], 'armed': bool(message.base_mode & 128)}
                if target != (message.get_srcSystem(), message.get_srcComponent()):
                    continue
                if kind == 'AUTOPILOT_VERSION':
                    report['firmware'] = message.to_dict()
                    value = message.flight_sw_version
                    report['flight_version_decoded'] = [value >> 24, (value >> 16) & 255,
                                                        (value >> 8) & 255, value & 255]
                    report['capabilities'] = value = message.capabilities
                    report['parameter_encoding'] = 'bytewise' if value & 16 else ('c_cast' if value & 131072 else 'unspecified')
                if kind == 'PARAM_VALUE' and message.param_id in PARAMETERS:
                    # Preserve raw float and MAV_PARAM_TYPE; integer bytewise values
                    # require reinterpretation rather than a numerical float cast.
                    report['parameters'][message.param_id] = message.to_dict()
        report['messages'] = dict(counts)
        formats = {1: 'B', 2: 'b', 3: 'H', 4: 'h', 5: 'I', 6: 'i', 9: 'f'}
        for row in report['parameters'].values():
            value = row.pop('param_value')
            raw = struct.pack('<f', value)
            row['param_value_wire_hex'] = raw.hex()
            row['param_value_float'] = value if value == value and abs(value) != float('inf') else None
            fmt = formats.get(row['param_type'])
            if fmt and report.get('parameter_encoding') == 'bytewise':
                decoded = struct.unpack('<'+fmt, raw[:struct.calcsize(fmt)])[0]
                row['decoded_value'] = decoded
            elif report.get('parameter_encoding') == 'c_cast':
                row['decoded_value'] = int(value) if row['param_type'] != 9 else value
        report['parameters_missing'] = [key for key in PARAMETERS if key not in report['parameters']]
        report['connected'] = report['heartbeat'] is not None
        return report
    finally:
        try:
            if original is not None:
                termios.tcsetattr(fd, termios.TCSANOW, original)
        finally:
            if exclusive:
                try:
                    fcntl.ioctl(fd, termios.TIOCNXCL)
                except OSError:
                    pass
            os.close(fd)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device', type=Path, required=True)
    p.add_argument('--dialect', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=20)
    a = p.parse_args()
    if not 5 <= a.seconds <= 60:
        p.error('seconds must be in [5,60]')
    print(json.dumps(inspect(a.device, a.dialect, a.seconds), indent=2, allow_nan=False))
