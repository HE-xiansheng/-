#!/usr/bin/env python3
"""PTY-only checks for the physical PX4 read-only inspector; no serial hardware."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import pty
import select
import struct
import termios
import threading
import time
from inspect_px4_readonly import PARAMETERS, allowed_read, inspect

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--dialect', type=Path, required=True)
a = p.parse_args()
spec = importlib.util.spec_from_file_location('test_mav', a.dialect)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
checks = {}

def check(key, value):
    checks[key] = bool(value)
    if not value:
        raise AssertionError(key)

for message in [m.MAVLink_command_long_message(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0),
                m.MAVLink_set_mode_message(1, 0, 0),
                m.MAVLink_param_set_message(1, 1, b'EKF2_EV_CTRL', 7, 6),
                m.MAVLink_command_long_message(1, 1, 512, 0, 148, 1, 0, 0, 0, 0, 0),
                m.MAVLink_param_request_read_message(1, 1, b'OTHER_SETTING', -1)]:
    check('reject_'+message.get_type()+'_'+str(len(checks)), not allowed_read(message))
master, slave = pty.openpty()
original = termios.tcgetattr(slave)
end = threading.Event()
received = []
errors = []

class Writer:
    def write(self, data):
        os.write(master, data)

peer = m.MAVLink(Writer(), srcSystem=1, srcComponent=1)
parser = m.MAVLink(None)

def emulate():
    try:
        next_heartbeat = time.monotonic() + .2
        while not end.is_set():
            if time.monotonic() >= next_heartbeat:
                peer.heartbeat_send(2, 12, 0, 0, 3)
                next_heartbeat = time.monotonic() + .5
            if not select.select([master], [], [], .02)[0]:
                continue
            for message in parser.parse_buffer(os.read(master, 65536)) or []:
                received.append(message)
                if message.get_type() == 'COMMAND_LONG':
                    peer.autopilot_version_send(16, (1 << 24) | (14 << 16) | 255,
                        0, 0, 0, [0]*8, [0]*8, [0]*8, 1, 2, 3)
                elif message.get_type() == 'PARAM_REQUEST_READ':
                    key = message.param_id
                    value = PARAMETERS.index(key)
                    raw = struct.unpack('<f', struct.pack('<i', value))[0]
                    peer.param_value_send(key.encode('ascii'), raw, 6, len(PARAMETERS), value)
    except Exception as exc:
        errors.append(repr(exc))

thread = threading.Thread(target=emulate)
thread.start()
try:
    report = inspect(Path(os.ttyname(slave)), a.dialect, 5)
    check('PX4_detected_disarmed', report['connected'] and not report['heartbeat']['armed'])
    check('firmware_decoded', report['flight_version_decoded'] == [1, 14, 0, 255])
    check('all_requested_parameters_received', not report['parameters_missing'])
    check('bytewise_integer_decoding', all(report['parameters'][key]['decoded_value'] == i
          for i, key in enumerate(PARAMETERS)))
    check('wire_only_allowed_reads', bool(received) and all(allowed_read(msg) for msg in received))
    check('terminal_settings_restored', termios.tcgetattr(slave) == original)
    reopened = os.open(os.ttyname(slave), os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    os.close(reopened)
    check('exclusive_lock_released', True)
finally:
    end.set()
    thread.join(2)
    os.close(slave)
    os.close(master)
check('emulator_no_errors', not errors)
print(json.dumps({'checks': checks, 'passed': all(checks.values()), 'scope': 'local PTY; no physical FC'}, indent=2))
