#!/usr/bin/env python3
"""Temporary serial/localhost bridge for live, disarmed EV ground validation."""
from collections import Counter
import fcntl
import importlib.util
import math
import os
from pathlib import Path
import select
import socket
import termios
import time
import tty

TOPICS = ('vehicle_visual_odometry', 'estimator_status_flags',
          'estimator_aid_src_ev_pos', 'estimator_aid_src_ev_hgt',
          'estimator_aid_src_ev_yaw', 'estimator_aid_src_ev_vel', 'timesync_status')


def allowed_message(message, now_usec):
    """Only telemetry queries/clocks and fresh finite external poses may pass."""
    kind = message.get_type()
    if kind == 'HEARTBEAT':
        # The onboard-controller heartbeat describes the companion itself.
        # MAVROS uses ACTIVE + MANUAL_INPUT/SAFETY flags; it does not arm PX4.
        return message.autopilot == 8 and ((message.type == 6 and message.base_mode == 0)
               or (message.type == 18 and message.base_mode in (0, 192)))
    if kind in ('TIMESYNC', 'PING'):
        return True
    if kind == 'SYSTEM_TIME':
        return abs(message.time_unix_usec-now_usec) < 2_000_000
    if kind in ('PARAM_REQUEST_LIST', 'PARAM_REQUEST_READ'):
        return message.target_system == 1 and message.target_component in (0, 1)
    if kind in ('MISSION_REQUEST_LIST', 'MISSION_REQUEST', 'MISSION_REQUEST_INT'):
        return message.target_system == 1 and message.target_component in (0, 1)
    if kind == 'MISSION_ACK':
        return message.target_system == 1 and message.target_component in (0, 1) and message.type == 0
    if kind == 'COMMAND_LONG':
        if message.target_system not in (0, 1) or message.target_component not in (0, 1):
            return False
        tail_zero = all(getattr(message, 'param'+str(n)) == 0 for n in range(2, 8))
        if message.command in (519, 520):
            return tail_zero and message.param1 == 1
        if message.command == 410:  # MAV_CMD_GET_HOME_POSITION, read only.
            return tail_zero and message.param1 == 0
        if message.command == 512:
            # REQUEST_MESSAGE p7 selects the reply destination (0/1/2).
            return (0 < message.param1 <= 16777215 and message.param1 == int(message.param1)
                    and all(getattr(message, 'param'+str(n)) == 0 for n in range(2, 7))
                    and message.param7 in (0, 1, 2))
        return False
    if kind == 'VISION_POSITION_ESTIMATE':
        return (-100_000 < now_usec-message.usec < 500_000 and
                all(math.isfinite(getattr(message, key)) for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')))
    return False


def load_dialect(path):
    spec = importlib.util.spec_from_file_location('physical_px4_common', path)
    dialect = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(dialect)
    return dialect


class GroundLink:
    def __init__(self, device, dialect):
        self.mav = load_dialect(dialect)
        self.original = None
        self.sock = None
        self.exclusive = False
        self.fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.physical = self.mav.MAVLink(None)
        self.physical.robust_parsing = True
        self.ros = self.mav.MAVLink(None)
        self.sender = self.mav.MAVLink(None, srcSystem=245, srcComponent=190)
        self.heartbeat = None
        self.heartbeat_time = 0
        self.counts = Counter()
        self.outgoing = []
        self.incoming = []
        self.shell = []
        self.current_query = None
        self.queries = []
        self.forbidden = []
        self.serial_noise = []
        try:
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            self.exclusive = True
            self.original = termios.tcgetattr(self.fd)
            tty.setraw(self.fd, termios.TCSANOW)
            attrs = termios.tcgetattr(self.fd)
            attrs[4] = attrs[5] = termios.B57600
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(('127.0.0.1', 14651))
            self.sock.setblocking(False)
        except BaseException:
            self.close()
            raise

    def write(self, data):
        deadline = time.monotonic()+2
        while data:
            if time.monotonic() > deadline:
                raise TimeoutError('physical serial write')
            if select.select([], [self.fd], [], .05)[1]:
                try:
                    data = data[os.write(self.fd, data):]
                except BlockingIOError:
                    pass

    def send_query(self, topic):
        if topic not in TOPICS or not self.disarmed():
            raise RuntimeError('query not allowed or no fresh disarmed heartbeat')
        command = 'listener '+topic+' 1\n'
        data = command.encode('ascii')
        assert len(data) <= 70
        self.current_query = {'topic': topic, 'wall_ns': time.time_ns(), 'text': ''}
        self.queries.append(self.current_query)
        message = self.mav.MAVLink_serial_control_message(10, 6, 0, 0, len(data), list(data.ljust(70, b'\0')))
        self.write(message.pack(self.sender))
        self.sender.seq = (self.sender.seq+1) % 256

    def disarmed(self):
        return bool(self.heartbeat and not self.heartbeat['base_mode'] & 128
                    and time.monotonic()-self.heartbeat_time < 2.5)

    def step(self):
        readable, _, _ = select.select([self.fd, self.sock], [], [], 0)
        if self.fd in readable:
            data = os.read(self.fd, 65536)
            if not data:
                raise RuntimeError('physical PX4 disconnected')
            for message in self.physical.parse_buffer(data) or []:
                kind = message.get_type()
                self.counts['fcu_'+kind] += 1
                if kind == 'BAD_DATA':
                    if len(self.serial_noise) < 20:
                        self.serial_noise.append({'wall_ns': time.time_ns(), 'hex': bytes(message.data).hex(),
                                                  'reason': message.reason})
                    continue
                if (message.get_srcSystem(), message.get_srcComponent()) != (1, 1):
                    continue
                if kind == 'HEARTBEAT' and message.autopilot == 12:
                    self.heartbeat = message.to_dict()
                    self.heartbeat_time = time.monotonic()
                    if message.base_mode & 128:
                        raise RuntimeError('physical FCU armed; stopping ground test')
                if kind == 'SERIAL_CONTROL':
                    text = bytes(message.data[:message.count]).decode('utf8', errors='replace')
                    self.shell.append({'wall_ns': time.time_ns(), 'text': text})
                    if self.current_query is not None:
                        self.current_query['text'] += text
                if kind in ('HEARTBEAT', 'ODOMETRY', 'LOCAL_POSITION_NED', 'TIMESYNC', 'ESTIMATOR_STATUS', 'AUTOPILOT_VERSION', 'STATUSTEXT'):
                    self.incoming.append({'wall_ns': time.time_ns(), **message.to_dict()})
                self.sock.sendto(message.get_msgbuf(), ('127.0.0.1', 14650))
        if self.sock in readable:
            data, peer = self.sock.recvfrom(65536)
            if peer != ('127.0.0.1', 14650):
                raise RuntimeError('unexpected MAVROS UDP peer')
            for message in self.ros.parse_buffer(data) or []:
                kind = message.get_type()
                if not allowed_message(message, time.time_ns()//1000):
                    self.forbidden.append(message.to_dict())
                    raise RuntimeError('blocked outgoing MAVROS packet '+kind)
                if kind == 'VISION_POSITION_ESTIMATE' and not self.disarmed():
                    raise RuntimeError('refusing estimator input without fresh disarmed physical heartbeat')
                self.write(message.get_msgbuf())
                self.counts['ros_'+kind] += 1
                self.outgoing.append({'wall_ns': time.time_ns(), **message.to_dict()})

    def close(self):
        if self.fd is None:
            return
        try:
            if self.queries:
                # Close only the shell allocated by this test; empty payload.
                message = self.mav.MAVLink_serial_control_message(10, 0, 0, 0, 0, [0]*70)
                self.write(message.pack(self.sender))
        finally:
            try:
                if self.original is not None:
                    termios.tcsetattr(self.fd, termios.TCSANOW, self.original)
            finally:
                if self.exclusive:
                    fcntl.ioctl(self.fd, termios.TIOCNXCL)
                os.close(self.fd)
                self.fd = None
                if self.sock:
                    self.sock.close()
