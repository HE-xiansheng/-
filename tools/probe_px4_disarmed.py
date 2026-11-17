#!/usr/bin/env python3
"""仅被动读取PX4心跳，确认未解锁；不向串口写入任何字节。"""
import argparse
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import select
import termios
import time
import tty


def probe(device, dialect, seconds):
    spec = importlib.util.spec_from_file_location('passive_px4_common', dialect)
    mav = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mav)
    fd = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    original = None
    exclusive = False
    heartbeats = []
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        original = termios.tcgetattr(fd)
        tty.setraw(fd, termios.TCSANOW)
        attrs = termios.tcgetattr(fd)
        attrs[4] = attrs[5] = termios.B57600
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        parser = mav.MAVLink(None)
        parser.robust_parsing = True
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if not select.select([fd], [], [], .1)[0]:
                continue
            try:
                data = os.read(fd, 65536)
            except BlockingIOError:
                continue
            if not data:
                raise RuntimeError('PX4 serial disconnected')
            for message in parser.parse_buffer(data) or []:
                if (message.get_type() != 'HEARTBEAT' or message.autopilot != 12 or
                        (message.get_srcSystem(), message.get_srcComponent()) != (1, 1)):
                    continue
                row = dict(message.to_dict(), receipt_wall_ns=time.time_ns(),
                           armed=bool(message.base_mode & 128))
                heartbeats.append(row)
                if row['armed']:
                    raise RuntimeError('PX4 is armed; deployment stopped')
            if len(heartbeats) >= 3:
                return {'passed': True, 'bytes_written': 0, 'heartbeats': heartbeats}
        raise TimeoutError('three physical PX4 heartbeats not observed')
    finally:
        try:
            if original is not None:
                termios.tcsetattr(fd, termios.TCSANOW, original)
        finally:
            if exclusive:
                fcntl.ioctl(fd, termios.TIOCNXCL)
            os.close(fd)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', type=Path, default=Path(
        '/dev/serial/by-id/usb-Matek_HKUST_UAV_NxtPX4_0-if00'))
    parser.add_argument('--dialect', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=8)
    args = parser.parse_args()
    print(json.dumps(probe(args.device, args.dialect, args.seconds), indent=2))
