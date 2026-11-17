#!/usr/bin/env python3
"""等待基础服务所需的实际设备和固定地址，不打开相机/串口或发送控制。"""
import argparse
import json
import os
from pathlib import Path
import stat
import subprocess
import time


def pending_devices():
    pending = []
    serial = Path('/dev/serial/by-id/usb-Matek_HKUST_UAV_NxtPX4_0-if00')
    try:
        if not stat.S_ISCHR(serial.stat().st_mode) or not os.access(serial, os.R_OK | os.W_OK):
            pending.append('PX4 serial inaccessible')
    except OSError:
        pending.append('PX4 serial missing')
    camera = False
    for device in Path('/sys/bus/usb/devices').iterdir():
        try:
            camera |= ((device / 'idVendor').read_text().strip() == '2bc5' and
                       (device / 'idProduct').read_text().strip() == '0804')
        except OSError:
            continue
    if not camera:
        pending.append('Gemini335L USB missing')
    try:
        interfaces = json.loads(subprocess.check_output(
            ['ip', '-j', 'address'], text=True, timeout=3))
        addresses = {item['ifname']: {address['local'] for address in item.get('addr_info', [])}
                     for item in interfaces}
        for interface, address in [('enP3p49s0', '192.168.1.51'),
                                   ('enx00e09a5af420', '192.168.144.100')]:
            if address not in addresses.get(interface, set()):
                pending.append(interface + ' needs ' + address)
        mac = Path('/sys/class/net/enx00e09a5af420/address').read_text().strip().lower()
        if mac != '00:e0:9a:5a:f4:20':
            pending.append('gimbal Ethernet identity differs')
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        pending.append('network probe: ' + str(error))
    return pending


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=75)
    args = parser.parse_args()
    deadline = time.monotonic() + args.timeout
    previous = None
    last_report = 0
    while True:
        pending = pending_devices()
        now = time.monotonic()
        if pending != previous or now - last_report >= 10:
            print(json.dumps({'sensor_devices_ready': not pending, 'pending': pending}), flush=True)
            previous, last_report = pending, now
        if not pending:
            return 0
        if now >= deadline:
            return 1
        time.sleep(min(1, deadline - now))


if __name__ == '__main__':
    raise SystemExit(main())
