#!/usr/bin/env python3
"""Read selected FAST-LIO timing globals through O_RDONLY /proc/PID/mem.

No ptrace attach, signal, process suspension or memory write is used. Deque
layout observations assume this deployed GCC/libstdc++ AArch64 binary; they are
approximate during concurrent updates and never drive control decisions.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pid', type=int, required=True)
    p.add_argument('--seconds', type=float, default=25.)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.pid != 73150 or not 5 <= a.seconds <= 60 or a.output.parent.parent != Path('/tmp') or not a.output.parent.name.startswith('fastlio-cause-'):
        p.error('fixed inspected PID, duration and experiment output required')
    proc = Path('/proc')/str(a.pid)
    executable = proc.joinpath('exe').resolve()
    assert hashlib.sha256(executable.read_bytes()).hexdigest() == '7707f011a357aa241b11eb098ce86fdbe6e9a73de762c36a960db6937810cbcd'
    mapping = proc.joinpath('maps').read_text()
    base = next(int(line.split()[0].split('-')[0], 16) for line in mapping.splitlines()
                if line.split()[-1] == str(executable) and int(line.split()[2], 16) == 0)
    names = {'last_timestamp_imu', 'last_timestamp_lidar', 'timediff_lidar_wrt_imu', 'lidar_end_time',
             'time_diff_lidar_to_imu', 'timediff_set_flg', 'imu_buffer', 'lidar_buffer', 'time_buffer'}
    symbols = {}
    for line in subprocess.check_output(['nm', '-S', str(executable)], text=True).splitlines():
        fields = line.split()
        if len(fields) == 4 and fields[3] in names:
            symbols[fields[3]] = base + int(fields[0], 16)
    assert set(symbols) == names
    fd = os.open(str(proc/'mem'), os.O_RDONLY)

    def double(name):
        return struct.unpack('<d', os.pread(fd, 8, symbols[name]))[0]

    def deque(name, element_size):
        data = os.pread(fd, 80, symbols[name])
        _, _, start, first, last, start_node, end, end_first, end_last, end_node = struct.unpack('<10Q', data)
        count = (512//element_size)*((end_node-start_node)//8-1) + (end-end_first)//element_size + (last-start)//element_size
        if not 0 <= count <= 100000:
            return dict(valid=False)
        result = dict(valid=True, count=count)
        if count:
            if element_size == 8:
                result['front_time'] = struct.unpack('<d', os.pread(fd, 8, start))[0]
            else:
                pointer = struct.unpack('<Q', os.pread(fd, 8, start))[0]
                if name == 'imu_buffer' and pointer:
                    sec, nsec = struct.unpack('<iI', os.pread(fd, 8, pointer))
                    result['front_time'] = sec+nsec*1e-9
        return result

    rows = []
    begin = time.monotonic()
    try:
        while time.monotonic()-begin < a.seconds:
            now = time.time()
            values = {name: double(name) for name in names if name not in ('timediff_set_flg', 'imu_buffer', 'lidar_buffer', 'time_buffer')}
            if not all(math.isfinite(v) for v in values.values()):
                raise RuntimeError('nonfinite or unsupported memory layout')
            shift = values['timediff_lidar_wrt_imu']
            row = dict(at=time.monotonic()-begin, wall=now, values=values,
                       synchronized=bool(os.pread(fd,1,symbols['timediff_set_flg'])[0]),
                       imu_callback_raw_source_age_s=now-(values['last_timestamp_imu']-shift),
                       lidar_callback_scan_start_age_s=now-values['last_timestamp_lidar'],
                       processing_scan_end_age_s=now-values['lidar_end_time'],
                       imu=deque('imu_buffer',16),lidar=deque('lidar_buffer',16),times=deque('time_buffer',8))
            rows.append(row)
            time.sleep(.05)
    finally:
        os.close(fd)
    result = dict(pid=a.pid, executable=str(executable), base_address=hex(base), symbols=symbols,
                  method='O_RDONLY pread only; no ptrace/signals/writes; concurrent deque snapshots approximate', rows=rows)
    a.output.write_text(json.dumps(result,allow_nan=False)+'\n')
    print(json.dumps(dict(samples=len(rows),file=str(a.output))),flush=True)


if __name__=='__main__':
    main()
