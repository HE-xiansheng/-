#!/usr/bin/env python3
"""Manage only this ROS2 adapter. Stop requests preserve OFFBOARD hold until pilot takeover."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parent
RUNTIME = Path(os.environ.get('EGO_EXECUTION_RUNTIME_DIR', '/tmp/ego-px4-execution-%d' % os.getuid()))
ACTIVE = RUNTIME / 'active.json'


def identity(pid):
    try:
        stat = Path('/proc/%d/stat' % pid).read_text().split(') ', 1)[1].split()
        cmd = Path('/proc/%d/cmdline' % pid).read_bytes().replace(b'\0', b' ').decode()
        return (stat[19], cmd) if stat[0] != 'Z' else (None, '')
    except (OSError, IndexError):
        return None, ''


def current():
    try:
        info = json.loads(ACTIVE.read_text())
        tick, cmd = identity(info['pid'])
        if tick == info['start_tick'] and str(ROOT / 'adapter.py') in cmd and '--publish' in cmd:
            return info
    except (OSError, KeyError, ValueError, TypeError):
        pass
    return None


def read_status(info):
    if not info:
        return {'running': False, 'output_enabled': False}
    try:
        status = json.loads((Path(info['run']) / 'status.json').read_text())
    except (OSError, ValueError):
        status = {'phase': 'STARTING', 'output_enabled': False}
    age = time.monotonic() - status.get('monotonic_stamp', 0)
    if status.get('phase') != 'DONE' and not 0 <= age <= .6:
        status.update(phase='STATUS_STALE', output_enabled=False, tracking_output_enabled=False,
                      reason='Execution status is stale; output state cannot be confirmed')
    return dict(status, running=True, pid=info['pid'], run=info['run'], status_age_s=age)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['enable', 'stop', 'status', 'check'])
    parser.add_argument('--isolated-test', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    RUNTIME.mkdir(parents=True, exist_ok=True)
    with (RUNTIME / 'management.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        info = current()
        if args.action == 'status':
            print(json.dumps(read_status(info), ensure_ascii=False))
            return
        if args.action == 'stop':
            if info:
                (Path(info['run']) / 'stop-request').touch()
                time.sleep(.3)
                print(json.dumps(read_status(current()), ensure_ascii=False))
                print('已请求停止跟踪；若仍在OFFBOARD，将保持位置，请遥控切回POSCTL后结束输出。')
            else:
                print(json.dumps({'running': False, 'output_enabled': False}, ensure_ascii=False))
            return
        if info:
            raise RuntimeError('接入会话已运行；使用status查看或stop请求受控停止。')
        run = RUNTIME / ('session-%d' % time.time_ns())
        run.mkdir()
        cmd = ['/usr/bin/python3', '-B', str(ROOT / 'adapter.py'), '--run', str(run),
               '--publish' if args.action == 'enable' else '--check']
        if args.isolated_test or os.environ.get('EGO_EXECUTION_ISOLATED_TEST') == '1':
            cmd.append('--isolated-test')
        if args.action == 'check':
            result = subprocess.run(cmd, timeout=15)
            try:
                print((run / 'status.json').read_text())
            except OSError:
                pass
            raise SystemExit(result.returncode)
        proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL, start_new_session=True)
        tick, _ = identity(proc.pid)
        data = {'pid': proc.pid, 'start_tick': tick, 'run': str(run)}
        temporary = ACTIVE.with_suffix('.next')
        temporary.write_text(json.dumps(data))
        os.replace(temporary, ACTIVE)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and proc.poll() is None:
            if (run / 'status.json').exists():
                print(json.dumps(read_status(data), ensure_ascii=False))
                return
            time.sleep(.05)
        if proc.poll() is not None:
            raise RuntimeError('执行器启动失败，退出码%d；请检查ROS2环境和单实例锁。' % proc.returncode)
        raise RuntimeError('执行器尚未产生状态，请使用status检查；未终止进程。')


if __name__ == '__main__':
    main()
