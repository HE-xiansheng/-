#!/usr/bin/env python3
"""只读验证任务状态监视器的退出；不启动任务或调用飞控服务。"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


def alive(pid):
    try:
        return (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()[0] != 'Z'
    except FileNotFoundError:
        return False


def close_case(manager_path, external):
    errors = []
    threading.excepthook = lambda event: errors.append(repr(event.exc_value))
    spec = importlib.util.spec_from_file_location('shutdown_manager', manager_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ros = module.RosState()
    helper_pid = ros.helper.pid
    time.sleep(.5)
    if external:
        ros.rclpy.try_shutdown()
        time.sleep(.25)
    ros.close()
    ros.close()
    result = dict(case='external_shutdown' if external else 'normal_close',
                  checks={'no background executor exception': not errors,
                          'executor thread exited': not ros.thread.is_alive(),
                          'owned identity helper exited': not alive(helper_pid),
                          'context closed': not ros.rclpy.ok()},
                  errors=errors)
    print(json.dumps(result), flush=True)
    return 0 if all(result['checks'].values()) else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manager', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', choices=['normal', 'external'])
    args = parser.parse_args()
    if args.case:
        return close_case(args.manager, args.case == 'external')
    args.output.mkdir(parents=True, exist_ok=True)
    checks = {}
    cases = []
    # 每例独立context；不修改环境中现有节点的context或默认DDS域。
    for name in ['normal', 'external']:
        result = subprocess.run([sys.executable, '-B', __file__, '--manager', str(args.manager),
                                 '--output', str(args.output), '--case', name],
                                capture_output=True, text=True, timeout=15)
        (args.output / (name + '.stdout')).write_text(result.stdout)
        (args.output / (name + '.stderr')).write_text(result.stderr)
        payload = [json.loads(row) for row in result.stdout.splitlines() if row.startswith('{')]
        checks[name + ' process exits zero'] = result.returncode == 0
        checks[name + ' no traceback'] = 'Traceback' not in result.stderr and 'Exception in thread' not in result.stderr
        checks[name + ' reports all lifecycle checks'] = bool(payload and all(payload[-1]['checks'].values()))
        cases.append(dict(case=name, returncode=result.returncode, result=payload))
    with (args.output / 'watch.jsonl').open('w') as output, (args.output / 'watch.stderr').open('w') as err:
        process = subprocess.Popen([sys.executable, '-B', str(args.manager), 'watch'],
                                   stdin=subprocess.DEVNULL, stdout=output, stderr=err)
        helper_pids = []
        try:
            deadline = time.monotonic() + 9
            while time.monotonic() < deadline and process.poll() is None:
                children = Path('/proc') / str(process.pid) / 'task' / str(process.pid) / 'children'
                try:
                    for value in children.read_text().split():
                        if b'lio_identity' in (Path('/proc') / value / 'cmdline').read_bytes():
                            if int(value) not in helper_pids:
                                helper_pids.append(int(value))
                except FileNotFoundError:
                    pass
                time.sleep(.1)
            checks['passive watch stays running until SIGINT'] = process.poll() is None
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            returncode = process.wait(timeout=8)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
    rows = [json.loads(line) for line in (args.output / 'watch.jsonl').read_text().splitlines() if line.startswith('{')]
    stderr = (args.output / 'watch.stderr').read_text()
    checks['watch exits zero after SIGINT'] = returncode == 0
    checks['watch returns at least five valid status records'] = len(rows) >= 5 and all(row.get('ok') for row in rows)
    checks['watch no traceback or shutdown error'] = 'Traceback' not in stderr and 'already called' not in stderr
    checks['watch helper was observed and exited'] = bool(helper_pids) and all(not alive(pid) for pid in helper_pids)
    checks['watch observed connected unarmed controller'] = any(row.get('state', {}).get('fc', {}).get('connected') for row in rows if row.get('state', {}).get('fc')) and all(not row['state']['fc']['armed'] for row in rows if row.get('state', {}).get('fc'))
    result = dict(passed=all(checks.values()), checks=checks, cases=cases,
                  watch=dict(returncode=returncode, records=len(rows), helper_pids=helper_pids),
                  manager=str(args.manager), domain=os.environ.get('ROS_DOMAIN_ID', '0'),
                  scope='passive status subscriptions and own-process shutdown only')
    (args.output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
