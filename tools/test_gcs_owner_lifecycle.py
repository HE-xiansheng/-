#!/usr/bin/env python3
"""验证 Linux 窗口生命周期与两类只读遥测桥接；SSH 完全由本机夹具替代。"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time


def identity(pid):
    try:
        fields = Path('/proc/%d/stat' % pid).read_text().rsplit(')', 1)[1].split()
        return fields[19] if fields[0] != 'Z' else None
    except (OSError, IndexError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    repo = Path(__file__).resolve().parents[1]
    results = []
    with tempfile.TemporaryDirectory(prefix='gcs-owner-lifecycle-') as temporary:
        root = Path(temporary)
        fake = root/'ssh'
        fake.write_text('''#!/usr/bin/python3
import json,os,signal,sys,time
from pathlib import Path
pid=os.getpid();fields=Path('/proc/%d/stat'%pid).read_text().rsplit(')',1)[1].split()
with open(os.environ['GCS_FIXTURE_EVENTS'],'a') as log:log.write(json.dumps({'pid':pid,'ticks':fields[19]})+'\\n')
sys.stdin.buffer.read()
print(json.dumps({'topics':{},'server_monotonic':time.monotonic(),'boot_id':'fixture'}),flush=True)
while True:time.sleep(.1)
''')
        fake.chmod(0o755)
        for role in ('preflight', 'pointcloud'):
            for case in ('owner_exit', 'owner_already_gone', 'manual_cli'):
                name = role+'-'+case
                events = args.output/(name+'-children.jsonl')
                events.write_text('')
                env = dict(os.environ, PATH=str(root)+os.pathsep+os.environ['PATH'],
                           GCS_FIXTURE_EVENTS=str(events), GCS_STATE_DIR=str(root/name))
                env.pop('GCS_GUI_OWNER_PID', None)
                owner = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(60)'])
                child = None
                receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                receiver.bind(('127.0.0.1', 0))
                raw = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                raw.bind(('127.0.0.1', 0));raw_port=raw.getsockname()[1];raw.close()
                if case != 'manual_cli':
                    env['GCS_GUI_OWNER_PID'] = str(owner.pid)
                if case == 'owner_already_gone':
                    owner.terminate();owner.wait(timeout=3)
                script = repo/('groundstation/tools/preflight/bridge.py' if role == 'preflight'
                               else 'groundstation/tools/pointcloud-only/manage.py')
                command = [sys.executable, str(script), 'run', '--host', 'fixture.invalid',
                           '--port', str(receiver.getsockname()[1])]
                command += ['--raw-udp-port', str(raw_port)] if role == 'preflight' else ['--pc-ip', '127.0.0.1']
                try:
                    with (args.output/(name+'.log')).open('w') as log:
                        child = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                                 start_new_session=True)
                        if case != 'owner_already_gone':
                            deadline = time.monotonic()+5
                            expected = 2 if role == 'preflight' else 1
                            while time.monotonic()<deadline and len(events.read_text().splitlines())<expected:
                                assert child.poll() is None, name+' exited before its fixture readers'
                                time.sleep(.05)
                            assert len(events.read_text().splitlines()) == expected
                            assert child.poll() is None
                            owner.terminate();owner.wait(timeout=3)
                            if case == 'manual_cli':
                                time.sleep(.6)
                                assert child.poll() is None, 'direct CLI must remain independent of a GUI'
                                child.terminate()
                        code = child.wait(timeout=8)
                    readers = [json.loads(line) for line in events.read_text().splitlines()]
                    assert code == 0, name+' did not exit normally'
                    assert all(identity(r['pid']) != r['ticks'] for r in readers), 'fixture SSH process leaked'
                    if case == 'owner_already_gone':
                        assert not readers, 'closed owner must not open new SSH sessions'
                    result = {'case': name, 'passed': True, 'exit': code, 'fixture_children': readers}
                    results.append(result);print(json.dumps(result), flush=True)
                finally:
                    if owner.poll() is None:owner.terminate();owner.wait(timeout=3)
                    if child and child.poll() is None:child.terminate();child.wait(timeout=8)
                    for row in [json.loads(line) for line in events.read_text().splitlines()]:
                        if identity(row['pid']) == row['ticks']:os.kill(row['pid'], signal.SIGTERM)
                    receiver.close()
    report = {'passed': len(results)==6 and all(r['passed'] for r in results), 'cases': results,
              'scope': 'local fixture SSH only, no ROS or onboard connection'}
    (args.output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
