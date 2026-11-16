#!/usr/bin/env python3
"""验证显示遥测的断线重连与进程归属；不连接机载设备或飞控。"""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


manager = load('gcs_cloud_manager_test', HERE / 'manage.py')
sender = load('gcs_cloud_sender_test', HERE / 'cloud_sender.py')


class ReconnectTests(unittest.TestCase):
    def test_managed_and_legacy_process_identity(self):
        with tempfile.TemporaryDirectory() as temp:
            state = Path(temp)
            (state / 'active.json').write_text(json.dumps({'pid': 42, 'start_tick': '123'}))
            variants = [
                [b'/usr/bin/python3', os.fsencode(str(HERE / 'manage.py')), b'run'],
                [b'/usr/bin/ssh', b'command # dianyun-pointcloud-only'],
            ]
            with patch.object(manager, 'STATE', state):
                for args in variants:
                    with patch.object(manager, 'identity', return_value=('123', args)):
                        self.assertEqual(manager.current()['pid'], 42)
                with patch.object(manager, 'identity', return_value=('999', variants[0])):
                    self.assertIsNone(manager.current())

    def test_duplicate_destination_lock_is_released(self):
        ip, port = '127.255.254.1', 50000 + os.getpid() % 10000
        lock_path = Path('/tmp/dianyun-gcs-cloud-{}-{}.lock'.format(ip, port))
        self.assertFalse(lock_path.exists(), '测试锁文件已存在，避免影响其他进程')
        first = sender.acquire_destination_lock(ip, port)
        try:
            with self.assertRaisesRegex(RuntimeError, '仍在退出'):
                sender.acquire_destination_lock(ip, port)
        finally:
            first.close()
        try:
            second = sender.acquire_destination_lock(ip, port)
            second.close()
        finally:
            lock_path.unlink(missing_ok=True)

    def test_ssh_failure_retries_and_stop_ends_owned_child(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            fake = directory / 'ssh'
            fake.write_text('''#!/usr/bin/env python3
import json, os, sys, time
sys.stdin.buffer.read()
path = os.environ['GCS_FAKE_SSH_LOG']
with open(path, 'a') as f:
    f.write(str(os.getpid()) + ' ' + sys.argv[-1] + '\\n')
with open(path) as f:
    attempts = sum(1 for _ in f)
if attempts == 1:
    sys.exit(255)
print(json.dumps({'phase':'STREAMING','stamp':time.time()}), flush=True)
while True:
    time.sleep(.1)
''')
            fake.chmod(0o755)
            attempt_log = directory / 'attempts.log'
            env = dict(os.environ, PATH=str(directory) + os.pathsep + os.environ.get('PATH', ''),
                       GCS_FAKE_SSH_LOG=str(attempt_log))
            output = directory / 'supervisor.log'
            with output.open('w') as log:
                proc = subprocess.Popen([sys.executable, str(HERE / 'manage.py'), 'run', '--host', '127.0.0.1',
                                         '--pc-ip', '127.0.0.1', '--port', '19000', '--with-pose',
                                         '--pose-port', '19001'], stdout=log, stderr=subprocess.STDOUT,
                                        env=env, start_new_session=True)
                try:
                    deadline = time.monotonic() + 8
                    attempts = []
                    while time.monotonic() < deadline:
                        if proc.poll() is not None:
                            self.fail('重连管理进程提前退出')
                        if attempt_log.exists():
                            attempts = attempt_log.read_text().splitlines()
                            if len(attempts) >= 2:
                                break
                        time.sleep(.1)
                    self.assertGreaterEqual(len(attempts), 2, '未重试 SSH')
                    self.assertIn('--pc-ip 127.0.0.1 --port 19000 --pose-port 19001', attempts[1])
                    child_pid = int(attempts[1].split()[0])
                    proc.terminate()
                    proc.wait(timeout=5)
                    with self.assertRaises(ProcessLookupError):
                        os.kill(child_pid, 0)
                    self.assertIn('RECONNECTING', output.read_text())
                finally:
                    if proc.poll() is None:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()


if __name__ == '__main__':
    unittest.main()
