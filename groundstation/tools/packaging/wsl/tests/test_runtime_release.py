"""验证实测通信修复、运行文件复用校验和旧版本地遥测切换；不连接机载。"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

WSL = Path(__file__).resolve().parents[1]
SRC = WSL.parents[2]
sys.path.insert(0, str(SRC / 'tools/communication'))
sys.path.insert(0, str(SRC / 'tools/preflight'))
import ssh_session
import bridge


def load(path):
    spec = importlib.util.spec_from_file_location('gcs_upgrade_prepare', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


upgrade = load(WSL / 'upgrade_prepare.py')


class RuntimeRelease(unittest.TestCase):
    def test_fixed_askpass_survives_deleted_gui_helper(self):
        with patch.dict(os.environ, {'GCS_SSH_PASSWORD': 'fixture-only-secret', 'SSH_ASKPASS': '/tmp/missing-gui-askpass'}):
            options, env = ssh_session.background_ssh()
            helper = Path(env['SSH_ASKPASS'])
            self.assertTrue(helper.is_file() and os.access(helper, os.X_OK))
            self.assertIn('StrictHostKeyChecking=yes', options)
            self.assertNotIn('fixture-only-secret', helper.read_text())
            output = subprocess.check_output([str(helper), 'Password:'], env=env)
            self.assertEqual(output, b'fixture-only-secret\n')
            self.assertEqual(os.environ['SSH_ASKPASS'], '/tmp/missing-gui-askpass')

    def test_key_and_windows_modes_are_preserved(self):
        with patch.dict(os.environ, {}, clear=True):
            options, env = ssh_session.background_ssh()
            self.assertIn('BatchMode=yes', options)
            self.assertNotIn('SSH_ASKPASS', env)
        with patch.object(ssh_session.sys, 'platform', 'win32'), patch.dict(os.environ, {'GCS_SSH_PASSWORD': 'fixture', 'SSH_ASKPASS': 'C:/app/ssh-askpass.exe'}):
            options, env = ssh_session.background_ssh()
            self.assertEqual(env['SSH_ASKPASS'], 'C:/app/ssh-askpass.exe')
            self.assertNotIn('StrictHostKeyChecking=yes', options)

    def test_local_udp_preserves_real_multikilobyte_payload(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 0)); receiver.settimeout(2)
            with bridge.local_udp_socket() as sender:
                payload = json.dumps({'type': 'PFS1', 'text': '真实状态回放' * 250}, ensure_ascii=False).encode()
                self.assertGreater(len(payload), 4000)
                sender.sendto(payload, receiver.getsockname())
                self.assertEqual(receiver.recv(60000), payload)
        fake = SimpleNamespace(setsockopt=lambda *args: self.fail('Windows must not bind Linux lo'))
        with patch.object(bridge.sys, 'platform', 'win32'), patch.object(bridge.socket, 'socket', return_value=fake):
            self.assertIs(bridge.local_udp_socket(), fake)

    def test_wsl_uses_lo_and_reports_binding_failure(self):
        fake = unittest.mock.Mock()
        with patch.object(bridge.sys, 'platform', 'linux'), patch.object(bridge.os, 'uname', return_value=SimpleNamespace(release='6.6-microsoft-standard-WSL2')), patch.object(bridge.socket, 'socket', return_value=fake):
            bridge.local_udp_socket()
            fake.setsockopt.assert_called_once_with(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, b'lo\0')
            fake.setsockopt.side_effect = OSError('fixture permission failure')
            with patch.object(bridge.sys, 'stderr') as stderr:
                bridge.local_udp_socket()
                self.assertTrue(stderr.write.called)

    def test_same_binary_does_not_hide_changed_runtime_or_lost_execute_bit(self):
        text = (WSL / 'install-in-wsl.sh').read_text()
        code = text.split('matches_install() {', 1)[1].split("<<'PY'\n", 1)[1].split('\nPY\n}', 1)[0]
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            binary = root / 'build/linux/customgeometry'; binary.parent.mkdir(parents=True); binary.write_bytes(b'unchanged binary')
            helper = root / 'dianyunGCS-master/tools/communication/gcs-askpass'; helper.parent.mkdir(parents=True); helper.write_text('fixture helper'); helper.chmod(0o700)
            video = root / 'dianyunGCS-master/tools/desktop/video-bin/gst-launch-1.0'; video.parent.mkdir(parents=True); video.write_text('fixture decoder wrapper'); video.chmod(0o755)
            (root / '.installation.json').write_text(json.dumps({'product': 'research-flight-groundstation'}))
            manifest = {'version': '1.0.20261110.2', 'files': {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in (binary, helper, video)}}
            raw = json.dumps(manifest).encode(); (root / 'payload-manifest.json').write_bytes(raw)
            args = [sys.executable, '-c', code, str(root), manifest['version'], manifest['files']['build/linux/customgeometry'], hashlib.sha256(raw).hexdigest()]
            self.assertEqual(subprocess.run(args).returncode, 0)
            helper.write_text('stale helper')
            self.assertNotEqual(subprocess.run(args).returncode, 0)
            helper.write_text('fixture helper'); helper.chmod(0o600)
            self.assertNotEqual(subprocess.run(args).returncode, 0)
            helper.chmod(0o700); video.chmod(0o644)
            self.assertNotEqual(subprocess.run(args).returncode, 0)
            video.chmod(0o755); (root / 'payload-manifest.json').write_bytes(raw + b' ')
            self.assertNotEqual(subprocess.run(args).returncode, 0)

    def test_upgrade_rejects_live_old_gui_and_stops_only_own_telemetry(self):
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp); base = home / '.local/opt'; old = base / 'research-flight-groundstation'; old.mkdir(parents=True)
            (old / '.installation.json').write_text(json.dumps({'product': 'research-flight-groundstation'}))
            for name in ('preflight/bridge.py', 'pointcloud-only/manage.py'):
                p = old / 'dianyunGCS-master/tools' / name; p.parent.mkdir(parents=True, exist_ok=True); p.write_text('# fixture')
            proc = home / 'proc'; (proc / '123').mkdir(parents=True)
            (proc / '123/exe').symlink_to(old / 'build/linux/customgeometry')
            calls = []
            def run(args, **kwargs):
                calls.append(args); return SimpleNamespace(returncode=0, stderr='')
            with self.assertRaisesRegex(RuntimeError, '关闭旧安装版'):
                upgrade.prepare(home, 'fixture-user', base / 'new', proc, run)
            self.assertEqual(calls, [])
            (proc / '123/exe').unlink()
            self.assertEqual(upgrade.prepare(home, 'fixture-user', base / 'new', proc, run), 1)
            self.assertEqual(len(calls), 2)
            self.assertTrue(all(x[-1] == 'stop' and x[0] == 'runuser' for x in calls))
            self.assertTrue(old.is_dir())
            calls.clear(); self.assertEqual(upgrade.prepare(home, 'fixture-user', old, proc, run), 0)
            self.assertEqual(calls, [])


if __name__ == '__main__':
    unittest.main()
