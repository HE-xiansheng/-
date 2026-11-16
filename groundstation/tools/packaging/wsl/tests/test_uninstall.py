"""在临时目录验证卸载归属、数据保留和失败分支；不接触真实安装。"""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

WSL = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('gcs_uninstall', WSL / 'uninstall_in_wsl.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class UninstallTests(unittest.TestCase):
    def fixture(self, home, suffix='', mtime=1, previous=None):
        prefix = home / '.local/opt' / ('research-flight-groundstation' + suffix)
        prefix.mkdir(parents=True)
        binary = prefix / 'build/linux/customgeometry'; binary.parent.mkdir(parents=True); binary.write_text('fixture')
        for name in ('preflight/bridge.py', 'pointcloud-only/manage.py'):
            script = prefix / 'dianyunGCS-master/tools' / name
            script.parent.mkdir(parents=True, exist_ok=True); script.write_text('# fixture stop')
        shipped = {str(p.relative_to(prefix)): 'unused' for p in prefix.rglob('*') if p.is_file()}
        (prefix / 'payload-manifest.json').write_text(json.dumps({'files': shipped}))
        shortcut = home / '.local/share/applications/research-flight-groundstation.desktop'
        shortcut.parent.mkdir(parents=True, exist_ok=True)
        content = 'fixture ' + str(prefix)
        shortcut.write_text(content)
        marker = prefix / '.installation.json'
        marker.write_text(json.dumps({'product': module.PRODUCT, 'shortcuts': [{'path': str(shortcut), 'installed': content, 'previous': previous}]}))
        os.utime(marker, ns=(mtime, mtime))
        return prefix, content, shortcut

    def run_uninstall(self, home, proc, calls):
        def run(args, **kwargs):
            calls.append(args)
            return SimpleNamespace(returncode=0, stderr='')
        return module.uninstall(home, 'fixture-user', os.getuid(), proc, run)

    def test_versions_and_user_data_preserved_without_following_links(self):
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp); proc = home / 'proc'; proc.mkdir()
            old, content, shortcut = self.fixture(home, mtime=1, previous='original other shortcut')
            new, _, _ = self.fixture(home, '-wsl-1.0.20261110.2', mtime=2, previous=content)
            for prefix in (old, new):
                (prefix / '.pointcloud-link').mkdir(); (prefix / '.pointcloud-link/connection.json').write_text('{"host":"fixture"}')
                (prefix / 'build/linux/logs').mkdir(); (prefix / 'build/linux/logs/flight.log').write_text('flight log')
                (prefix / 'manual.bag').write_bytes(b'flight data')
            outside = home / 'outside'; outside.mkdir(); (outside / 'keep.txt').write_text('outside data')
            (new / 'external-link').symlink_to(outside, target_is_directory=True)
            foreign = home / '.local/opt/research-flight-groundstation-foreign'; foreign.mkdir(); (foreign / 'keep').write_text('foreign')
            (home / '.ssh').mkdir(); (home / '.ssh/known_hosts').write_text('host keys')
            settings = home / '.config/customgeometry.conf'; settings.parent.mkdir(); settings.write_text('user config')
            calls = []
            result = self.run_uninstall(home, proc, calls)
            self.assertEqual(result['removed'], [str(new), str(old)])
            self.assertFalse(old.exists() or new.exists())
            self.assertEqual(len(calls), 4)
            self.assertTrue(all(args[-1] == 'stop' and args[0] == '/usr/bin/python3' for args in calls))
            self.assertEqual(shortcut.read_text(), 'original other shortcut')
            for record in result['saved']:
                saved = Path(record['directory'])
                self.assertEqual((saved / 'manual.bag').read_bytes(), b'flight data')
                self.assertEqual((saved / 'build/linux/logs/flight.log').read_text(), 'flight log')
                self.assertTrue((saved / '.pointcloud-link/connection.json').is_file())
            self.assertTrue((Path(result['saved'][0]['directory']) / 'external-link').is_symlink())
            self.assertEqual((outside / 'keep.txt').read_text(), 'outside data')
            self.assertTrue((foreign / 'keep').is_file() and (home / '.ssh/known_hosts').is_file() and settings.is_file())
            repeat = self.run_uninstall(home, proc, [])
            self.assertEqual(repeat['removed'], [])
            self.assertTrue(Path(result['saved'][0]['directory']).is_dir())

    def test_live_gui_blocks_all_mutation(self):
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp); prefix, _, _ = self.fixture(home)
            proc = home / 'proc'; (proc / '321').mkdir(parents=True)
            (proc / '321/exe').symlink_to(prefix / 'build/linux/customgeometry')
            calls = []
            with self.assertRaisesRegex(RuntimeError, '先关闭'):
                self.run_uninstall(home, proc, calls)
            self.assertEqual(calls, [])
            self.assertTrue(prefix.is_dir())

    def test_backup_failure_keeps_every_program_and_original_data(self):
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp); old, _, _ = self.fixture(home)
            new, _, _ = self.fixture(home, '-wsl-2', mtime=2)
            (new / 'important.bag').write_text('keep')
            proc = home / 'proc'; proc.mkdir()
            with patch.object(module.shutil, 'copy2', side_effect=OSError('disk full')):
                with self.assertRaisesRegex(OSError, 'disk full'):
                    self.run_uninstall(home, proc, [])
            self.assertTrue(old.is_dir() and new.is_dir())
            self.assertEqual((new / 'important.bag').read_text(), 'keep')

    def test_invalid_manifest_or_owner_is_never_removed(self):
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp); prefix, _, _ = self.fixture(home)
            with self.assertRaisesRegex(RuntimeError, '归属'):
                module.installations(home, os.getuid() + 1)
            (prefix / 'payload-manifest.json').write_text('{"files":{"../../outside":"x"}}')
            with self.assertRaisesRegex(RuntimeError, '越界'):
                module.installations(home, os.getuid())
            self.assertTrue(prefix.is_dir())

    def test_directory_link_and_modified_shortcut_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp); prefix, _, shortcut = self.fixture(home)
            shortcut.write_text('user replaced this shortcut')
            alias = prefix.parent / 'research-flight-groundstation-wsl-linked'; alias.symlink_to(prefix, target_is_directory=True)
            proc = home / 'proc'; proc.mkdir()
            result = self.run_uninstall(home, proc, [])
            self.assertEqual(result['removed'], [str(prefix)])
            self.assertTrue(alias.is_symlink())
            self.assertEqual(shortcut.read_text(), 'user replaced this shortcut')

    def test_stop_failure_keeps_program(self):
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp); prefix, _, _ = self.fixture(home); proc = home / 'proc'; proc.mkdir()
            with self.assertRaisesRegex(RuntimeError, '后台未停止'):
                module.uninstall(home, 'fixture', os.getuid(), proc, lambda *a, **k: SimpleNamespace(returncode=1, stderr='fixture denied'))
            self.assertTrue(prefix.is_dir())

    def test_root_entry_drops_to_recorded_user_before_mutation(self):
        account = SimpleNamespace(pw_dir='/home/fixture-user', pw_uid=1234)
        with patch.object(module.os, 'geteuid', return_value=0), patch.object(module.pwd, 'getpwnam', return_value=account), patch.object(module.subprocess, 'run') as run:
            with patch('sys.argv', ['uninstall_in_wsl.py', '--user', 'fixture-user', '--result-file', '/mnt/c/中文 path/result.json']):
                module.main()
            args = run.call_args[0][0]
            self.assertEqual(args[:6], ['runuser', '-u', 'fixture-user', '--', 'env', 'HOME=/home/fixture-user'])
            self.assertEqual(args[-1], '/mnt/c/中文 path/result.json')


if __name__ == '__main__':
    unittest.main()
