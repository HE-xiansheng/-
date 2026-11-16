"""真实安装载荷后卸载临时安装，检查窗口保护及数据保留；不访问机载。"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import pwd
import signal
import subprocess
import tempfile
import time
from unittest.mock import patch

parser = argparse.ArgumentParser()
parser.add_argument('installer', type=Path)
parser.add_argument('--report', type=Path, required=True)
args = parser.parse_args()
base = Path(tempfile.mkdtemp(prefix='gcs-uninstall-payload-'))
home = base / '用户 home'; home.mkdir()
prefix = home / '.local/opt/research-flight-groundstation'
apps = home / '.local/share/applications'
subprocess.run(['bash', str(args.installer), '--prefix', str(prefix), '--applications-dir', str(apps), '--no-desktop'], check=True, capture_output=True, text=True, timeout=90)
manifest = json.loads((prefix / 'payload-manifest.json').read_text())
for relative, digest in manifest['files'].items():
    assert hashlib.sha256((prefix / relative).read_bytes()).hexdigest() == digest, relative
(prefix / '.pointcloud-link').mkdir()
(prefix / '.pointcloud-link/connection.json').write_text('{"host":"fixture"}')
(prefix / 'build/linux/logs').mkdir()
(prefix / 'build/linux/logs/keep.log').write_text('flight log')
flight = home / '.local/share/customgeometry/flight-records/demo.bag'; flight.parent.mkdir(parents=True); flight.write_bytes(b'flight data outside installation')
settings = home / '.config/customgeometry.conf'; settings.parent.mkdir(); settings.write_text('saved user settings')
(home / '.ssh').mkdir(); (home / '.ssh/known_hosts').write_text('keep fingerprints')
fake = base / 'bin'; fake.mkdir()
(fake / 'ssh').write_text('#!/bin/sh\nexit 255\n'); (fake / 'ssh').chmod(0o755)
env = dict(os.environ, HOME=str(home), XDG_CONFIG_HOME=str(home / '.config'), XDG_DATA_HOME=str(home / '.local/share'), XDG_CACHE_HOME=str(home / '.cache'), QT_QPA_PLATFORM='offscreen', QT_OPENGL='software', LIBGL_ALWAYS_SOFTWARE='1', PATH=str(fake) + ':' + os.environ['PATH'])
env.pop('GCS_SSH_PASSWORD', None)
path = Path(__file__).resolve().parents[1] / 'uninstall_in_wsl.py'
spec = importlib.util.spec_from_file_location('gcs_uninstall_actual', path)
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
log = base / 'gui.log'
with log.open('w') as output:
    proc = subprocess.Popen([str(prefix / 'start-groundstation.sh'), '--no-splash', '--cloud-port', '56218'], env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(3)
        assert proc.poll() is None and '首帧就绪' in log.read_text()
        try:
            module.uninstall(home, pwd.getpwuid(os.getuid()).pw_name, os.getuid())
        except RuntimeError as error:
            assert '先关闭' in str(error)
        else:
            raise AssertionError('Actual running GUI was not detected')
        assert prefix.is_dir()
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM); proc.wait(timeout=8)
calls = []
def run(command, **kwargs):
    calls.append(command)
    return subprocess.run(command, **dict(kwargs, env=env))
with patch.dict(os.environ, env, clear=True):
    result = module.uninstall(home, pwd.getpwuid(os.getuid()).pw_name, os.getuid(), run=run)
assert result['success'] and not prefix.exists()
assert len(calls) == 2 and all(command[-1] == 'stop' for command in calls)
assert not (apps / 'research-flight-groundstation.desktop').exists()
backup = Path(result['saved'][0]['directory'])
assert (backup / '.pointcloud-link/connection.json').is_file()
assert (backup / 'build/linux/logs/keep.log').read_text() == 'flight log'
assert flight.read_bytes() == b'flight data outside installation'
assert settings.read_text() == 'saved user settings'
assert (home / '.ssh/known_hosts').read_text() == 'keep fingerprints'
assert module.uninstall(home, 'fixture', os.getuid())['removed'] == []
report = {'passed': True, 'version': manifest['version'], 'real_install_manifest_hashes': True, 'actual_gui_blocks_uninstall': True,
          'actual_local_telemetry_stop': True, 'program_and_owned_shortcut_removed': True, 'data_log_config_and_ssh_preserved': True,
          'repeat_keeps_backup': True, 'directory': str(base), 'scope': 'Ubuntu20.04 temporary installation; fake SSH; no aircraft or Windows access'}
args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2))
print(json.dumps(report, ensure_ascii=False))
