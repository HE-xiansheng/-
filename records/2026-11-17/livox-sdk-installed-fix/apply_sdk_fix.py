#!/usr/bin/env python3
"""Apply the explicitly authorized, previously tested SDK binary on .224 only."""
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

record = Path(__file__).resolve().parent
archive = Path('/home/tyzk/drone-migration-work/livox-sdk-symbol-build1')
original = '480fd4377886adca14c95256d9ff0a7a1a0817b15f62eba93c965c8b6a68a6d7'
candidate = '292c8fdd45ad82feb960ee3c6aa9708f74e2e4362dedcc5973178616a3b437b3'
for name, expected in [('original', original), ('candidate', candidate)]:
    assert hashlib.sha256((archive/name/'liblivox_lidar_sdk_shared.so').read_bytes()).hexdigest() == expected

ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
       '-o', 'UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts',
       '-o', 'HostKeyAlias=opi-ros2-emmc', 'orangepi@192.168.1.224']
remote = r'''
import ctypes, hashlib, json, os, stat, subprocess, sys
from datetime import datetime, timezone
from pathlib import Path

root = Path('/home/orangepi/drone_platform')
target = root/'02_drivers/sensors/sdk2/lib/liblivox_lidar_sdk_shared.so'
staged = target.with_name('.liblivox_lidar_sdk_shared.so.sdkfix-new')
old_sha = '480fd4377886adca14c95256d9ff0a7a1a0817b15f62eba93c965c8b6a68a6d7'
new_sha = '292c8fdd45ad82feb960ee3c6aa9708f74e2e4362dedcc5973178616a3b437b3'
expected_commit = '7a3c0cb71864d02a5764a22d8e004debb615232e'
protected_sessions = {}

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def command(*args):
    return subprocess.check_output(args, cwd=root, text=True).strip()

def users():
    found = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            data = (proc/'maps').read_bytes()
            if b'liblivox_lidar_sdk_shared.so' in data:
                found.append(int(proc.name))
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            try:
                comm = (proc/'comm').read_text().strip()
                uid = proc.stat().st_uid
            except (FileNotFoundError, ProcessLookupError):
                continue
            if 'livox' in comm or 'fastlio' in comm:
                raise RuntimeError('active protected sensor process '+proc.name)
            if uid == os.getuid():
                if comm not in {'(sd-pam)', 'ssh-agent', 'gpg-agent', 'sshd'}:
                    raise RuntimeError('cannot inspect application process '+proc.name+' '+comm)
                protected_sessions[proc.name] = comm
    return found

assert os.getuid() == 1000 and command('hostname') == 'orangepi5ultra'
assert command('git', 'rev-parse', 'HEAD') == expected_commit
assert command('git', 'status', '--porcelain') == ''
before_stat = target.lstat()
assert stat.S_ISREG(before_stat.st_mode) and before_stat.st_uid == os.getuid()
assert digest(target) == old_sha and not users()
assert not staged.exists() and not staged.is_symlink()
payload = sys.stdin.buffer.read(2*1024*1024)
assert len(payload) == 1131144 and hashlib.sha256(payload).hexdigest() == new_sha
report = {'utc': datetime.now(timezone.utc).isoformat(), 'authorized_by': 'user: 那你先修复一下吧',
          'target': str(target), 'product_commit': expected_commit, 'before_sha256': old_sha,
          'before_mode': oct(stat.S_IMODE(before_stat.st_mode)), 'library_users_before': users(),
          'source_tree_modified': False, 'services_changed': False, 'px4_accessed': False}
created = False
try:
    fd = os.open(staged, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    created = True
    with os.fdopen(fd, 'wb') as output:
        output.write(payload)
        output.flush()
        os.fchmod(output.fileno(), stat.S_IMODE(before_stat.st_mode))
        os.fsync(output.fileno())
    assert digest(staged) == new_sha and digest(target) == old_sha and not users()
    assert target.lstat().st_ino == before_stat.st_ino
    os.replace(staged, target)
    created = False
    dirfd = os.open(target.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(dirfd)
    finally:
        os.close(dirfd)
    assert digest(target) == new_sha
    class Version(ctypes.Structure):
        _fields_ = [('major', ctypes.c_int), ('minor', ctypes.c_int), ('patch', ctypes.c_int)]
    lib = ctypes.CDLL(str(target))
    lib.GetLivoxLidarSdkVer.argtypes = [ctypes.POINTER(Version)]
    lib.GetLivoxLidarSdkVer.restype = None
    version = Version()
    lib.GetLivoxLidarSdkVer(ctypes.byref(version))
    report.update(after_sha256=digest(target), after_mode=oct(stat.S_IMODE(target.stat().st_mode)),
                  after_uid=target.stat().st_uid, after_gid=target.stat().st_gid,
                  runtime_sdk_version=[version.major, version.minor, version.patch],
                  git_status_after=command('git', 'status', '--porcelain'),
                  sensor_route=command('ip', 'route', 'get', '192.168.1.158'),
                  default_route=command('ip', 'route', 'get', '1.1.1.1'),
                  nomachine=command('systemctl', 'is-active', 'nxserver'),
                  protected_session_maps_unreadable=protected_sessions,
                  staging_removed=not staged.exists())
    assert report['runtime_sdk_version'] == [1, 3, 0]
    print(json.dumps(report, indent=2))
finally:
    if created:
        staged.unlink()
'''

with (archive/'candidate/liblivox_lidar_sdk_shared.so').open('rb') as payload:
    process = subprocess.run(ssh+['python3 -c '+shlex.quote(remote)], stdin=payload,
                             capture_output=True, text=True, timeout=35)
(record/'application.stdout.log').write_text(process.stdout)
(record/'application.stderr.log').write_text(process.stderr)
if process.returncode:
    raise SystemExit('SDK application failed; see application.stderr.log (exit '+str(process.returncode)+')')
result = json.loads(process.stdout)
assert result['after_sha256'] == candidate and result['runtime_sdk_version'] == [1, 3, 0]
assert result['git_status_after'] == '' and result['staging_removed']
(record/'application.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
print(json.dumps(result, ensure_ascii=False))
