#!/usr/bin/env python3
"""Computer-owned runner; all physical ground-test records are retrieved here."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tarfile

repo = Path(__file__).resolve().parents[1]
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=False)
extended = '--extended-sync' in sys.argv[2:]
if set(sys.argv[2:]) - {'--extended-sync'}:
    raise ValueError('unknown option')
remote = '/tmp/px4-ev-ground-check'
ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=15',
       '-o', 'ServerAliveCountMax=3',
       '-o', 'UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts',
       '-o', 'HostKeyAlias=opi-ros2-emmc', 'orangepi@192.168.1.224']
inputs = {name: repo/'tools'/name for name in ('px4_ground_link.py', 'test_px4_live_localization_ros2.py',
          'cleanup_livox_live_check.py', 'run_px4_live_localization.py')}
inputs['sensor_readiness.py'] = repo/'platform/01_apps/fast_lio/sensor_readiness.py'
inputs['common.py'] = Path('/home/tyzk/drone-migration-work/px4-sitl-venv/lib/python3.8/site-packages/pymavlink/dialects/v20/common.py')
(out/'inputs-manifest.json').write_text(json.dumps({name: hashlib.sha256(path.read_bytes()).hexdigest()
                                                  for name, path in inputs.items()}, indent=2)+'\n')
result = {'passed': False, 'cleanup_errors': []}
created = False


def run(command, name, timeout=20, check=True):
    p = subprocess.run(ssh+[command], capture_output=True, text=True, timeout=timeout)
    (out/(name+'.log')).write_text(p.stdout+p.stderr)
    if check:
        p.check_returncode()
    return p


try:
    before = run('date -Is; git -C /home/orangepi/drone_platform rev-parse HEAD; '
                 'git -C /home/orangepi/drone_platform status --porcelain; '
                 'sha256sum /home/orangepi/drone_platform/02_drivers/sensors/sdk2/lib/liblivox_lidar_sdk_shared.so; '
                 'ip route get 192.168.1.158; ls -l /dev/serial/by-id', 'before')
    if '292c8fdd45ad82feb960ee3c6aa9708f74e2e4362dedcc5973178616a3b437b3' not in before.stdout:
        raise RuntimeError('installed SDK differs from accepted fix')
    active = run('pgrep -af "[m]avros_node|[l]ivox_ros_driver2_node|[f]astlio_mapping|[v]ision2mavros" || true', 'processes-before')
    if active.stdout.strip():
        raise RuntimeError('refusing to overlap active sensor/MAVROS processes')
    run('mkdir '+remote, 'create')
    created = True
    with tarfile.open(out/'inputs.tar', 'w') as archive:
        for name, path in inputs.items():
            archive.add(path, arcname=name)
    with (out/'inputs.tar').open('rb') as payload:
        p = subprocess.run(ssh+['tar -xf - -C '+remote], stdin=payload, capture_output=True, timeout=20)
        (out/'upload.stderr.log').write_bytes(p.stderr)
        p.check_returncode()
    command = ('source /home/orangepi/drone_platform/env.sh && ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 '
               'ROS_LOG_DIR='+remote+'/ros-log timeout --signal=INT --kill-after=15s '+('195s' if extended else '125s')+' python3 -u '
               +remote+'/test_px4_live_localization_ros2.py'+(' --extended-sync' if extended else ''))
    p = run(command, 'fixture', timeout=220 if extended else 150, check=False)
    result['fixture_exit'] = p.returncode
except Exception as exc:
    result['error'] = repr(exc)
finally:
    if created:
        try:
            run('python3 '+remote+'/cleanup_livox_live_check.py '+remote, 'owned-cleanup', timeout=25)
            result['owned_processes_cleaned'] = True
            with (out/'evidence.tar.gz').open('wb') as payload:
                subprocess.run(ssh+['tar -czf - -C /tmp px4-ev-ground-check'], stdout=payload, check=True, timeout=40)
            subprocess.run(['tar', '-xzf', str(out/'evidence.tar.gz'), '-C', str(out)], check=True, timeout=20)
            result['evidence_retrieved'] = True
            run('rm -rf -- '+remote, 'remove')
            result['remote_removed'] = True
        except Exception as exc:
            result['cleanup_errors'].append(repr(exc))
    try:
        run('date -Is; git -C /home/orangepi/drone_platform status --porcelain; '
            'systemctl is-active nxserver; ip route get 192.168.1.158; '
            'pgrep -af "[m]avros_node|[l]ivox_ros_driver2_node|[f]astlio_mapping|[v]ision2mavros" || true', 'final-state')
    except Exception as exc:
        result['cleanup_errors'].append(repr(exc))
    report = out/'px4-ev-ground-check/result.json'
    result['passed'] = bool(report.exists() and json.loads(report.read_text()).get('passed')
                            and result.get('fixture_exit') == 0 and result.get('owned_processes_cleaned')
                            and not result['cleanup_errors'] and 'error' not in result)
    (out/'runner-result.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result))
raise SystemExit(0 if result['passed'] else 1)
