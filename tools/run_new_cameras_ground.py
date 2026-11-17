#!/usr/bin/env python3
"""Own a temporary gimbal NIC profile and actual camera check on .224."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import uuid

repo = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('output')
parser.add_argument('--installed', action='store_true', help='verify installed sources/binaries without candidate overlays')
parser.add_argument('--build-only', action='store_true', help='compile candidates only; do not configure networking or open devices')
parser.add_argument('--without-usb', action='store_true', help='verify Gemini/gimbal only, recording independent USB as untested')
parser.add_argument('--prebuilt', type=Path, help='reuse a retrieved build with identical source hashes')
parser.add_argument('--skip-compat', action='store_true', help='retain prior unchanged legacy-helper regression; check real services below')
args = parser.parse_args()
out = Path(args.output).resolve()
out.mkdir(parents=True, exist_ok=False)
remote = '/tmp/new-camera-ground-check'
ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=15',
       '-o', 'ServerAliveCountMax=3', '-o',
       'UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts',
       '-o', 'HostKeyAlias=opi-ros2-emmc', 'orangepi@192.168.1.224']
profile = 'ROS2-Gimbal-check-'+str(uuid.uuid4())[:8]
created = network_created = False
original = ''
result = {'passed': False, 'cleanup_errors': [], 'temporary_profile': profile}


def run(command, name, timeout=20, check=True):
    p = subprocess.run(ssh+[command], capture_output=True, text=True, timeout=timeout)
    (out/(name+'.log')).write_text(p.stdout+p.stderr)
    if check:
        p.check_returncode()
    return p


try:
    before = run('date -Is; ip -j address; nmcli -t -f NAME,UUID,DEVICE connection show --active; '
                 'ls -l /dev/v4l/by-id; cat /sys/class/net/enx00e09a5af420/address; '
                 'git -C /home/orangepi/drone_platform status --porcelain', 'before')
    if '00:e0:9a:5a:f4:20' not in before.stdout:
        raise RuntimeError('physical gimbal NIC identity differs')
    active = run('pgrep -af "[u]sb_cam_node|[r]tsp_hw_pub_ultra|[o]rbbec_camera_node|[m]ediamtx" || true', 'processes-before')
    if active.stdout.strip():
        raise RuntimeError('refusing to overlap active camera processes')
    if not args.build_only:
        original = run('nmcli -g GENERAL.CON-UUID device show enx00e09a5af420', 'original-profile').stdout.strip()
        run('nmcli connection add save no type ethernet ifname enx00e09a5af420 con-name '+profile+
            ' connection.autoconnect no 802-3-ethernet.mac-address 00:E0:9A:5A:F4:20 '
            'ipv4.method manual ipv4.addresses 192.168.144.100/24 ipv4.never-default yes '
            'ipv4.ignore-auto-dns yes ipv4.dad-timeout 3000 ipv4.may-fail no ipv6.method disabled', 'network-create')
        network_created = True
        run('nmcli -w 15 connection up id '+profile+' ifname enx00e09a5af420', 'network-up', timeout=22)
        routes = run('ip route get 192.168.144.108; ip route get 192.168.1.158; ip route get 1.1.1.1', 'network-active')
        if ('192.168.144.108 dev enx00e09a5af420 src 192.168.144.100' not in routes.stdout
            or '192.168.1.158 dev enP3p49s0 src 192.168.1.51' not in routes.stdout
            or '1.1.1.1 via 192.168.1.1 dev wlan0' not in routes.stdout):
            raise RuntimeError('unexpected management or sensor route')
        run('ping -I enx00e09a5af420 -c 3 -W 1 192.168.144.108', 'gimbal-ping')
    run('mkdir '+remote, 'create')
    created = True
    inputs = {name: repo/'tools'/name for name in ('test_new_cameras_ros2.py', 'test_usb_camera_ros2.py', 'test_usb_timestamp.cpp',
              'cleanup_livox_live_check.py', 'run_new_cameras_ground.py')}
    for p in (repo/'tools/gimbal_delivery_observer').iterdir():
        if p.is_file():
            inputs['observer-src/'+p.name] = p
    module = repo/'platform/02_drivers/usb_camera'
    for name in ('launch/usb_camera.launch.py', 'config/usb_cam.yaml', 'capture_compat.py'):
        inputs['candidate-usb/'+name] = module/name
    if not args.installed:
        for prefix, source in [('candidate-usb/workspace/src/usb_cam', module/'workspace/src/usb_cam'),
                               ('candidate-mpp/src/rtsp_test', repo/'platform/01_apps/mission_demo/workspace/src/rtsp_test')]:
            for source_file in source.rglob('*'):
                if source_file.is_file():
                    inputs[prefix+'/'+str(source_file.relative_to(source))] = source_file
    (out/'inputs-manifest.json').write_text(json.dumps({name: hashlib.sha256(p.read_bytes()).hexdigest()
                                                      for name, p in inputs.items()}, indent=2)+'\n')
    if args.prebuilt:
        previous = json.loads((args.prebuilt/'inputs-manifest.json').read_text())
        for name, p in inputs.items():
            if name.startswith(('candidate-usb/workspace/src/', 'candidate-mpp/')):
                if previous.get(name) != hashlib.sha256(p.read_bytes()).hexdigest():
                    raise RuntimeError('prebuilt source differs: '+name)
    with tarfile.open(out/'inputs.tar', 'w') as archive:
        for name, p in inputs.items():
            archive.add(p, arcname=name)
    with (out/'inputs.tar').open('rb') as payload:
        p = subprocess.run(ssh+['tar -xf - -C '+remote], stdin=payload, capture_output=True, timeout=20)
        (out/'upload.stderr.log').write_bytes(p.stderr)
        p.check_returncode()
    if args.prebuilt:
        with (args.prebuilt/'evidence.tar.gz').open('rb') as payload:
            subprocess.run(ssh+['tar -xzf - -C /tmp new-camera-ground-check/candidate-install'],
                           stdin=payload, check=True, timeout=25)
        result['prebuilt_source'] = str(args.prebuilt)
    setup = ('source /home/orangepi/drone_platform/env.sh && '
             'source /home/orangepi/drone_platform/02_drivers/usb_camera/workspace/install/setup.bash && ')
    if not args.installed:
        if not args.prebuilt:
            setup += ('CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base '+remote+'/candidate-build-log build '
                  '--base-paths '+remote+'/candidate-usb/workspace/src '+remote+'/candidate-mpp/src '
                  '--build-base '+remote+'/candidate-build --install-base '+remote+'/candidate-install '
                  '--packages-select usb_cam rtsp_test --executor sequential '
                  '--cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && ')
        setup += ('source '+remote+'/candidate-install/setup.bash && '
                  'g++ -std=c++17 -I'+remote+'/candidate-usb/workspace/src/usb_cam/include '
                  +remote+'/test_usb_timestamp.cpp -Wl,--wrap=gettimeofday -Wl,--wrap=clock_gettime '
                  '-o '+remote+'/test-usb-timestamp && '+remote+'/test-usb-timestamp && ')
    command = (setup+
               'cmake -S '+remote+'/observer-src -B '+remote+'/observer-build && '
               'cmake --build '+remote+'/observer-build -j2 && ')
    if not args.skip_compat and not args.build_only:
        command += ('mkdir '+remote+'/compat && '
                    'ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR='+remote+'/compat/ros-log '
                    'python3 '+remote+'/test_usb_camera_ros2.py '+remote+'/compat '+remote+'/candidate-usb && ')
    if args.build_only:
        command += 'true'
    else:
        command += ('CAMERA_CHECK_INSTALLED='+('1' if args.installed else '0')+' '
                    'CAMERA_CHECK_NO_USB='+('1' if args.without_usb else '0')+' '
                    'ROS_DOMAIN_ID=189 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR='+remote+'/ros-log '
                    'timeout --signal=INT --kill-after=15s 135s python3 -u '+remote+'/test_new_cameras_ros2.py')
    p = run(command, 'fixture', timeout=330, check=False)
    result['fixture_exit'] = p.returncode
except Exception as exc:
    result['error'] = repr(exc)
finally:
    if created:
        try:
            run('python3 '+remote+'/cleanup_livox_live_check.py '+remote, 'owned-cleanup', timeout=25)
            result['owned_processes_cleaned'] = True
            with (out/'evidence.tar.gz').open('wb') as payload:
                subprocess.run(ssh+['tar -czf - -C /tmp new-camera-ground-check'], stdout=payload, check=True, timeout=40)
            subprocess.run(['tar', '-xzf', str(out/'evidence.tar.gz'), '-C', str(out)], check=True, timeout=20)
            result['evidence_retrieved'] = True
            run('rm -rf -- '+remote, 'remove')
            result['remote_removed'] = True
        except Exception as exc:
            result['cleanup_errors'].append(repr(exc))
    if network_created:
        commands = [('nmcli -w 5 connection down id '+profile, 'network-down'),
                    ('nmcli connection delete id '+profile, 'network-delete')]
        if original and original != '--':
            commands.append(('nmcli -w 0 connection up uuid '+original+' ifname enx00e09a5af420', 'network-restore'))
        for command, name in commands:
            try:
                run(command, name)
            except Exception as exc:
                result['cleanup_errors'].append(repr(exc))
    try:
        run("journalctl -k -b --since '2026-11-17 14:28:00' --no-pager", 'kernel-after')
        run('date -Is; ip -brief address; ip route get 192.168.1.158; ip route get 1.1.1.1; '
            'git -C /home/orangepi/drone_platform status --porcelain; systemctl is-active nxserver', 'final-state')
    except Exception as exc:
        result['cleanup_errors'].append(repr(exc))
    report = out/'new-camera-ground-check/result.json'
    result['build_only'] = args.build_only
    result['passed'] = bool((args.build_only or (report.exists() and json.loads(report.read_text()).get('passed')))
        and result.get('fixture_exit') == 0 and result.get('owned_processes_cleaned')
        and not result['cleanup_errors'] and 'error' not in result)
    (out/'runner-result.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result))
raise SystemExit(0 if result['passed'] else 1)
