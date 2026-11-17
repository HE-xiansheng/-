#!/usr/bin/env python3
"""Physical ROS2 check on an owned temporary or existing sensor network."""
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tarfile
import uuid

repo = Path(__file__).resolve().parents[1]
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=False)
lidar_ip = str(ipaddress.IPv4Address(sys.argv[2] if len(sys.argv) > 2 else '192.168.1.158'))
if not lidar_ip.startswith('192.168.1.') or lidar_ip in ('192.168.1.51', '192.168.1.224', '192.168.1.235'):
    raise ValueError('unexpected sensor address')
flags = set(sys.argv[3:])
if flags - {'--existing-network', '--passive-planner', '--ready-start'}:
    raise ValueError('unknown check option')
existing_network = '--existing-network' in flags
ssh = ['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ServerAliveInterval=15',
       '-o','ServerAliveCountMax=3','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts',
       '-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
remote = '/tmp/livox-native-check'
original = '832dd3b5-8a1a-3c4b-93db-c09f90bdd28a'
profile = str(uuid.uuid4())
profile_name = 'ROS2-Livox-test-'+profile[:8]
created = network_created = False
process = None
result = {'passed': False, 'profile_name': profile_name, 'lidar_ip': lidar_ip, 'cleanup_errors': [], 'existing_network': existing_network}
files = ['test_livox_native_ros2.py','cleanup_livox_live_check.py','run_livox_native_check.py']
manifest = {n:hashlib.sha256((repo/'tools'/n).read_bytes()).hexdigest() for n in files}
if '--ready-start' in flags:
    manifest['sensor_readiness.py'] = hashlib.sha256((repo/'platform/01_apps/fast_lio/sensor_readiness.py').read_bytes()).hexdigest()
(out/'inputs-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')


def run(command, name, timeout=20, check=True):
    p = subprocess.run(ssh+[command], capture_output=True, text=True, timeout=timeout)
    (out/(name+'.log')).write_text(p.stdout+p.stderr)
    if check:
        p.check_returncode()
    return p


try:
    active_connections = run('nmcli -t -f UUID,DEVICE connection show --active', 'active-connections-before').stdout.splitlines()
    active_ethernet = [line.split(':',1)[0] for line in active_connections if line.endswith(':enP3p49s0')]
    if active_ethernet:
        original = active_ethernet[0]
    result['original_profile_uuid'] = original
    run('date -Is; ip -j address; ip -j route; nmcli -g connection.id,connection.uuid,ipv4.method,ipv4.addresses,ipv4.routes,connection.autoconnect connection show '+original,
        'network-before')
    preflight = run('pgrep -af "[l]ivox_ros_driver2_node|[f]astlio_mapping|[v]ision2mavros" || true', 'processes-before')
    if preflight.stdout.strip():
        raise RuntimeError('refusing to overlap existing driver/localization processes')
    if not existing_network:
        add = ('nmcli connection add save no type ethernet ifname enP3p49s0 con-name '+profile_name+' '
               'connection.autoconnect no 802-3-ethernet.mac-address C0:74:2B:F9:C0:D8 '
               'ipv4.method manual ipv4.addresses 192.168.1.51/32 ipv4.routes '+lidar_ip+'/32 '
               'ipv4.never-default yes ipv4.ignore-auto-dns yes ipv4.dad-timeout 3000 ipv4.may-fail no ipv6.method disabled')
        run(add, 'temporary-profile-add')
        network_created = True
        result['profile_uuid'] = run('nmcli -g connection.uuid connection show id '+profile_name,'temporary-profile-id').stdout.strip()
        run('nmcli -w 15 connection up id '+profile_name+' ifname enP3p49s0', 'temporary-profile-up', timeout=22)
    network = run('ip -j address; ip -j route; ip route get '+lidar_ip+'; ip route get 1.1.1.1; nmcli -g GENERAL.STATE,IP4.ADDRESS device show enP3p49s0', 'network-active')
    if lidar_ip+' dev enP3p49s0 src 192.168.1.51' not in network.stdout:
        raise RuntimeError('sensor route/source does not match the intended isolated route')
    if '1.1.1.1 via 192.168.1.1 dev wlan0' not in network.stdout:
        raise RuntimeError('default management route changed')
    run('ping -I enP3p49s0 -c 3 -W 1 '+lidar_ip, 'lidar-ping', check=False)
    config = json.loads(run('cat /home/orangepi/drone_platform/02_drivers/sensors/config/MID360s_config.json', 'deployed-device-config').stdout)
    if len(config['lidar_configs']) != 1:
        raise RuntimeError('expected one sensor config')
    if existing_network and config['lidar_configs'][0]['ip'] != lidar_ip:
        raise RuntimeError('deployed sensor config does not match requested IP')
    config['lidar_configs'][0]['ip'] = lidar_ip
    (out/'device-config.json').write_text(json.dumps(config,indent=2)+'\n')
    with tarfile.open(out/'inputs.tar','w') as archive:
        for name in files:
            archive.add(repo/'tools'/name, arcname=name)
        archive.add(out/'device-config.json', arcname='device-config.json')
        if '--ready-start' in flags:
            archive.add(repo/'platform/01_apps/fast_lio/sensor_readiness.py', arcname='sensor_readiness.py')
    with (out/'inputs.tar').open('rb') as f:
        subprocess.run(ssh+['mkdir '+remote+' && tar -xf - -C '+remote], stdin=f, check=True, timeout=20)
    created = True
    with (out/'consumer.log').open('wb') as log:
        command = ('source /home/orangepi/drone_platform/env.sh && ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1 '
                   'ROS_LOG_DIR='+remote+'/ros-log timeout --signal=INT --kill-after=15s 110s python3 -u '
                   +remote+'/test_livox_native_ros2.py'
                   + (' --deployed-config' if existing_network else '')
                   + (' --passive-planner' if '--passive-planner' in flags else '')
                   + (' --ready-start' if '--ready-start' in flags else ''))
        process = subprocess.Popen(ssh+[command], stdin=subprocess.DEVNULL, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        process.wait(timeout=150)
    result['consumer_exit'] = process.returncode
except Exception as exc:
    result['error'] = repr(exc)
finally:
    if process and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
    if created:
        try:
            cleanup = run('python3 '+remote+'/cleanup_livox_live_check.py '+remote, 'owned-cleanup', timeout=25)
            result['owned_processes_cleaned'] = cleanup.returncode == 0
            run('date -Is; ip -j address; ip -j route; journalctl -k -b --since "-5 min" --no-pager | grep -Ei "enP3p49s0|r8125" | tail -60', 'network-after-test')
            with (out/'evidence.tar.gz').open('wb') as f:
                subprocess.run(ssh+['tar -czf - -C /tmp livox-native-check'], stdout=f, check=True, timeout=40)
            subprocess.run(['tar','-xzf',str(out/'evidence.tar.gz'),'-C',str(out)],check=True,timeout=20)
            result['evidence_retrieved'] = True
            run('rm -rf -- '+remote, 'remove-remote')
            result['remote_removed'] = True
        except Exception as exc:
            result['cleanup_errors'].append(repr(exc))
    if network_created:
        for command, name in [('nmcli -w 5 connection down id '+profile_name, 'temporary-profile-down'),
                              ('nmcli connection delete id '+profile_name, 'temporary-profile-delete'),
                              ('nmcli -w 0 connection up uuid '+original+' ifname enP3p49s0', 'restore-original-profile')]:
            try:
                p = run(command,name,timeout=12,check=False)
                if p.returncode:
                    result['cleanup_errors'].append(name+': '+str(p.returncode))
            except Exception as exc:
                result['cleanup_errors'].append(name+': '+repr(exc))
    try:
        run('date -Is; ip -j address; ip -j route; nmcli -g NAME,UUID connection show; git -C /home/orangepi/drone_platform status --short; systemctl is-active nxserver', 'final-state')
    except Exception as exc:
        result['cleanup_errors'].append(repr(exc))
    report = out/'livox-native-check/result.json'
    result['passed'] = bool(report.exists() and json.loads(report.read_text()).get('passed')
        and result.get('consumer_exit') == 0 and result.get('owned_processes_cleaned')
        and not result['cleanup_errors'] and 'error' not in result)
    (out/'runner-result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
raise SystemExit(0 if result['passed'] else 1)
