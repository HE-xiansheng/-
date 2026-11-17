#!/usr/bin/env python3
"""经桌面认证启用已准备的基础服务；凭据仅留在内存和sudo标准输入。"""
import argparse
import getpass
import json
import re
import shlex
from pathlib import Path
import subprocess
import sys
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--reboot-after-check', action='store_true')
parser.add_argument('--resume-installed', action='store_true')
parser.add_argument('--password-stdin', action='store_true')
parser.add_argument('--record-prefix')
args = parser.parse_args()
repo = Path(__file__).resolve().parents[1]
record = repo / 'records/2026-11-17/base-autostart'
remote = '/tmp/drone-base-autostart'
ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=6',
       '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2',
       '-o', 'UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts',
       '-o', 'HostKeyAlias=opi-ros2-emmc', 'orangepi@192.168.1.224']
result = {'passed': False, 'reboot_requested': False}
secret = bytearray()
prefix = args.record_prefix if args.record_prefix is not None else ('resume-' if args.resume_installed else '')
assert re.fullmatch(r'[A-Za-z0-9_-]*', prefix)
observer_binary = Path(json.loads((record/'native-check-inputs.json').read_text())['native-observer']['computer_path'])


def run(command, name, timeout=30, privileged=False, check=True):
    print(json.dumps({'step': name, 'state': 'running'}), flush=True)
    with (record / (prefix + name + '.log')).open('wb') as log:
        process = subprocess.run(ssh + [command],
            input=bytes(secret) if privileged else None,
            stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
    print(json.dumps({'step': name, 'returncode': process.returncode}), flush=True)
    if check:
        process.check_returncode()
    return process


def collect(label):
    run('systemctl show Safina_sensor.service gimbal-rtsp-relay.service '
        '-p Id -p ActiveState -p SubState -p UnitFileState -p NRestarts -p MainPID; '
        'journalctl -b -u Safina_sensor.service -u gimbal-rtsp-relay.service '
        '-n 120 --no-pager -o short-iso; '
        'ip -brief address; ip route get 192.168.1.158; ip route get 192.168.144.108; '
        'ip route get 1.1.1.1; git -C /home/orangepi/drone_platform status --porcelain', label)


def wait_for_services(label):
    deadline = time.monotonic() + 95
    with (record / (prefix + label + '.log')).open('w') as log:
        while time.monotonic() < deadline:
            status = subprocess.run(ssh + ['systemctl is-active Safina_sensor.service '
                                          'gimbal-rtsp-relay.service'],
                                    capture_output=True, text=True, timeout=12)
            log.write(status.stdout + status.stderr)
            log.flush()
            if status.returncode == 0:
                return
            time.sleep(2)
    raise TimeoutError('base services did not become active')


def authenticate():
    if args.password_stdin:
        if sys.stdin.isatty():
            secret.extend(getpass.getpass('Onboard administrator authentication: ').encode() + b'\n')
        else:
            secret.extend(sys.stdin.buffer.readline())
        if not secret or secret == b'\n':
            raise RuntimeError('administrator credential input is empty')
        return
    print(json.dumps({'administrator_password_dialog': 'waiting'}), flush=True)
    prompt = subprocess.run(['zenity', '--password', '--title=验证 .224 基础服务开机启动',
        '--text=请输入机载电脑 orangepi 的管理员密码。\n用于已批准的系统服务部署或重启自启动验证。\n密码不会写入日志。', '--width=520'],
        stdout=subprocess.PIPE, timeout=240)
    if prompt.returncode:
        raise RuntimeError('administrator authentication dialog cancelled')
    secret.extend(prompt.stdout.rstrip(b'\n') + b'\n')


def upload_checker():
    for filename, source_path in [('check_base_autostart_ros2.py', repo/'tools/check_base_autostart_ros2.py'),
                                   ('native-observer', observer_binary)]:
        with source_path.open('rb') as source:
            subprocess.run(ssh + ['cat > ' + remote + '/' + filename], stdin=source,
                           capture_output=True, check=True, timeout=25)
    run('chmod 700 ' + remote + '/native-observer', 'native-observer-permission')


def archive_evidence(name):
    with (record/(prefix+name+'.tar.gz')).open('wb') as output:
        subprocess.run(ssh + ['tar -I "gzip -1" -cf - -C /tmp drone-base-autostart'],
                       stdout=output, check=True, timeout=40)


try:
    before = subprocess.run(ssh + ['cat /proc/sys/kernel/random/boot_id'],
                            capture_output=True, text=True, check=True, timeout=10)
    result['boot_id_before'] = before.stdout.strip()
    if args.resume_installed:
        run('cat ' + remote + '/native-initial-check.json', 'initial-check-result')
        assert json.loads((record/(prefix+'initial-check-result.log')).read_text())['passed']
    else:
        authenticate()
        run('sudo -S -p "" python3 ' + remote + '/install_services.py',
            'service-install', timeout=35, privileged=True)
        wait_for_services('initial-services-active')
        upload_checker()
        run('source /home/orangepi/drone_platform/env.sh && '
            'ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ROS_LOG_DIR=' + remote + '/ros-log '
            'python3 -u ' + remote + '/check_base_autostart_ros2.py ' + remote + '/initial-check.json',
            'initial-runtime-check', timeout=85)
        run('cat ' + remote + '/initial-check.json', 'initial-check-result')
    result['installed'] = True
    collect('initial-services-and-network')
    result['initial_check_passed'] = True
    archive_evidence('before-reboot-evidence')
    if args.reboot_after_check:
        if not secret:
            cached = subprocess.run(ssh+['sudo -n true'], capture_output=True, timeout=10)
            if cached.returncode:
                authenticate()
        guard = '''import json,time,rclpy
from mavros_msgs.msg import State
rclpy.init();node=rclpy.create_node('base_reboot_readonly_guard');state=[]
sub=node.create_subscription(State,'/mavros/state',lambda message:state.append(message),10)
deadline=time.monotonic()+6
while not state and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
assert state and state[-1].connected and not state[-1].armed
print(json.dumps({'connected':state[-1].connected,'armed':state[-1].armed,'wall_ns':time.time_ns()}))
node.destroy_node();rclpy.shutdown()
'''
        run('source /home/orangepi/drone_platform/env.sh && ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 python3 -c '
            +shlex.quote(guard), 'fresh-px4-before-reboot', timeout=15)
        print(json.dumps({'reboot': 'initial passive check passed; rebooting once'}), flush=True)
        process = run('sudo -S -p "" /sbin/shutdown -r now',
                      'reboot-request', privileged=True, check=False)
        if process.returncode not in (0, 255):
            process.check_returncode()
        result['reboot_requested'] = True
        deadline = time.monotonic() + 180
        attempt = 0
        while time.monotonic() < deadline:
            attempt += 1
            status = subprocess.run(ssh + ['cat /proc/sys/kernel/random/boot_id'],
                                    capture_output=True, text=True, timeout=12)
            if status.returncode == 0 and status.stdout.strip() != result['boot_id_before']:
                result['boot_id_after'] = status.stdout.strip()
                print(json.dumps({'new_boot_connected': result['boot_id_after']}), flush=True)
                break
            if attempt % 3 == 0:
                print(json.dumps({'waiting_for_new_boot': attempt}), flush=True)
            time.sleep(3)
        else:
            raise TimeoutError('new boot not connected within 180 seconds')
        # /tmp为tmpfs，仅重新上传只读检查器，不重新启动任何服务。
        run('mkdir ' + remote, 'create-after-reboot')
        upload_checker()
        wait_for_services('boot-services-active')
        run('source /home/orangepi/drone_platform/env.sh && '
            'ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ROS_LOG_DIR=' + remote + '/ros-log '
            'python3 -u ' + remote + '/check_base_autostart_ros2.py ' + remote + '/boot-check.json',
            'boot-runtime-check', timeout=85)
        run('cat ' + remote + '/boot-check.json', 'boot-check-result')
        collect('boot-services-and-network')
        result['boot_check_passed'] = True
        archive_evidence('after-boot-evidence')
        run('rm -rf -- '+remote, 'temporary-evidence-cleanup')
        result['temporary_evidence_cleaned'] = True
    result['passed'] = True
except BaseException as error:
    result['error'] = repr(error)
    try:
        collect('failure-services-and-network')
    except Exception as failure:
        result['collection_error'] = repr(failure)
    raise
finally:
    for index in range(len(secret)):
        secret[index] = 0
    (record / (prefix+'activation-result.json')).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)
