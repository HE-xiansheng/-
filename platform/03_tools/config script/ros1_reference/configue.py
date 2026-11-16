#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Inspect/configure Livox and client SSH trust. Python 3.8+, Linux, no pip packages.

No arming, flight-mode, setpoint, lidar-IP-write or firmware commands are sent.
Discovery command 0x0000 follows the local Livox SDK2 sdk_protocol/define headers.
Aircraft mode writes only requested configuration; client mode updates one SSH
trust record using the groundstation's saved target, never its application code.
On aircraft, rollback data stays in memory; no logs, cache, .bak or temporary
files are made. Optional pairing writes only its necessary private config.
"""
import sys
sys.dont_write_bytecode = True

import argparse
import base64
import binascii
import collections
import copy
import fcntl
import fnmatch
import glob
import hashlib
import hmac
import ipaddress
import json
import os
from pathlib import Path
import pwd
import re
import select
import shlex
import shutil
import signal
import socket
import struct
import subprocess
import tempfile
import time
import xmlrpc.client
from urllib.parse import urlparse
from xml.etree import ElementTree
import zlib


class ConfigError(Exception):
    pass


SSH_HOST_KEY_FILES = {'ssh-ed25519': 'ssh_host_ed25519_key',
                      'ecdsa-sha2-nistp256': 'ssh_host_ecdsa_key',
                      'ecdsa-sha2-nistp384': 'ssh_host_ecdsa_key',
                      'ecdsa-sha2-nistp521': 'ssh_host_ecdsa_key',
                      'ssh-rsa': 'ssh_host_rsa_key'}


def ssh_public_identity(public):
    fields = public.split()
    kind = fields[0].decode('ascii') if fields else ''
    if len(fields) < 2 or kind not in SSH_HOST_KEY_FILES:
        raise ConfigError('不支持或无效的 SSH 主机公钥格式')
    blob = base64.b64decode(fields[1], validate=True)
    if len(blob) < 8:
        raise ConfigError('SSH 主机公钥数据不完整')
    length = struct.unpack_from('>I', blob)[0]
    if blob[4:4 + length].decode('ascii') != kind or len(blob) <= 4 + length:
        raise ConfigError('SSH 主机公钥类型与内容不匹配')
    fingerprint = 'SHA256:' + base64.b64encode(hashlib.sha256(blob).digest()).decode().rstrip('=')
    return kind, fingerprint


def show_ssh_identity(key_dir=Path('/etc/ssh')):
    """Server-side audit only: never change client known_hosts or create keys."""
    result = {}
    print('SSH 主机身份：现有密钥保持不变，不因主机名或 IP 变化重新生成。', flush=True)
    for filename in sorted(set(SSH_HOST_KEY_FILES.values())):
        public = key_dir / (filename + '.pub')
        if not public.is_file():
            continue
        kind, fingerprint = ssh_public_identity(public.read_bytes())
        result[kind] = fingerprint
        print('SSH_HOST_KEY {} {}'.format(kind, fingerprint), flush=True)
    if not result:
        raise ConfigError('未找到现有 SSH 主机公钥；为避免身份变化，本脚本不自动生成新主机密钥')
    return result


def configure_server_ssh_identity(args):
    show_ssh_identity()
    if args.apply:
        if os.geteuid() != 0:
            raise ConfigError('检查 SSH 服务配置需要 sudo')
        run([shutil.which('sshd') or '/usr/sbin/sshd', '-t'])
        print('机载 SSH 配置有效，现有身份已保留；没有生成新密钥或修改电脑端记录。', flush=True)
    return 0


def valid_ssh_host(host):
    if not isinstance(host, str) or not host or host.startswith('-') or not all(c.isalnum() or c in '.-:%' for c in host):
        raise ConfigError('无效的 SSH 目标地址')
    return host


def groundstation_connection(args, home=None, script=None):
    home = Path.home() if home is None else Path(home)
    script = Path(__file__).resolve() if script is None else Path(script).resolve()
    if args.groundstation_config:
        candidates = [Path(args.groundstation_config).expanduser()]
    else:
        candidates = [parent / '.pointcloud-link/connection.json' for parent in list(script.parents)[:4]]
        candidates += [home / '.local/opt/research-flight-groundstation/.pointcloud-link/connection.json',
                       Path.cwd() / '.pointcloud-link/connection.json']
    config = {}
    source = None
    for candidate in candidates:
        if candidate.is_file():
            config = json.loads(candidate.read_text())
            if not isinstance(config, dict):
                raise ConfigError('地面站连接配置必须是 JSON 对象')
            source = candidate.resolve()
            break
    host = args.ssh_host or config.get('host') or config.get('onboard_ip')
    if not host:
        raise ConfigError('未找到地面站保存的机载地址；请在电脑端运行，或用 --groundstation-config 指定现有连接配置')
    host = valid_ssh_host(host)
    user = args.ssh_user or config.get('user') or config.get('username') or 'orangepi'
    if not isinstance(user, str) or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]*', user):
        raise ConfigError('无效的 SSH 用户名')
    port = args.ssh_port if args.ssh_port is not None else config.get('ssh_port', 22)
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise ConfigError('SSH 端口必须为 1～65535')
    return host, user, port, source


def known_host_matches(pattern, target):
    if pattern == target:
        return True
    if pattern.startswith('|1|'):
        try:
            _, version, salt, digest = pattern.split('|')
            actual = hmac.new(base64.b64decode(salt, validate=True), target.encode(), hashlib.sha1).digest()
            return version == '1' and hmac.compare_digest(actual, base64.b64decode(digest, validate=True))
        except ValueError:
            return False
    return False


def known_host_records(content, target):
    records = []
    for line in content.decode().splitlines():
        fields = line.split()
        if not fields or fields[0].startswith('#'):
            continue
        marker = fields.pop(0) if fields[0].startswith('@') else None
        if len(fields) < 3:
            continue
        names = fields[0].split(',')
        exact = any(known_host_matches(name, target) for name in names)
        wildcard = any(not name.startswith(('!', '|')) and any(c in name for c in '*?') and fnmatch.fnmatchcase(target, name) for name in names)
        if not exact and not wildcard:
            continue
        if marker:
            raise ConfigError('目标存在 SSH 证书或撤销记录，不自动覆盖：' + marker)
        if wildcard:
            raise ConfigError('目标匹配通配主机信任记录，不能仅删除单台设备；请使用独立 known_hosts 文件')
        kind, fingerprint = ssh_public_identity((' '.join(fields[1:3])).encode())
        records.append((kind, fields[2], fingerprint))
    return records


def plan_known_hosts(content, target, selected):
    existing = known_host_records(content, target)  # Reject revocation/certificate/wildcard policies.
    if len(existing) == len(selected) and {key[:2] for key in existing} == {key[:2] for key in selected}:
        return content
    output = []
    pattern = None
    for line in content.decode().splitlines(keepends=True):
        fields = line.split()
        if len(fields) < 3 or fields[0].startswith(('#', '@')):
            output.append(line)
            continue
        names = fields[0].split(',')
        hits = [name for name in names if known_host_matches(name, target)]
        if not hits:
            output.append(line)
            continue
        pattern = pattern or hits[0]
        remaining = [name for name in names if not known_host_matches(name, target)]
        if remaining:
            output.append(','.join(remaining) + ' ' + line.split(None, 1)[1])
    if pattern is None:
        salt = os.urandom(20)
        digest = hmac.new(salt, target.encode(), hashlib.sha1).digest()
        pattern = '|1|' + base64.b64encode(salt).decode() + '|' + base64.b64encode(digest).decode()
    if output and not output[-1].endswith('\n'):
        output[-1] += '\n'
    for kind, blob, _ in selected:
        output.append(pattern + ' ' + kind + ' ' + blob + '\n')
    return ''.join(output).encode()


def choose_ssh_host_keys(scanned, saved, trusted=None):
    if trusted:
        selected = [key for key in scanned if key[2] == trusted]
        if not selected:
            raise ConfigError('提供的可信指纹与目标当前主机密钥不匹配；未更新 SSH 记录')
        return selected
    selected = [key for key in scanned if any(key[:2] == old[:2] for old in saved)]
    if selected:
        return selected
    return []


def verify_pinned_ssh(host, user, port, known_hosts, key_type):
    # Host-key handshake is mandatory even if login needs a password afterwards.
    argv = ['ssh', '-v', '-T', '-p', str(port), '-l', user,
            '-o', 'BatchMode=yes', '-o', 'NumberOfPasswordPrompts=0',
            '-o', 'StrictHostKeyChecking=yes', '-o', 'UpdateHostKeys=no',
            '-o', 'UserKnownHostsFile=' + str(known_hosts),
            '-o', 'HostKeyAlgorithms=' + ('rsa-sha2-512,rsa-sha2-256' if key_type == 'ssh-rsa' else key_type),
            '-o', 'ConnectTimeout=6', host, 'hostname']
    result = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=15)
    error = result.stderr.decode(errors='replace')
    if result.returncode == 0:
        return True, result.stdout.decode(errors='replace').strip()
    proved = re.search(r"(?m)^debug1: Host '[^\n]+' is known and matches the [A-Z0-9_-]+ host key\.\r?$", error)
    if (proved and result.returncode == 255 and 'Permission denied' in error
            and 'Host key verification failed' not in error):
        return False, '主机指纹校验通过，但用户登录需要密码或授权密钥'
    raise ConfigError('SSH 身份握手失败，未更新旧记录：' + error.strip()[-800:])


def configure_groundstation_ssh(args):
    if os.geteuid() == 0:
        raise ConfigError('电脑端 SSH 配置请以当前桌面用户运行，不要加 sudo')
    host, user, port, config = groundstation_connection(args)
    effective = run(['ssh', '-G', '-p', str(port), '-l', user, host])
    settings = {}
    for line in effective.splitlines():
        fields = line.split(None, 1)
        if len(fields) == 2:
            settings[fields[0]] = fields[1]
    remote = valid_ssh_host(settings.get('hostname', host))
    alias = settings.get('hostkeyalias')
    target = alias if alias and alias != 'none' else (remote if port == 22 else '[{}]:{}'.format(remote, port))
    known = (Path(args.known_hosts).expanduser() if args.known_hosts else Path(shlex.split(settings.get('userknownhostsfile', str(Path.home() / '.ssh/known_hosts')))[0]).expanduser()).resolve()
    if str(known) == '/dev/null':
        raise ConfigError('不能把永久 SSH 身份记录保存到 /dev/null')
    local = {a['local'] for item in json.loads(run(['ip', '-j', 'address', 'show']))
             for a in item.get('addr_info', []) if a.get('family') in ('inet', 'inet6')}
    resolved = {item[4][0].split('%')[0] for item in socket.getaddrinfo(remote, port, type=socket.SOCK_STREAM)}
    if resolved & local or any(ipaddress.ip_address(value).is_loopback for value in resolved):
        raise ConfigError('目标就是本机；此模式必须在运行地面站的电脑执行，不能在机载端更新电脑的 SSH 记录')
    print('读取连接配置：' + str(config or '显式地址参数'), flush=True)
    print('自动读取 SSH 目标：{}@{}，端口 {}；身份文件 {}'.format(user, host, port, known), flush=True)
    scan = subprocess.run(['ssh-keyscan', '-T', '5', '-p', str(port), '-t', 'ed25519,ecdsa,rsa', remote],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=15)
    scanned = []
    for line in scan.stdout.decode().splitlines():
        fields = line.split()
        if len(fields) < 3 or fields[0].startswith('#'):
            continue
        kind, fingerprint = ssh_public_identity((' '.join(fields[1:3])).encode())
        key = (kind, fields[2], fingerprint)
        if key not in scanned:
            scanned.append(key)
    if not scanned:
        raise ConfigError('未读到目标的 SSH 公钥；请检查当前连接地址及网络，未修改记录')
    before = known.read_bytes() if known.is_file() else b''
    saved = known_host_records(before, target)
    print('旧指纹：' + str([(key[0], key[2]) for key in saved]), flush=True)
    print('设备当前指纹：' + str([(key[0], key[2]) for key in scanned]), flush=True)
    selected = choose_ssh_host_keys(scanned, saved, args.trust_fingerprint)
    if not args.apply:
        print('只检查结束：{}。'.format('已有可信密钥匹配' if selected else '新设备或身份变化，应用时需要核对指纹'), flush=True)
        return 0
    if not selected:
        if not sys.stdin.isatty():
            raise ConfigError('身份已变化；请在交互终端核对机载端指纹，或传入 --trust-fingerprint SHA256:...；未更新旧记录')
        print('请与机载端 --ssh-identity --check 显示的指纹核对。', flush=True)
        fingerprint = input('输入已核对的完整 SHA256 指纹以更新（直接回车取消）：').strip()
        selected = choose_ssh_host_keys(scanned, saved, fingerprint) if fingerprint else []
        if not selected:
            raise ConfigError('未确认可信设备，旧 SSH 记录保持不变')
    # Verify proof of possession through SSH before changing the real trust file.
    # Never trust every keyscan result just because one public fingerprint matched.
    with tempfile.TemporaryDirectory(prefix='gcs-ssh-verify-') as folder:
        candidate = Path(folder) / 'known_hosts'
        candidate.write_text(target + ' ' + selected[0][0] + ' ' + selected[0][1] + '\n')
        os.chmod(candidate, 0o600)
        authenticated, detail = verify_pinned_ssh(host, user, port, candidate, selected[0][0])
    after = plan_known_hosts(before, target, selected)
    if (known.read_bytes() if known.is_file() else b'') != before:
        raise ConfigError('验证期间 SSH 记录被其他进程修改，请重新运行；未覆盖其他修改')
    if after != before:
        known.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        descriptor, temporary = tempfile.mkstemp(prefix='.known_hosts.configure.', dir=known.parent)
        try:
            os.fchmod(descriptor, known.stat().st_mode & 0o777 if known.exists() else 0o600)
            with os.fdopen(descriptor, 'wb') as stream:
                stream.write(after)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, known)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
    # Verify the actual hashed entry too, not only the temporary un-hashed pin.
    authenticated, detail = verify_pinned_ssh(host, user, port, known, selected[0][0])
    print(('SSH 身份已匹配，无需改动。' if after == before else 'SSH 身份记录已自动更新，仅涉及当前目标。') + '地面站程序和机载配置均未修改。', flush=True)
    print(('SSH 登录验证通过：' if authenticated else 'SSH 身份已配置：') + detail, flush=True)
    return 0


SSH_HELPER_PORT = 48136
SSH_HELPER_DIR = '.config/livox-ssh-helper'


def owner_home():
    return Path(pwd.getpwnam(os.environ.get('SUDO_USER') or pwd.getpwuid(os.getuid()).pw_name).pw_dir)


def helper_document(value, secret):
    body = dict(value)
    body.pop('mac', None)
    raw = json.dumps(body, sort_keys=True, separators=(',', ':'), ensure_ascii=True).encode()
    body['mac'] = hmac.new(bytes.fromhex(secret), raw, hashlib.sha256).hexdigest()
    return json.dumps(body, sort_keys=True, separators=(',', ':')).encode()


def helper_verify(raw, pairing):
    if len(raw) > 16384:
        raise ConfigError('配对消息过长')
    value = json.loads(raw)
    if not isinstance(value, dict) or value.get('v') != 1 or value.get('pair') != pairing['pair']:
        raise ConfigError('配对身份不匹配')
    signature = value.get('mac', '')
    expected = json.loads(helper_document(value, pairing['secret']))['mac']
    if not isinstance(signature, str) or not hmac.compare_digest(signature, expected):
        raise ConfigError('配对消息签名不匹配')
    if not isinstance(value.get('nonce'), str) or not re.fullmatch(r'[0-9a-f]{32}', value['nonce']):
        raise ConfigError('配对随机数无效')
    return value


def read_helper_config(path):
    if path.stat().st_mode & 0o077:
        raise ConfigError('配对配置权限必须为 600：' + str(path))
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ConfigError('配对配置必须为 JSON 对象：' + str(path))
    if (not isinstance(value.get('pair'), str) or not isinstance(value.get('secret'), str) or
            not re.fullmatch(r'[0-9a-f]{32}', value.get('pair', '')) or
            not re.fullmatch(r'[0-9a-f]{64}', value.get('secret', '')) or
            type(value.get('port')) is not int or not 1024 <= value['port'] <= 65535):
        raise ConfigError('配对配置无效：' + str(path))
    return value


def write_private_config(path, value):
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'w') as stream:
        os.fchmod(stream.fileno(), 0o600)
        json.dump(value, stream, indent=2)
        stream.write('\n')


def helper_update_known_hosts(config, peer, request):
    # The source address is observed by the receiver, never supplied by a request.
    host = ipv4(peer)
    keys = request.get('keys')
    if not isinstance(keys, list) or not 1 <= len(keys) <= 6:
        raise ConfigError('机载公钥列表无效')
    selected = []
    for public in keys:
        if not isinstance(public, str) or len(public) > 4096 or '\n' in public:
            raise ConfigError('机载公钥格式无效')
        kind, fingerprint = ssh_public_identity(public.encode())
        key = (kind, public.split()[1], fingerprint)
        if key not in selected:
            selected.append(key)
    selected.sort(key=lambda key: (key[0] != 'ssh-ed25519', key[0] == 'ssh-rsa'))
    # Retain only the key whose private-key possession is actually proved below.
    selected = selected[:1]
    port, user = config['ssh_port'], config['user']
    target = host if port == 22 else '[{}]:{}'.format(host, port)
    known = Path(config['known_hosts']).expanduser().resolve()
    # Match actual SSH settings; do not silently write a file the client ignores.
    effective = dict(line.split(None, 1) for line in run(['ssh', '-G', '-p', str(port), '-l', user, host]).splitlines() if ' ' in line)
    alias = effective.get('hostkeyalias', 'none')
    if alias != 'none':
        target = alias
    actual_known = Path(shlex.split(effective['userknownhostsfile'])[0]).expanduser().resolve()
    if actual_known != known or effective.get('hostname', host) != host:
        raise ConfigError('该地址的 SSH 配置使用其他身份文件或 HostName；请重新核对配对设置')
    before = known.read_bytes() if known.is_file() else b''
    # Refuse revoked/certificate/wildcard policies before contacting SSH.
    after = plan_known_hosts(before, target, selected)
    with tempfile.TemporaryDirectory(prefix='gcs-paired-verify-') as folder:
        candidate = Path(folder) / 'known_hosts'
        candidate.write_text(target + ' ' + selected[0][0] + ' ' + selected[0][1] + '\n')
        os.chmod(candidate, 0o600)
        authenticated, detail = verify_pinned_ssh(host, user, port, candidate, selected[0][0])
    if (known.read_bytes() if known.is_file() else b'') != before:
        raise ConfigError('SSH 记录在验证期间被修改，请重试')
    if before != after:
        known.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        fd, temporary = tempfile.mkstemp(prefix='.known_hosts.paired.', dir=known.parent)
        try:
            with os.fdopen(fd, 'wb') as stream:
                os.fchmod(stream.fileno(), known.stat().st_mode & 0o777 if known.exists() else 0o600)
                stream.write(after)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, known)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
    return {'host': host, 'changed': before != after, 'login': authenticated,
            'message': detail, 'fingerprints': [key[2] for key in selected]}


class SSHHelperReceiver:
    """Authenticated, single-purpose UDP receiver. No remote shell or eval."""
    def __init__(self, config):
        self.config = config
        self.pending = {}

    def handle(self, raw, peer):
        request = helper_verify(raw, self.config)
        now = time.monotonic()
        self.pending = {key: value for key, value in self.pending.items() if now - value['created'] < 90}
        key = (peer, request['nonce'])
        base = {'v': 1, 'pair': self.config['pair'], 'nonce': request['nonce']}
        if request.get('action') == 'discover':
            if key not in self.pending:
                if len(self.pending) >= 64:
                    raise ConfigError('待处理配对请求过多')
                self.pending[key] = {'created': now, 'challenge': os.urandom(16).hex()}
            base.update(action='ready', challenge=self.pending[key]['challenge'])
        elif request.get('action') == 'update':
            state = self.pending.get(key)
            if not state or request.get('challenge') != state['challenge']:
                raise ConfigError('配对请求已过期，请重新发现电脑')
            if 'result' not in state:
                try:
                    state['result'] = dict(ok=True, **helper_update_known_hosts(self.config, peer, request))
                except (ConfigError, OSError, ValueError, subprocess.SubprocessError) as error:
                    state['result'] = {'ok': False, 'message': str(error)[-1000:]}
                print('配对 SSH 更新 {}：{}'.format(peer, '成功' if state['result']['ok'] else state['result']['message']), flush=True)
            base.update(action='result', **state['result'])
        else:
            raise ConfigError('不支持的配对操作')
        return helper_document(base, self.config['secret'])


def serve_ssh_helper():
    if os.geteuid() == 0:
        raise ConfigError('电脑辅助服务必须以桌面用户运行，不要使用 sudo')
    config = read_helper_config(Path.home() / SSH_HELPER_DIR / 'receiver.json')
    receiver = SSHHelperReceiver(config)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as stream:
        stream.bind(('0.0.0.0', config['port']))
        print('配对 SSH 辅助服务已启动，UDP {}'.format(config['port']), flush=True)
        while True:
            raw, peer = stream.recvfrom(16385)
            try:
                reply = receiver.handle(raw, peer[0])
                stream.sendto(reply, peer)
            except (ConfigError, OSError, ValueError, KeyError, TypeError, RecursionError):
                pass  # Unpaired traffic cannot change files or obtain a response.


def helper_discovery_addresses():
    addresses = {'255.255.255.255'}
    for interface in json.loads(run(['ip', '-j', '-4', 'addr', 'show', 'up'])):
        for address in interface.get('addr_info', []):
            if address.get('family') == 'inet' and address.get('scope') == 'global':
                addresses.add(str(ipaddress.IPv4Network('{}/{}'.format(address['local'], address['prefixlen']), strict=False).broadcast_address))
    return sorted(addresses)


def sync_computer_ssh(apply=False):
    path = owner_home() / SSH_HELPER_DIR / 'pairing.json'
    if not path.is_file():
        raise ConfigError('尚未与电脑配对；电脑端先用同一脚本 --install-ssh-helper --apply 完成一次安装')
    config = read_helper_config(path)
    nonce = os.urandom(16).hex()
    base = {'v': 1, 'pair': config['pair'], 'nonce': nonce}
    discover = helper_document(dict(base, action='discover'), config['secret'])
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as stream:
        stream.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        stream.bind(('0.0.0.0', 0))
        found = {}
        for attempt in range(3):
            for address in helper_discovery_addresses():
                try:
                    stream.sendto(discover, (address, config['port']))
                except OSError:
                    continue
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                stream.settimeout(max(0.01, end - time.monotonic()))
                try:
                    raw, peer = stream.recvfrom(16385)
                    reply = helper_verify(raw, config)
                    if reply.get('action') == 'ready' and reply['nonce'] == nonce and peer[1] == config['port']:
                        found[peer] = reply
                except socket.timeout:
                    break
                except (ConfigError, ValueError, OSError, KeyError, TypeError):
                    continue
            if found:
                break
        if len(found) != 1:
            raise ConfigError('未找到唯一的已配对电脑；请确认电脑已登录、辅助服务运行，且两端在同一局域网（UDP {}）'.format(config['port']))
        peer, reply = next(iter(found.items()))
        print('自动发现已配对电脑：' + peer[0], flush=True)
        if not apply:
            print('只检查：配对链路正常，未更新电脑 SSH 记录。', flush=True)
            return 0
        keys = []
        for name in sorted(set(SSH_HOST_KEY_FILES.values())):
            public = Path('/etc/ssh') / (name + '.pub')
            if public.is_file():
                fields = public.read_text().split()
                ssh_public_identity(public.read_bytes())
                keys.append(' '.join(fields[:2]))
        if not keys:
            raise ConfigError('机载端没有现有 SSH 公钥；不自动生成新身份')
        request = helper_document(dict(base, action='update', challenge=reply['challenge'], keys=keys), config['secret'])
        for attempt in range(2):
            stream.sendto(request, peer)
            end = time.monotonic() + 25
            while time.monotonic() < end:
                stream.settimeout(max(0.01, end - time.monotonic()))
                try:
                    raw, source = stream.recvfrom(16385)
                    result = helper_verify(raw, config)
                except socket.timeout:
                    break
                except (ConfigError, ValueError, OSError, KeyError, TypeError):
                    continue
                if source != peer or result['nonce'] != nonce or result.get('action') != 'result':
                    continue
                if not result.get('ok'):
                    raise ConfigError('电脑 SSH 更新失败：' + result.get('message', '未知错误'))
                print('电脑已自动识别本机地址：' + result['host'], flush=True)
                print('电脑 SSH 记录：' + ('已更新。' if result['changed'] else '已匹配，无需修改。'), flush=True)
                print(('SSH 登录验证通过：' if result['login'] else 'SSH 身份验证通过；用户登录仍需密码：') + result['message'], flush=True)
                return 0
        raise ConfigError('电脑未返回更新结果；请检查辅助服务，重复执行安全且不会删除其他设备记录')


def install_ssh_helper(args):
    if not args.apply or os.geteuid() == 0:
        raise ConfigError('首次配对请在电脑端以桌面用户运行 --install-ssh-helper --apply，不要加 sudo')
    # Pair only over an independently verified, strict SSH connection.
    configure_groundstation_ssh(args)
    host, user, port, _ = groundstation_connection(args)
    effective = dict(line.split(None, 1) for line in run(['ssh', '-G', '-p', str(port), '-l', user, host]).splitlines() if ' ' in line)
    known = (Path(args.known_hosts).expanduser() if args.known_hosts else Path(shlex.split(effective['userknownhostsfile'])[0]).expanduser()).resolve()
    config_path = Path.home() / SSH_HELPER_DIR / 'receiver.json'
    config = read_helper_config(config_path) if config_path.exists() else {
        'pair': os.urandom(16).hex(), 'secret': os.urandom(32).hex(), 'port': SSH_HELPER_PORT}
    config.update(user=user, ssh_port=port, known_hosts=str(known))
    pairing = {key: config[key] for key in ('pair', 'secret', 'port')}
    # This is the only aircraft-side persistent addition: necessary pairing
    # configuration. No cache, backup, host private key or remote-control key.
    provision = '''import json,os,sys
from pathlib import Path
data=json.load(sys.stdin)
path=Path.home()/'.config/livox-ssh-helper/pairing.json'
if path.exists() and json.loads(path.read_text()).get('pair')!=data['pair']:
    raise SystemExit('Device already paired with another computer; explicitly remove old pairing first')
path.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
fd=os.open(str(path),os.O_WRONLY|os.O_CREAT|os.O_TRUNC|os.O_NOFOLLOW,0o600)
with os.fdopen(fd,'w') as stream:
    os.fchmod(stream.fileno(),0o600)
    json.dump(data,stream,indent=2)
    stream.write('\\n')
print('Aircraft pairing saved; no host private keys changed')
'''
    run(['ssh', '-T', '-o', 'StrictHostKeyChecking=yes', '-o', 'UpdateHostKeys=no', '-p', str(port), '-l', user,
         host, 'python3 -B -c ' + shlex.quote(provision)], data=json.dumps(pairing).encode(), timeout=30)
    write_private_config(config_path, config)
    installed = Path.home() / '.local/lib/livox-ssh-helper/configure_livox.py'
    installed.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if Path(__file__).resolve() != installed:
        installed.write_bytes(Path(__file__).read_bytes())
    os.chmod(installed, 0o700)
    unit = Path.home() / '.config/systemd/user/livox-ssh-helper.service'
    unit.parent.mkdir(parents=True, exist_ok=True)
    escaped = str(installed).replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%')
    unit.write_text('[Unit]\nDescription=Paired aircraft SSH identity helper\n\n[Service]\n'
                    'ExecStart=/usr/bin/python3 -B "' + escaped + '" --serve-ssh-helper\n'
                    'Restart=on-failure\nRestartSec=3\nUMask=0077\nNoNewPrivileges=yes\n'
                    '\n[Install]\nWantedBy=default.target\n')
    run(['systemctl', '--user', 'daemon-reload'])
    run(['systemctl', '--user', 'enable', 'livox-ssh-helper.service'])
    run(['systemctl', '--user', 'restart', 'livox-ssh-helper.service'])
    run(['systemctl', '--user', 'is-active', 'livox-ssh-helper.service'])
    print('首次配对已完成，电脑辅助服务随桌面用户登录启动。以后只需在香橙派执行脚本 --ssh-sync --apply；正常雷达 --apply 也会自动同步。', flush=True)
    return 0


def run(argv, *, binary=False, data=None, stdin=None, timeout=15):
    p = subprocess.run(argv, input=data, stdin=stdin, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, timeout=timeout)
    if p.returncode:
        raise ConfigError('{}: {}'.format(' '.join(argv), p.stderr.decode(errors='replace').strip()))
    return p.stdout if binary else p.stdout.decode().strip()


def ipv4(value):
    address = ipaddress.IPv4Address(value)
    if address.is_unspecified or address.is_multicast or address.is_loopback or int(address) == 0xffffffff:
        raise ConfigError('无效的设备/主机 IPv4 地址：' + value)
    return str(address)


def interfaces():
    result = {}
    for item in json.loads(run(['ip', '-j', '-4', 'address', 'show'])):
        name = item['ifname']
        if name == 'lo' or 'UP' not in item.get('flags', []):
            continue
        if Path('/sys/class/net', name, 'wireless').exists():
            continue
        # Exclude bridges, tunnels, veth and other non-physical links.
        if not Path('/sys/class/net', name, 'device').exists():
            continue
        addrs = [a['local'] for a in item.get('addr_info', [])
                 if a['family'] == 'inet' and a.get('scope') == 'global']
        if addrs:
            result[name] = addrs
    return result


def discovery_packet():
    # SDK2: 18-byte header, CRC16-CCITT-FALSE, then zero CRC32 for empty data.
    header = struct.pack('<BBHIHBB6s', 0xaa, 0, 24, 1, 0, 0, 0, b'\0' * 6)
    return header + struct.pack('<HI', binascii.crc_hqx(header, 0xffff), 0)


def discovery_reply(payload):
    if len(payload) < 48 or payload[:2] != b'\xaa\0':
        return None
    length = struct.unpack_from('<H', payload, 2)[0]
    if length != len(payload) or struct.unpack_from('<H', payload, 8)[0] != 0:
        return None
    if payload[10:12] != b'\x01\x01':
        return None
    if struct.unpack_from('<H', payload, 18)[0] != binascii.crc_hqx(payload[:18], 0xffff):
        return None
    data = payload[24:]
    if struct.unpack_from('<I', payload, 20)[0] != zlib.crc32(data) & 0xffffffff:
        return None
    if data[0] != 0:
        return None
    return {'type': data[1], 'serial': data[2:18].split(b'\0')[0].decode('ascii', errors='replace'),
            'ip': socket.inet_ntoa(data[18:22])}


def udp_frame(frame):
    if len(frame) < 14:
        return None
    ethertype = struct.unpack_from('!H', frame, 12)[0]
    offset = 14
    for _ in range(2):
        if ethertype not in (0x8100, 0x88a8):
            break
        if len(frame) < offset + 4:
            return None
        ethertype = struct.unpack_from('!H', frame, offset + 2)[0]
        offset += 4
    if ethertype != 0x0800 or len(frame) < offset + 20:
        return None
    header_size = (frame[offset] & 15) * 4
    total = struct.unpack_from('!H', frame, offset + 2)[0]
    frag = struct.unpack_from('!H', frame, offset + 6)[0]
    if frame[offset] >> 4 != 4 or header_size < 20 or frame[offset + 9] != 17 or frag & 0x3fff:
        return None
    if total < header_size + 8 or len(frame) < offset + total:
        return None
    start = offset + header_size
    sport, dport, length = struct.unpack_from('!HHH', frame, start)
    if length < 8 or length != total - header_size:
        return None
    return (socket.inet_ntoa(frame[offset + 12:offset + 16]),
            socket.inet_ntoa(frame[offset + 16:offset + 20]), sport, dport,
            frame[start + 8:start + length])


def is_livox_data(payload):
    # LivoxLidarEthernetPacket: 36-byte fixed part followed by points/IMU.
    return (len(payload) >= 36 and payload[0] == 0 and
            struct.unpack_from('<H', payload, 1)[0] == len(payload) and
            payload[10] in (1, 2, 3, 0))


def discover(ifaces, seconds):
    observations = {}
    probe_sockets = []
    receiver = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0003))
    try:
        for name, addrs in ifaces.items():
            for address in addrs:
                sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                try:
                    sender.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                    sender.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, name.encode() + b'\0')
                    sender.bind((address, 0))
                    probe_sockets.append(sender)
                except OSError:
                    sender.close()
        end = time.monotonic() + seconds
        next_probe = 0
        while time.monotonic() < end:
            if time.monotonic() >= next_probe:
                for sender in probe_sockets:
                    try:
                        sender.sendto(discovery_packet(), ('255.255.255.255', 56000))
                    except OSError:
                        pass
                next_probe = time.monotonic() + 1
            ready, _, _ = select.select([receiver], [], [], min(.2, max(0, end - time.monotonic())))
            if not ready:
                continue
            frame, metadata = receiver.recvfrom(65535)
            name = metadata[0]
            if name not in ifaces or metadata[2] == socket.PACKET_OUTGOING:
                continue
            parsed = udp_frame(frame)
            if parsed is None:
                continue
            source, target, sport, dport, payload = parsed
            reply = discovery_reply(payload) if sport == 56000 else None
            stream = sport in (56300, 56400) and is_livox_data(payload)
            if not reply and not stream:
                continue
            if source in ifaces[name] or (reply and reply['ip'] != source):
                continue
            observation = observations.setdefault((name, source), {
                'interface': name, 'ip': source, 'targets': set(), 'ports': collections.Counter(),
                'type': None, 'serial': None})
            if stream:
                observation['ports'][sport] += 1
                observation['targets'].add(target)
            if reply:
                observation.update(type=reply['type'], serial=reply['serial'])
    finally:
        receiver.close()
        for sender in probe_sockets:
            sender.close()
    return list(observations.values())


def choose_device(observations, wanted_ip=None):
    candidates = [o for o in observations if
                  (o['type'] in (9, 35) or (o['ports'][56300] >= 2 and o['ports'][56400] >= 2))
                  and (wanted_ip is None or o['ip'] == wanted_ip)]
    if len(candidates) != 1:
        raise ConfigError('需唯一识别一台 MID360/MID360s，当前候选 {}。请检查供电/网线，或用 --interface 和 --lidar-ip 指定；不猜测地址。'.format(
            [(c['interface'], c['ip']) for c in candidates]))
    return candidates[0]


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3
        return connection


def proxy(uri):
    return xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport(), allow_none=True)


def ros_master(uri):
    master = proxy(uri)
    code, message, graph = master.getSystemState('/livox_network_config')
    if code != 1:
        raise ConfigError('ROS master: ' + message)
    return master, dict(graph[0])


def read_exact(sock, size):
    if size > 8 * 1024 * 1024:
        raise ConfigError('ROS 消息过大')
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConfigError('ROS TCP 连接中断')
        chunks.extend(chunk)
    return bytes(chunks)


def ros_message(master, publishers, topic, timeout=3):
    nodes = publishers.get(topic, [])
    if len(nodes) != 1:
        raise ConfigError('{} 需要唯一发布者，实际 {}'.format(topic, nodes))
    code, _, uri = master.lookupNode('/livox_network_config', nodes[0])
    if code != 1:
        raise ConfigError('找不到 ROS 发布节点：' + nodes[0])
    code, message, protocol = proxy(uri).requestTopic('/livox_network_config', topic, [['TCPROS']])
    if code != 1 or protocol[0] != 'TCPROS':
        raise ConfigError('ROS 订阅失败：' + str(message))
    fields = ['callerid=/livox_network_config', 'topic=' + topic, 'md5sum=*', 'tcp_nodelay=1']
    header = b''.join(struct.pack('<I', len(x.encode())) + x.encode() for x in fields)
    with socket.create_connection((protocol[1], protocol[2]), timeout=timeout) as sock:
        sock.sendall(struct.pack('<I', len(header)) + header)
        read_exact(sock, struct.unpack('<I', read_exact(sock, 4))[0])
        return read_exact(sock, struct.unpack('<I', read_exact(sock, 4))[0])


def ros_header(data):
    if len(data) < 16:
        raise ConfigError('ROS 消息 Header 不完整')
    _, seconds, nanos, frame_len = struct.unpack_from('<IIII', data)
    offset = 16 + frame_len
    if len(data) < offset:
        raise ConfigError('ROS frame_id 不完整')
    return seconds + nanos / 1e9, offset


def flight_gate(uri):
    # Read via TCPROS without rospy: no ROS node registration or log/cache files.
    master, publishers = ros_master(uri)
    commands = ['/mavros/setpoint_raw/local', '/mavros/setpoint_raw/attitude',
                '/mavros/setpoint_raw/global', '/mavros/setpoint_position/local',
                '/mavros/setpoint_velocity/cmd_vel', '/mavros/setpoint_velocity/cmd_vel_unstamped',
                '/mavros/setpoint_attitude/attitude', '/mavros/setpoint_attitude/thrust',
                '/mavros/actuator_control', '/mavros/rc/override']
    active = {t: publishers[t] for t in commands if publishers.get(t)}
    if active:
        raise ConfigError('请先停止飞行任务，存在控制指令发布者：' + str(active))
    data = ros_message(master, publishers, '/mavros/state')
    stamp, offset = ros_header(data)
    if not -.5 <= time.time() - stamp <= 3 or len(data) < offset + 8:
        raise ConfigError('飞控状态无效或不新鲜，拒绝修改')
    connected, armed = struct.unpack_from('<??', data, offset)
    length = struct.unpack_from('<I', data, offset + 4)[0]
    mode = data[offset + 8:offset + 8 + length].decode()
    if not connected or armed or mode == 'OFFBOARD':
        raise ConfigError('要求飞控已连接、未解锁且不在 OFFBOARD；当前 connected={} armed={} mode={}'.format(connected, armed, mode))
    landed = ros_message(master, publishers, '/mavros/extended_state')
    stamp, offset = ros_header(landed)
    if not -.5 <= time.time() - stamp <= 3 or len(landed) < offset + 2 or landed[offset + 1] != 1:
        raise ConfigError('需要新鲜的在地面状态，拒绝修改')
    print('飞控检查：已连接、未解锁、在地面，模式 ' + mode, flush=True)


def find_config(argument, uri, home, wait_seconds=0):
    if argument:
        path = Path(argument).expanduser().resolve(strict=True)
    else:
        path = None
        deadline = time.monotonic() + wait_seconds
        next_notice = 0
        while path is None:
            try:
                code, _, value = proxy(uri).getParam('/livox_network_config', '/user_config_path')
                if code == 1:
                    candidate = Path(value)
                    if candidate.is_file():
                        path = candidate.resolve()
                        break
            except (OSError, xmlrpc.client.Error):
                pass
            root = home / 'drone_platform/02_drivers/sensors/share/livox_ros_driver2/config'
            choices = [p for p in root.glob('*config.json') if p.name.lower().startswith('mid360')]
            if len(choices) == 1:
                path = choices[0].resolve()
                break
            if time.monotonic() >= deadline:
                raise ConfigError('驱动尚未提供实际配置路径且存在多个模板；请等待传感器服务启动，或用 --config 指定实际 JSON')
            if time.monotonic() >= next_notice:
                print('等待传感器服务提供实际雷达配置路径，未修改任何配置……', flush=True)
                next_notice = time.monotonic() + 10
            time.sleep(.5)
    return path, path.read_bytes()


def plan_config(original, device, host, entry_ip=None):
    config = json.loads(original)
    entries = config.get('lidar_configs', [])
    if not isinstance(entries, list) or not entries:
        raise ConfigError('配置必须含非空 lidar_configs 数组')
    if entry_ip:
        selected = [i for i, e in enumerate(entries) if e.get('ip') == entry_ip]
    elif len(entries) == 1:
        selected = [0]
    else:
        selected = [i for i, e in enumerate(entries) if e.get('ip') == device['ip']]
    if len(selected) != 1:
        raise ConfigError('多雷达配置需要 --entry-ip 指明要替换的原设备 IP')
    keys = [k for k, v in config.items() if k.lower() in ('mid360', 'mid360s') and isinstance(v, dict)]
    if len(keys) != 1:
        raise ConfigError('仅支持单个 MID360 或 MID360s 配置段，不自动修改混合型号配置')
    model = keys[0]
    expected_type = 35 if model.lower() == 'mid360s' else 9
    if device['type'] is not None and device['type'] != expected_type:
        raise ConfigError('检测到型号与 {} 配置不一致；请先选择该型号驱动和配置模板'.format(model))
    changed = copy.deepcopy(config)
    changes = []
    selected = selected[0]

    def update(mapping, key, value, label):
        if mapping.get(key) != value:
            changes.append('{}: {} -> {}'.format(label, mapping.get(key), value))
            mapping[key] = value

    update(changed['lidar_configs'][selected], 'ip', device['ip'], 'lidar_configs[{}].ip'.format(selected))
    network = changed[model].get('host_net_info')
    if isinstance(network, list):
        if len(network) != 1 or not isinstance(network[0], dict) or 'host_ip' not in network[0]:
            raise ConfigError('多主机或不支持的 host_net_info，拒绝猜测')
        update(network[0], 'host_ip', host, model + '.host_net_info[0].host_ip')
    elif isinstance(network, dict) and 'host_ip' in network:
        update(network, 'host_ip', host, model + '.host_net_info.host_ip')
    elif isinstance(network, dict) and 'cmd_data_ip' in network:
        for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
            if key not in network:
                raise ConfigError('host_net_info 缺少字段 ' + key)
            update(network, key, host, model + '.host_net_info.' + key)
        if network.get('log_data_ip'):
            update(network, 'log_data_ip', host, model + '.host_net_info.log_data_ip')
    else:
        raise ConfigError('不支持的 host_net_info 格式')
    if len(entries) > 1 and changed[model]['host_net_info'] != config[model]['host_net_info']:
        raise ConfigError('修改共享主机地址会影响其他雷达，拒绝自动修改')
    encoded = (json.dumps(changed, ensure_ascii=False, indent=2) + '\n').encode()
    return encoded if changes else original, changes


def host_address(device, ifaces, argument):
    addrs = ifaces[device['interface']]
    if argument:
        host = ipv4(argument)
        if host not in addrs:
            raise ConfigError('--host-ip 必须是所选有线网口已有的 IPv4；脚本不会自动改网口地址')
        return host
    targets = set(addrs) & device['targets']
    if len(targets) == 1:
        return targets.pop()
    if len(addrs) == 1:
        return addrs[0]
    raise ConfigError('所选网口有多个主机地址，请用 --host-ip 明确指定')


def nm_profile(interface):
    if not shutil.which('nmcli'):
        raise ConfigError('持久路由需要 NetworkManager/nmcli；其他网络管理器请使用 --runtime-only')
    uuid = run(['nmcli', '-g', 'GENERAL.CON-UUID', 'device', 'show', interface])
    if not uuid or uuid == '--':
        raise ConfigError('网口没有活动的 NetworkManager 配置；可用 --runtime-only 仅配置本次运行')
    routes = run(['nmcli', '-g', 'ipv4.routes', 'connection', 'show', uuid])
    return uuid, routes


def sensor_ros_hostname(uri):
    """Use the local sensor node's advertised address, not a fixed board IP."""
    master, publishers = ros_master(uri)
    nodes = publishers.get('/livox/lidar') or publishers.get('/livox/imu') or publishers.get('/mavros/state')
    if not nodes or len(nodes) != 1:
        raise ConfigError('无法唯一确认本机传感器 ROS 通告地址，未修改定位启动环境')
    code, message, address = master.lookupNode('/livox_network_config', nodes[0])
    if code != 1:
        raise ConfigError('无法读取传感器节点地址：' + str(message))
    host = urlparse(address).hostname
    if host in ('localhost', '127.0.0.1'):
        host = socket.gethostname()
    if not host or not re.fullmatch(r'[A-Za-z0-9_.-]+', host):
        raise ConfigError('不支持的 ROS 主机名：' + str(host))
    own = {a['local'] for link in json.loads(run(['ip', '-j', '-4', 'address', 'show']))
           for a in link.get('addr_info', []) if a['family'] == 'inet'}
    resolved = {item[4][0] for item in socket.getaddrinfo(host, None, socket.AF_INET)}
    if not resolved or any(address not in own and not ipaddress.IPv4Address(address).is_loopback for address in resolved):
        raise ConfigError('传感器通告主机名不能确认属于本机：{} -> {}'.format(host, sorted(resolved)))
    return host


def plan_ros_launcher(path, host):
    path = Path(path).resolve()
    if not path.is_file():
        return None
    raw = path.read_bytes()
    text = raw.decode('utf-8')
    marker = re.compile(r'^# BEGIN configure_livox ROS environment\n.*?^# END configure_livox ROS environment\n?', re.M | re.S)
    if ('BEGIN configure_livox ROS' in text or 'END configure_livox ROS' in text) and len(marker.findall(text)) != 1:
        raise ConfigError('定位启动文件的受管 ROS 环境标记不完整或重复：' + str(path))
    block = ('# BEGIN configure_livox ROS environment\n'
             'export ROS_MASTER_URI=http://127.0.0.1:11311\n'
             'export ROS_HOSTNAME=' + shlex.quote(host) + '\n'
             'export PYTHONDONTWRITEBYTECODE=1\n'
             'unset ROS_IP\n'
             '# END configure_livox ROS environment\n')
    if marker.search(text):
        updated = marker.sub(lambda _: block, text)
    else:
        anchors = list(re.finditer(r'^exec\b[^\n]*manual_session\.py[^\n]*$', text, re.M))
        if len(anchors) != 1:
            raise ConfigError('无法唯一定位 manual_session.py 的执行行，拒绝猜测启动脚本：' + str(path))
        pos = anchors[0].start()
        updated = text[:pos] + block + text[pos:]
    encoded = updated.encode()
    run(['bash', '-n'], data=encoded)
    return (path, raw, encoded, '定位启动 ROS_HOSTNAME={}，在 exec 前清除 ROS_IP'.format(host))


def plan_driver_serial(config_path, device, explicit=None):
    serial = device.get('serial')
    if not serial:
        print('未收到有效 SN 发现响应，保留 launch 中的 SN；不根据 IP 猜测 SN。', flush=True)
        return None
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,16}', serial):
        raise ConfigError('发现响应中的 SN 格式无效')
    if explicit:
        path = Path(explicit).expanduser().resolve(strict=True)
    else:
        config = json.loads(config_path.read_bytes())
        models = [k for k in config if k.lower() in ('mid360', 'mid360s')]
        if len(models) != 1:
            return None
        model = 'MID360s' if models[0].lower() == 'mid360s' else 'MID360'
        path = config_path.parent.parent / 'launch_ROS1' / ('msg_' + model + '.launch')
        if not path.is_file():
            print('未找到默认驱动 launch，仅显示探测 SN：' + serial, flush=True)
            return None
        path = path.resolve()
    raw = path.read_bytes()
    text = raw.decode('utf-8')
    root = ElementTree.fromstring(text)
    args = [node for node in root.iter('arg') if node.get('name') == 'bd_list']
    if len(args) != 1 or args[0].get('default') is None:
        raise ConfigError('驱动 launch 中没有唯一的 bd_list 默认值，未修改：' + str(path))
    old = args[0].get('default')
    tags = list(re.finditer(r'<arg\b[^>]*\bname\s*=\s*[\'"]bd_list[\'"][^>]*>', text))
    if len(tags) != 1:
        raise ConfigError('不能安全定位 bd_list 标签：' + str(path))
    tag = tags[0]
    replacement, count = re.subn(r'(\bdefault\s*=\s*)([\'"])(.*?)\2',
                                lambda match: match[1] + match[2] + serial + match[2], tag[0])
    if count != 1:
        raise ConfigError('不能安全定位 bd_list.default')
    updated = (text[:tag.start()] + replacement + text[tag.end():]).encode()
    ElementTree.fromstring(updated)
    return (path, raw, updated, '驱动 bd_list SN：{} -> {}'.format(old, serial))


def report_running_localization():
    found = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            argv = path.joinpath('cmdline').read_bytes().split(b'\0')
            if not argv or (not argv[0].endswith(b'/fastlio_mapping') and b'__name:=factory_vision2mavros' not in argv):
                continue
            env = {}
            for field in path.joinpath('environ').read_bytes().split(b'\0'):
                if field.startswith((b'ROS_IP=', b'ROS_HOSTNAME=')):
                    name, value = field.decode().split('=', 1)
                    env[name] = value
            found.append((int(path.name), env))
        except (OSError, UnicodeError):
            continue
    for pid, env in found:
        try:
            loopback_ip = ipaddress.ip_address(env.get('ROS_IP', '')).is_loopback
        except ValueError:
            loopback_ip = False
        note = ('含回环限制；修改文件不会更新该进程，需重启定位/转发终端'
                if loopback_ip or env.get('ROS_HOSTNAME') in ('localhost', '127.0.0.1')
                else '未发现这次排查的回环限制')
        print('现有定位/转发进程 PID={} 环境={}；{}。'.format(pid, env, note), flush=True)
    return found


def plan_hostname_files(name, old_name, hostname_path=Path('/etc/hostname'), hosts_path=Path('/etc/hosts')):
    if not re.fullmatch(r'(?=.{1,63}$)[a-z0-9]+(?:-[a-z0-9]+)*', name):
        raise ConfigError('主机名必须是 1～63 位小写字母、数字或中间连字符')
    before_hostname = hostname_path.read_bytes()
    before_hosts = hosts_path.read_bytes()
    output = []
    inserted = False
    for line in before_hosts.decode().splitlines(keepends=True):
        content, separator, comment = line.partition('#')
        fields = content.split()
        try:
            address = ipaddress.ip_address(fields[0]) if fields else None
        except ValueError:
            address = None
        if address is None or not address.is_loopback:
            output.append(line)
            continue
        aliases = [item for item in fields[1:] if item not in (name, old_name)]
        if str(address) == '127.0.1.1' and not inserted:
            aliases.insert(0, name)
            inserted = True
        if aliases == fields[1:]:
            output.append(line)
        elif aliases:
            output.append(fields[0] + '\t' + ' '.join(aliases) + ('  #' + comment.rstrip('\n') if separator else '') + '\n')
        elif separator:
            output.append('#' + comment.rstrip('\n') + '\n')
    if not inserted:
        if output and not output[-1].endswith('\n'):
            output[-1] += '\n'
        output.append('127.0.1.1\t' + name + '\n')
    return [(hostname_path, before_hostname, (name + '\n').encode(), '系统主机名：{} -> {}'.format(old_name, name)),
            (hosts_path, before_hosts, ''.join(output).encode(), '本机 hosts 名称映射同步为 ' + name)]


def ifupdown_plan(interface, lidar, host, root=Path('/etc/network/interfaces')):
    """ifupdown-imported NM profiles are regenerated at boot: edit their origin.

    Keep all pre-existing hooks (especially removal of the overlapping /24).
    Add/update only our marked /32 hook; never reload/reapply the interface.
    """
    visited = set()
    matches = []

    def inspect(path):
        path = path.resolve()
        if path in visited or not path.is_file():
            return
        visited.add(path)
        raw = path.read_bytes()
        text = raw.decode('utf-8')
        for directive, value in re.findall(r'^\s*(source|source-directory)\s+([^#\n]+)', text, re.M):
            for pattern in value.split():
                target = Path(pattern)
                if not target.is_absolute():
                    target = path.parent / target
                for item in sorted(glob.glob(str(target))):
                    target_path = Path(item)
                    if directive == 'source-directory' and target_path.is_dir():
                        for child in sorted(target_path.iterdir()):
                            if re.fullmatch(r'[A-Za-z0-9_-]+', child.name):
                                inspect(child)
                    elif directive == 'source':
                        inspect(target_path)
        lines = text.splitlines(keepends=True)
        for start, line in enumerate(lines):
            match = re.match(r'^\s*iface\s+(\S+)\s+inet\s+(\S+)', line)
            if not match or match.group(1) != interface:
                continue
            end = start + 1
            while end < len(lines) and not re.match(r'^\s*(?:iface|mapping|auto|allow-\S+|source|source-directory)\s', lines[end]):
                end += 1
            matches.append((path, raw, lines, start, end, match.group(2)))

    inspect(root)
    if not matches:
        return None
    if len(matches) != 1:
        raise ConfigError('ifupdown 中存在重复网口定义，拒绝修改：' + interface)
    path, raw, lines, start, end, method = matches[0]
    if method != 'static':
        raise ConfigError('检测到 ifupdown {} 网口；只自动修改 static 网口的持久钩子'.format(method))
    begin = '    # BEGIN configure_livox managed route\n'
    finish = '    # END configure_livox managed route\n'
    body = ''.join(lines[start:end])
    marker = re.compile(r'^[ \t]*# BEGIN configure_livox managed route\n.*?^[ \t]*# END configure_livox managed route\n?', re.M | re.S)
    if ('BEGIN configure_livox' in body or 'END configure_livox' in body) and len(marker.findall(body)) != 1:
        raise ConfigError('ifupdown 的受管路由标记不完整或重复，请人工检查：' + str(path))
    block = begin + '    post-up ip -4 route replace {}/32 dev {} src {} scope link\n'.format(lidar, interface, host) + finish
    if marker.search(body):
        body = marker.sub(lambda _: block, body)
    else:
        if not body.endswith('\n'):
            body += '\n'
        body += block
    updated = (''.join(lines[:start]) + body + ''.join(lines[end:])).encode('utf-8')
    return path, raw, updated


def write_in_place(path, data):
    # Preserve owner, mode, inode and symlink target; never leave backup/temp files.
    with path.open('r+b', buffering=0) as stream:
        remaining = memoryview(data)
        while remaining:
            written = stream.write(remaining)
            if not written:
                raise OSError('写入配置时发生短写')
            remaining = remaining[written:]
        stream.truncate()
        os.fsync(stream.fileno())


def verify_route(lidar, interface, host):
    route = json.loads(run(['ip', '-j', '-4', 'route', 'get', lidar]))[0]
    if route.get('dev') != interface or route.get('prefsrc', route.get('src')) != host or route.get('gateway'):
        raise ConfigError('路由验证失败：' + str(route))
    return route


def remote_connection_routes(lidar):
    """Record actual SSH/NoMachine peer routes without relying on sudo's env."""
    peers = set()
    path = Path('/proc/net/tcp')
    if path.is_file():
        for line in path.read_text().splitlines()[1:]:
            fields = line.split()
            if len(fields) < 4 or fields[3] != '01':
                continue
            local, local_port = fields[1].split(':')
            remote, _ = fields[2].split(':')
            if int(local_port, 16) not in (22, 4000):
                continue
            peer = socket.inet_ntoa(struct.pack('<I', int(remote, 16)))
            if peer != lidar and not ipaddress.IPv4Address(peer).is_loopback:
                peers.add(peer)
    result = {}
    for peer in sorted(peers):
        route = json.loads(run(['ip', '-j', '-4', 'route', 'get', peer]))[0]
        result[peer] = {k: route.get(k) for k in ('dev', 'prefsrc', 'gateway')}
    return result


def verify_remote_connections(before, lidar):
    after = remote_connection_routes(lidar)
    # Also check peers whose TCP connection has since dropped.
    for peer, expected in before.items():
        route = json.loads(run(['ip', '-j', '-4', 'route', 'get', peer]))[0]
        actual = {k: route.get(k) for k in ('dev', 'prefsrc', 'gateway')}
        if actual != expected:
            raise ConfigError('远程连接 {} 的路由意外改变：{} -> {}'.format(peer, expected, actual))
    return after


def profile_has_route(routes, prefix):
    entries = re.findall(r'\{[^}]*\}', routes) if '{' in routes else routes.split(',')
    found = False
    for entry in entries:
        if not re.search(re.escape(prefix) + r'(?![0-9])', entry):
            continue
        found = True
        if '{' in entry:
            gateway = re.search(r'\bnh\s*=\s*([^,}\s]+)', entry)
            if gateway and gateway.group(1) not in ('0.0.0.0', '--'):
                raise ConfigError('已有同目标的网关路由，拒绝覆盖 NetworkManager 配置：' + entry)
        else:
            fields = entry.strip().split()
            if fields[0] != prefix or (len(fields) > 1 and fields[1] != '0.0.0.0'):
                raise ConfigError('无法确认已有路由为直连路由：' + entry)
    return found


def verify_ros(uri, seconds):
    end = time.monotonic() + seconds
    pending = {'/livox/lidar', '/livox/imu'}
    last_error = ''
    next_notice = 0
    while time.monotonic() < end and pending:
        try:
            master, publishers = ros_master(uri)
            for topic in sorted(pending.copy()):
                if not publishers.get(topic):
                    continue
                message = ros_message(master, publishers, topic, timeout=2)
                stamp, _ = ros_header(message)
                if -.5 <= time.time() - stamp <= 3:
                    pending.remove(topic)
                    print('已收到新鲜 ROS 数据：' + topic, flush=True)
        except (OSError, ConfigError, xmlrpc.client.Error) as exc:
            last_error = str(exc)
        if pending:
            if time.monotonic() >= next_notice:
                print('等待传感器服务启动：' + ', '.join(sorted(pending)), flush=True)
                next_notice = time.monotonic() + 10
            time.sleep(.5)
    if pending:
        raise ConfigError('超时，未收到新鲜数据 {}。{}'.format(sorted(pending), last_error))


def verify_driver_serial(uri, serial):
    if not serial:
        return
    code, _, value = proxy(uri).getParam('/livox_network_config', '/cmdline_str')
    if code == 1:
        if value != serial:
            raise ConfigError('运行中驱动 SN 参数 {} 与实物 {} 不同，服务可能覆盖 bd_list'.format(value, serial))
        print('运行中驱动 SN 参数与实物匹配：' + serial, flush=True)


def apply_plan(args, path, original, updated, device, host, uuid, nm_routes, ifupdown=None, extra_edits=()):
    print('检查飞控和地面状态中……', flush=True)
    flight_gate(args.ros_master)
    prefix = device['ip'] + '/32'
    interface = device['interface']
    control_routes = remote_connection_routes(device['ip'])
    if control_routes:
        print('保护现有 SSH/远程桌面路由：' + str(control_routes), flush=True)
    if path.read_bytes() != original:
        raise ConfigError('检查后配置被其他程序修改，请重新运行')
    if ifupdown and ifupdown[0].read_bytes() != ifupdown[1]:
        raise ConfigError('检查后 ifupdown 配置被其他程序修改，请重新运行')
    for target, before, after, description in extra_edits:
        if target.read_bytes() != before:
            raise ConfigError('检查后文件被其他程序修改，请重新运行：' + str(target))
    # Save only the selected destination's routes, in RAM; never touch default routes.
    old_routes = run(['ip', '-4', 'route', 'save', 'exact', prefix], binary=True)
    nm_added = False
    touched_ifupdown = False
    touched_file = False
    touched_route = False
    restart_attempted = False
    touched_edits = []
    old_hostname = socket.gethostname()
    touched_hostname = False
    try:
        for target, before, after, description in extra_edits:
            if before == after:
                continue
            print('应用启动配置：{}；文件 {}'.format(description, target), flush=True)
            touched_edits.append((target, before))
            write_in_place(target, after)
            if target.read_bytes() != after:
                raise ConfigError('启动配置写入校验失败：' + str(target))
        requested_hostname = getattr(args, 'hostname', None)
        if requested_hostname and old_hostname != requested_hostname:
            print('切换系统主机名：{} -> {}'.format(old_hostname, requested_hostname), flush=True)
            touched_hostname = True
            run(['hostnamectl', 'set-hostname', requested_hostname])
            if socket.gethostname() != requested_hostname:
                raise ConfigError('系统主机名切换后校验失败')
        if ifupdown and ifupdown[1] != ifupdown[2]:
            print('保存雷达的 ifupdown 开机路由，不重载网口：' + str(ifupdown[0]), flush=True)
            touched_ifupdown = True
            write_in_place(ifupdown[0], ifupdown[2])
            if ifupdown[0].read_bytes() != ifupdown[2]:
                raise ConfigError('ifupdown 配置写入校验失败')
        if uuid and not profile_has_route(nm_routes, prefix):
            print('保存雷达的 NetworkManager 专用路由……', flush=True)
            run(['nmcli', 'connection', 'modify', uuid, '+ipv4.routes', prefix + ' 0.0.0.0 0'])
            nm_added = True
        touched_route = True
        print('应用雷达单设备路由：{} dev {} src {}'.format(prefix, interface, host), flush=True)
        run(['ip', '-4', 'route', 'replace', prefix, 'dev', interface, 'src', host, 'scope', 'link'])
        verify_route(device['ip'], interface, host)
        verify_remote_connections(control_routes, device['ip'])
        if updated != original:
            print('更新驱动 JSON 中的雷达/主机地址……', flush=True)
            touched_file = True
            write_in_place(path, updated)
            if path.read_bytes() != updated:
                raise ConfigError('驱动配置写入校验失败')
        print('配置和路由已应用。无线地址、网口地址、默认路由均未修改，远程连接路由检查通过。', flush=True)
        if args.restart_service:
            flight_gate(args.ros_master)
            restart_attempted = True
            print('重启 {}（会同时重连 MAVROS、相机及雷达），不启动定位或飞行任务。'.format(args.service), flush=True)
            run(['systemctl', 'restart', args.service], timeout=30)
            verify_ros(args.ros_master, args.verify_seconds)
            if not getattr(args, 'network_only', False):
                verify_driver_serial(args.ros_master, device.get('serial'))
            print('验证通过：雷达点云和 IMU 已发布。定位及位姿转发请另行启动。', flush=True)
        else:
            print('未重启服务。运行中的驱动仍需重新加载配置；可再次加 --restart-service 执行。', flush=True)
        if extra_edits:
            print('启动配置已核对。请在两个终端依次启动定位和转发；若有旧进程仍在运行，须先关闭旧终端。', flush=True)
            report_running_localization()
    except BaseException:
        print('执行未完成，尝试用内存原值回滚本次配置和路由……', file=sys.stderr, flush=True)
        failures = []
        restore_edits = [('启动配置 ' + str(target), lambda p=target, data=before: write_in_place(p, data))
                         for target, before in reversed(touched_edits)]
        for description, action in restore_edits + [
            ('系统主机名', lambda: run(['hostnamectl', 'set-hostname', old_hostname]) if touched_hostname else None),
            ('JSON', lambda: write_in_place(path, original) if touched_file else None),
            ('ifupdown', lambda: write_in_place(ifupdown[0], ifupdown[1]) if touched_ifupdown else None),
            ('NetworkManager', lambda: run(['nmcli', 'connection', 'modify', uuid, '-ipv4.routes', prefix + ' 0.0.0.0 0']) if nm_added else None),
            ('route', lambda: restore_routes(prefix, old_routes) if touched_route else None),
        ]:
            try:
                action()
            except BaseException as exc:
                failures.append(description + ': ' + str(exc))
        if restart_attempted:
            try:
                flight_gate(args.ros_master)
                run(['systemctl', 'restart', args.service], timeout=30)
            except BaseException as exc:
                failures.append('原配置已尝试恢复，但服务未重新加载：' + str(exc))
        if failures:
            print('需要人工检查：' + '; '.join(failures), file=sys.stderr)
        else:
            print('已恢复本次修改前的配置和目标路由。', file=sys.stderr)
        raise


def restore_routes(prefix, saved):
    # Flush/restore only this one /32, not the interface/subnet/default routing table.
    # iproute2 restore uses ftell/fseek, so a pipe is insufficient. memfd is a
    # seekable anonymous RAM object, never a file/cache/backup on the board disk.
    descriptor = os.memfd_create('livox-route-rollback', os.MFD_CLOEXEC)
    with os.fdopen(descriptor, 'w+b') as stream:
        stream.write(saved)
        stream.flush()
        stream.seek(0)
        run(['ip', '-4', 'route', 'flush', 'exact', prefix])
        run(['ip', '-4', 'route', 'restore'], binary=True, stdin=stream)


def plan_gimbal(args, required=False):
    """Find the camera by ARP on physical Ethernet, including links without IPv4."""
    target = args.gimbal_ip
    if not target:
        config = Path('/etc/mediamtx/gimbal.yml')
        match = re.search(r'^\s*source:\s*[\'"]?(rtsp://[^\s\'"]+)',
                          config.read_text(), re.M) if config.is_file() else None
        target = urlparse(match.group(1)).hostname if match else '192.168.144.108'
    target = ipv4(target)
    address = ipaddress.IPv4Interface(args.gimbal_host_ip)
    ipv4(str(address.ip))
    if (ipaddress.IPv4Address(target) not in address.network or str(address.ip) == target
            or address.network.prefixlen > 30
            or ipaddress.IPv4Address(target) in (address.network.network_address, address.network.broadcast_address)
            or address.ip in (address.network.network_address, address.network.broadcast_address)):
        raise ConfigError('云台主机地址必须与相机同网段且不同，不能使用网络/广播地址')
    candidates = {}
    for item in json.loads(run(['ip', '-j', 'link', 'show'])):
        name = item['ifname']
        if (item.get('link_type') != 'ether' or 'UP' not in item.get('flags', [])
                or 'LOWER_UP' not in item.get('flags', [])
                or Path('/sys/class/net', name, 'wireless').exists()
                or not Path('/sys/class/net', name, 'device').exists()
                or (args.gimbal_interface and args.gimbal_interface != name)):
            continue
        candidates[name] = bytes.fromhex(item['address'].replace(':', ''))
    found = set()
    target_bytes = socket.inet_aton(target)
    with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0806)) as receiver:
        end = time.monotonic() + args.seconds
        next_probe = 0
        while candidates and time.monotonic() < end:
            if time.monotonic() >= next_probe:
                for name, mac in candidates.items():
                    # RFC 5227 ARP probe: no IP assignment, device-IP write or motion command.
                    packet = (b'\xff' * 6 + mac + b'\x08\x06'
                              + struct.pack('!HHBBH', 1, 0x0800, 6, 4, 1)
                              + mac + b'\0' * 4 + b'\0' * 6 + target_bytes)
                    receiver.sendto(packet, (name, 0))
                next_probe = time.monotonic() + 1
            ready, _, _ = select.select([receiver], [], [], min(.2, max(0, end - time.monotonic())))
            if not ready:
                continue
            frame, metadata = receiver.recvfrom(2048)
            name = metadata[0]
            if (name not in candidates or metadata[2] == socket.PACKET_OUTGOING
                    or len(frame) < 42 or frame[12:14] != b'\x08\x06'):
                continue
            if (frame[14:22] == struct.pack('!HHBBH', 1, 0x0800, 6, 4, 2)
                    and frame[28:32] == target_bytes and frame[22:28] != candidates[name]
                    and frame[32:38] == candidates[name]):
                found.add(name)
    if not found and not required and not args.gimbal_interface and not args.gimbal_ip:
        print('未发现云台相机 {} 的 ARP 应答，跳过云台配置；原有功能继续。'.format(target), flush=True)
        return None
    if len(found) != 1:
        raise ConfigError('云台相机 {} 需唯一识别网口，当前 {}；请检查供电/网线，或用 --gimbal-interface 指定'.format(target, sorted(found)))
    name = found.pop()
    for item in json.loads(run(['ip', '-j', '-4', 'address', 'show', 'dev', name])):
        for addr in item.get('addr_info', []):
            if addr.get('scope') == 'global' and addr.get('family') == 'inet':
                if ipaddress.IPv4Address(addr['local']) not in address.network:
                    raise ConfigError('云台候选网口 {} 还有其他网段地址 {}，拒绝覆盖'.format(name, addr['local']))
                if addr['local'] == str(address.ip) and addr['prefixlen'] != address.network.prefixlen:
                    raise ConfigError('云台主机 IP 已使用不同前缀，请先核对 --gimbal-host-ip')
    if not args.runtime_only and not shutil.which('nmcli'):
        raise ConfigError('云台持久 IP 配置需要 NetworkManager/nmcli；可用 --runtime-only')
    print('云台计划：相机 {}，发现网口 {}，主机 {}，{}；不设置默认网关。'.format(
        target, name, address, '仅本次运行' if args.runtime_only else '保存专用 NetworkManager 连接'), flush=True)
    return {'interface': name, 'ip': target, 'address': str(address), 'host': str(address.ip)}


def apply_gimbal(args, plan):
    """Apply only the identified camera NIC; retain rollback data in memory."""
    flight_gate(args.ros_master)
    name, target = plan['interface'], plan['ip']
    prefix = target + '/32'
    control_routes = remote_connection_routes(target)
    if any(route.get('dev') == name for route in control_routes.values()):
        raise ConfigError('云台候选网口正在承载 SSH/远程桌面连接，拒绝重配')
    old_routes = run(['ip', '-4', 'route', 'save', 'exact', prefix], binary=True)
    current_addresses = json.loads(run(['ip', '-j', '-4', 'address', 'show', 'dev', name]))
    already_addressed = any(a.get('local') == plan['host'] for item in current_addresses for a in item.get('addr_info', []))
    profile_name = 'configure_livox-gimbal'
    uuid = None
    old_settings = None
    old_active = ''
    added_address = False
    keys = ['connection.interface-name', 'connection.autoconnect', 'connection.autoconnect-priority',
            'ipv4.method', 'ipv4.addresses', 'ipv4.gateway', 'ipv4.never-default', 'ipv4.routes',
            'ipv6.method']
    route_touched = profile_touched = False
    try:
        if args.runtime_only:
            if not already_addressed:
                run(['ip', '-4', 'address', 'add', plan['address'], 'dev', name])
                added_address = True
        else:
            old_active = run(['nmcli', '-g', 'GENERAL.CON-UUID', 'device', 'show', name])
            rows = run(['nmcli', '-t', '-f', 'UUID,NAME', 'connection', 'show']).splitlines()
            matches = [row.split(':', 1)[0] for row in rows if row.split(':', 1)[-1] == profile_name]
            if len(matches) > 1:
                raise ConfigError('存在重复的云台专用连接名称，拒绝修改')
            if matches:
                uuid = matches[0]
                old_settings = {key: run(['nmcli', '-g', key, 'connection', 'show', uuid]) for key in keys}
                active = run(['nmcli', '-t', '-f', 'UUID,DEVICE', 'connection', 'show', '--active']).splitlines()
                if any(row.startswith(uuid + ':') and row.split(':', 1)[1] != name for row in active):
                    raise ConfigError('云台专用连接正在其他网口使用，拒绝修改')
            settings = ['connection.interface-name', name, 'connection.autoconnect', 'yes',
                        'connection.autoconnect-priority', '100', 'ipv4.method', 'manual',
                        'ipv4.addresses', plan['address'], 'ipv4.gateway', '', 'ipv4.never-default', 'yes',
                        'ipv4.routes', '', 'ipv6.method', 'disabled']
            if old_settings is None:
                # Let NetworkManager allocate UUID: older nmcli rejects setting
                # connection.uuid during 'connection add'. A failed add creates
                # no connection and must not trigger a misleading delete error.
                run(['nmcli', 'connection', 'add', 'type', 'ethernet', 'ifname', name,
                     'con-name', profile_name] + settings)
                profile_touched = True
                uuid = run(['nmcli', '-g', 'connection.uuid', 'connection', 'show', profile_name])
            else:
                profile_touched = True
                run(['nmcli', 'connection', 'modify', uuid] + settings)
            run(['nmcli', '--wait', '15', 'connection', 'up', uuid, 'ifname', name], timeout=20)
        route_touched = True
        run(['ip', '-4', 'route', 'replace', prefix, 'dev', name, 'src', plan['host'], 'scope', 'link'])
        verify_route(target, name, plan['host'])
        verify_remote_connections(control_routes, target)
        with socket.create_connection((target, 554), timeout=5):
            pass
        print('云台网口配置完成，{}:554 已连通；未重启服务、未发送运动指令。'.format(target), flush=True)
    except BaseException:
        failures = []
        actions = []
        if profile_touched:
            if old_settings is None:
                actions.append(('删除本次新建云台连接', lambda: run(['nmcli', 'connection', 'delete', uuid or profile_name])))
            else:
                settings = [value for key in keys for value in (key, old_settings[key])]
                actions.append(('还原云台连接', lambda: run(['nmcli', 'connection', 'modify', uuid] + settings)))
            if old_active and old_active != '--':
                actions.append(('恢复原活动连接', lambda: run(['nmcli', '--wait', '15', 'connection', 'up', old_active, 'ifname', name], timeout=20)))
        if added_address:
            actions.append(('移除本次地址', lambda: run(['ip', '-4', 'address', 'del', plan['address'], 'dev', name])))
        if route_touched:
            actions.append(('还原云台路由', lambda: restore_routes(prefix, old_routes)))
        for description, action in actions:
            try:
                action()
            except BaseException as exc:
                failures.append(description + ': ' + str(exc))
        print('云台配置未完成，已尝试回滚本次云台改动。' + ('需要检查：' + '; '.join(failures) if failures else ''), file=sys.stderr, flush=True)
        raise


def main():
    parser = argparse.ArgumentParser(description='Livox MID360/MID360s 网络检查与配置；默认只检查，不写缓存或备份。')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='只检查（默认）；发只读发现请求并监听数据')
    mode.add_argument('--apply', action='store_true', help='应用已唯一识别设备的驱动地址及 /32 路由')
    parser.add_argument('--interface', help='限制到指定有线网口，例如 enP3p49s0')
    parser.add_argument('--gimbal-only', action='store_true', help='只发现/配置云台网口，不执行雷达或 SSH 配置；默认只检查，应用需 --apply')
    parser.add_argument('--gimbal-interface', help='可选：限定云台有线网口，仍需相机 ARP 应答验证')
    parser.add_argument('--gimbal-ip', help='云台相机 IP；默认读取 /etc/mediamtx/gimbal.yml 的 RTSP 源')
    parser.add_argument('--gimbal-host-ip', default='192.168.144.100/24', help='云台专网主机地址/前缀，默认 192.168.144.100/24')
    parser.add_argument('--lidar-ip', help='多设备时选择此雷达，仍要求发现验证')
    parser.add_argument('--host-ip', help='选择该网口已有的主机 IP，不自动改网口地址')
    parser.add_argument('--config', help='驱动实际使用的 JSON；默认读取 ROS /user_config_path')
    parser.add_argument('--entry-ip', help='多雷达 JSON 中要替换的原条目 IP')
    parser.add_argument('--ros-launcher', help='定位公共启动脚本路径，默认在用户 drone_platform/01_apps/fast_lio/management 下')
    parser.add_argument('--driver-launch', help='需同步 SN 的实际雷达 launch，默认根据当前 JSON 型号查找')
    parser.add_argument('--network-only', action='store_true', help='只处理雷达 IP/路由，跳过 ROS 启动环境及 launch SN 同步')
    parser.add_argument('--hostname', help='同步系统主机名、/etc/hostname 和 /etc/hosts；例如 orangepi5ultra')
    parser.add_argument('--ssh-identity', action='store_true', help='只检查并保留机载 SSH 主机身份，不要求配置 IP')
    parser.add_argument('--groundstation-ssh', action='store_true', help='在电脑端自动读取地面站目标并配置 SSH 身份记录，不修改地面站程序')
    parser.add_argument('--install-ssh-helper', action='store_true', help='电脑端首次安装受配对保护的 SSH 辅助服务；以后只在机载端执行')
    parser.add_argument('--serve-ssh-helper', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--ssh-sync', action='store_true', help='机载端自动发现已配对电脑，更新本机对应的电脑 SSH 记录；不操作雷达')
    parser.add_argument('--groundstation-config', help='现有地面站连接 JSON；默认自动查找 .pointcloud-link/connection.json')
    parser.add_argument('--ssh-host', help='可选的目标覆盖值；不提供时自动读取地面站配置，不写死 IP')
    parser.add_argument('--ssh-user', help='可选的 SSH 用户覆盖值，默认读取连接配置')
    parser.add_argument('--ssh-port', type=int, help='可选的 SSH 端口覆盖值，默认读取连接配置或使用 22')
    parser.add_argument('--known-hosts', help='可选的电脑端 SSH 身份文件，默认遵循当前用户的 SSH 配置')
    parser.add_argument('--trust-fingerprint', help='已从设备/可信通道核对的完整 SHA256 指纹；身份变化时用于明确授权更新')
    parser.add_argument('--runtime-only', action='store_true', help='只设置本次运行的路由，不写持久网络配置')
    parser.add_argument('--restart-service', action='store_true', help='应用后重启传感器服务并验证雷达数据；会重连 MAVROS 和相机')
    parser.add_argument('--service', default='Safina_sensor.service', help='传感器 systemd 服务名')
    parser.add_argument('--seconds', type=float, default=5, help='发现/监听秒数，默认 5')
    parser.add_argument('--verify-seconds', type=float, default=100, help='服务重启后等待数据的秒数，默认 100')
    parser.add_argument('--ros-master', default=os.environ.get('ROS_MASTER_URI', 'http://127.0.0.1:11311/'))
    args = parser.parse_args()
    if args.gimbal_only and any((args.groundstation_ssh, args.install_ssh_helper, args.serve_ssh_helper, args.ssh_sync, args.ssh_identity, args.restart_service, args.hostname)):
        parser.error('--gimbal-only 不能混用 SSH、主机名或传感器重启选项')
    if sum(bool(value) for value in (args.groundstation_ssh, args.install_ssh_helper, args.serve_ssh_helper, args.ssh_sync, args.ssh_identity)) > 1:
        parser.error('SSH 专用模式不能混用')
    if args.serve_ssh_helper or args.ssh_sync:
        if args.restart_service or args.hostname or args.groundstation_config or args.ssh_host or args.trust_fingerprint:
            parser.error('SSH 辅助模式不能混用雷达或电脑配置参数')
        return serve_ssh_helper() if args.serve_ssh_helper else sync_computer_ssh(args.apply)
    if args.install_ssh_helper:
        if args.restart_service or args.hostname:
            parser.error('首次电脑配对不能混用机载操作选项')
        return install_ssh_helper(args)
    if args.groundstation_ssh:
        if args.ssh_identity or args.restart_service or args.hostname:
            parser.error('--groundstation-ssh 只处理电脑 SSH 记录，不能混用机载操作选项')
        return configure_groundstation_ssh(args)
    if any((args.groundstation_config, args.ssh_host, args.ssh_user, args.ssh_port is not None, args.known_hosts, args.trust_fingerprint)):
        parser.error('电脑端 SSH 参数必须与 --groundstation-ssh 一起使用')
    if args.ssh_identity:
        if args.restart_service:
            parser.error('--ssh-identity 不重启传感器服务，请勿同时使用 --restart-service')
        return configure_server_ssh_identity(args)
    if args.restart_service and not args.apply:
        parser.error('--restart-service 需要 --apply')
    if not 1 <= args.seconds <= 60 or not 5 <= args.verify_seconds <= 300:
        parser.error('发现时间应为 1～60 秒，验证时间为 5～300 秒')
    if os.geteuid() != 0:
        raise ConfigError('请用 sudo python3 -B 运行；只检查也需要网口监听权限')
    if args.apply and not hasattr(os, 'memfd_create'):
        raise ConfigError('内存路由回滚需要 Linux memfd/Python 3.8+；未修改配置')
    if args.gimbal_only:
        plan = plan_gimbal(args, required=True)
        if not args.apply:
            print('云台只检查结束；应用请加 --apply。未修改 IP 或配置。', flush=True)
            return 0
        with open(__file__, 'rb') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise ConfigError('已有另一个实例正在应用配置')
            apply_gimbal(args, plan)
        return 0
    show_ssh_identity()
    if args.lidar_ip:
        args.lidar_ip = ipv4(args.lidar_ip)
    # SUDO_USER/HOME is not assumed to point at /home/orangepi on a replacement board.
    import pwd
    owner = os.environ.get('SUDO_USER') or os.environ.get('USER') or 'root'
    home = Path(pwd.getpwnam(owner).pw_dir)
    path, original = find_config(args.config, args.ros_master, home, wait_seconds=60)
    ifaces = interfaces()
    if args.interface:
        if args.interface not in ifaces:
            raise ConfigError('指定网口不是已启用、有 IPv4 的物理有线网口：' + args.interface)
        ifaces = {args.interface: ifaces[args.interface]}
    if not ifaces:
        raise ConfigError('没有已启用且配置 IPv4 的物理有线网口，请先连接雷达并配置主机地址')
    print('驱动配置：' + str(path), flush=True)
    print('检查有线网口：' + str(ifaces), flush=True)
    print('发现雷达中（只读 SDK2 查询，不修改雷达设备 IP）……', flush=True)
    observations = discover(ifaces, args.seconds)
    for item in observations:
        print('发现：网口={} IP={} 型号={} 序列号={} 点云包={} IMU包={}'.format(
            item['interface'], item['ip'], item['type'], item['serial'],
            item['ports'][56300], item['ports'][56400]), flush=True)
    device = choose_device(observations, args.lidar_ip)
    ipv4(device['ip'])
    host = host_address(device, ifaces, args.host_ip)
    updated, changes = plan_config(original, device, host, args.entry_ip)
    ifupdown = None if args.runtime_only else ifupdown_plan(device['interface'], device['ip'], host)
    uuid, nm_routes = (None, '') if args.runtime_only or ifupdown else nm_profile(device['interface'])
    current_route = json.loads(run(['ip', '-j', '-4', 'route', 'get', device['ip']]))
    print('当前路由：' + str(current_route), flush=True)
    print('目标：{} -> {}，有线网口 {}，源地址 {}'.format(
        host, device['ip'], device['interface'], host), flush=True)
    for change in changes:
        print('配置变更：' + change, flush=True)
    if not changes:
        print('JSON 配置已匹配。', flush=True)
    persistence = '本次运行有效' if args.runtime_only else ('保存到 ifupdown 启动文件 ' + str(ifupdown[0]) if ifupdown else '保存到当前 NetworkManager 连接')
    print('路由：仅设置 {}/32，{}。'.format(device['ip'], persistence), flush=True)
    extra_edits = []
    if args.hostname:
        extra_edits.extend(plan_hostname_files(args.hostname, socket.gethostname()))
    if not args.network_only:
        launcher = Path(args.ros_launcher).expanduser() if args.ros_launcher else home / 'drone_platform/01_apps/fast_lio/management/manual-launch.sh'
        if launcher.is_file():
            ros_host = sensor_ros_hostname(args.ros_master)
            if args.hostname and ros_host != args.hostname:
                raise ConfigError('目标系统主机名 {} 与传感器通告名 {} 不同，请先统一传感器配置；未修改文件'.format(args.hostname, ros_host))
            edit = plan_ros_launcher(launcher, ros_host)
            if edit:
                extra_edits.append(edit)
        else:
            print('未找到本平台定位启动脚本，跳过该项：' + str(launcher), flush=True)
        edit = plan_driver_serial(path, device, args.driver_launch)
        if edit:
            extra_edits.append(edit)
        for target, before, after, description in extra_edits:
            print(('计划修复：' if before != after else '已匹配：') + description, flush=True)
        report_running_localization()
    gimbal_plan = plan_gimbal(args)
    if not args.apply:
        print('只检查结束；未写配置、路由或备份。应用请加 --apply；重新加载驱动再加 --restart-service。', flush=True)
        return 0
    if args.restart_service:
        if '/' in args.service or not args.service.endswith('.service'):
            raise ConfigError('服务名必须是不含路径的 .service 名称')
        if run(['systemctl', 'show', '-p', 'LoadState', '--value', args.service]) != 'loaded':
            raise ConfigError('指定传感器服务不存在：' + args.service)
    # Lock this existing script inode; no lock file is created on the board.
    with open(__file__, 'rb') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ConfigError('已有另一个实例正在应用配置')
        apply_plan(args, path, original, updated, device, host, uuid, nm_routes, ifupdown, extra_edits)
        if gimbal_plan:
            apply_gimbal(args, gimbal_plan)
    if (owner_home() / SSH_HELPER_DIR / 'pairing.json').is_file():
        print('雷达配置已完成，开始同步电脑 SSH 身份记录……', flush=True)
        return sync_computer_ssh(apply=True)
    return 0


if __name__ == '__main__':
    def stop_requested(signum, _frame):
        raise KeyboardInterrupt('收到停止信号 ' + str(signum))
    signal.signal(signal.SIGTERM, stop_requested)
    try:
        sys.exit(main())
    except (ConfigError, OSError, ValueError, struct.error, xmlrpc.client.Error, ElementTree.ParseError, subprocess.SubprocessError) as error:
        print('未完成：' + str(error), file=sys.stderr, flush=True)
        sys.exit(1)
    except KeyboardInterrupt:
        print('已停止。', file=sys.stderr, flush=True)
        sys.exit(130)
