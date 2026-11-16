#!/usr/bin/env python3
"""读取WSL端地面站进程、桥接日志与本机套接字，不连接机载或改变服务。"""
import ctypes.util
import datetime
import importlib.util
import json
import os
from pathlib import Path
import pwd
import re
import socket
import subprocess


def command(args):
    try:
        p = subprocess.run(args, capture_output=True, text=True, errors='replace', timeout=8)
        return {'code': p.returncode, 'out': p.stdout[-24000:], 'err': p.stderr[-4000:]}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'error': str(exc)}


def read_tail(path, limit=16000):
    try:
        with path.open('rb') as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - limit))
            text = f.read().decode('utf-8', errors='replace')
        # 不采集环境变量/私钥；常见明文密码赋值仍做额外脱敏。
        return re.sub(r'(?im)((?:GCS_SSH_PASSWORD|password)\s*[:=]\s*)[^\r\n]+', r'\1[redacted]', text)
    except OSError as exc:
        return '[read error] ' + str(exc)


def main():
    processes, roots = [], set()
    scripts = {'bridge.py', 'manage.py', 'manage_link.py', 'windows_bridge.py'}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            args = [os.fsdecode(x) for x in (proc / 'cmdline').read_bytes().split(b'\0') if x]
            exe = Path(os.readlink(proc / 'exe').removesuffix(' (deleted)')) if hasattr(str, 'removesuffix') else Path(os.readlink(proc / 'exe').replace(' (deleted)', ''))
            script = next((Path(x) for x in args[1:3] if Path(x).name in scripts and '/tools/' in x), None)
            if exe.name != 'customgeometry' and script is None:
                continue
            user = pwd.getpwuid(proc.stat().st_uid).pw_name
            fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
            record = {'pid': int(proc.name), 'ppid': int(fields[1]), 'user': user, 'exe': str(exe)}
            if script:
                record['script'] = str(script)
                roots.add(script.resolve().parents[3])
            else:
                roots.add(exe.parents[2])
            # 仅取已知诊断参数，不输出SSH命令、密码环境或任意进程参数。
            for flag in ('--host', '--port', '--cloud-port', '--pose-port', '--preflight-port'):
                if flag in args and args.index(flag) + 1 < len(args):
                    record[flag] = args[args.index(flag) + 1]
            processes.append(record)
        except (OSError, ValueError, IndexError, KeyError):
            continue
    # 即使界面已经关闭，也能读取默认安装位置的最近日志。
    try:
        base = Path(pwd.getpwnam('gcs').pw_dir) / '.local/opt'
        roots.update(p for p in base.glob('research-flight-groundstation*') if p.is_dir())
    except KeyError:
        pass
    installations = []
    for root in sorted(roots):
        state = root / '.pointcloud-link'
        entry = {'prefix': str(root), 'states': {}, 'logs': {}}
        for name in ('connection.json', 'preflight-active.json', 'active.json'):
            path = state / name
            if path.is_file():
                try:
                    data = json.loads(path.read_text())
                    allowed = {'host', 'port', 'cloud_port', 'pose_port', 'pid', 'start_tick', 'log', 'pc_ip'}
                    entry['states'][name] = {k: v for k, v in data.items() if k in allowed}
                except (ValueError, OSError) as exc:
                    entry['states'][name] = {'error': str(exc)}
        for directory, pattern in ((state, 'preflight-*.log'), (state, 'sender-*.log'),
                                   (root / 'build/linux/logs', 'customgeometry_*.log')):
            try:
                files = sorted(directory.glob(pattern), key=lambda p: p.stat().st_mtime, reverse=True)[:2]
                for path in files:
                    entry['logs'][str(path)] = read_tail(path)
            except OSError as exc:
                entry['logs'][str(directory)] = str(exc)
        installations.append(entry)
    sockets = command(['ss', '-lunp'])
    if 'out' in sockets:
        sockets['out'] = '\n'.join(x for x in sockets['out'].splitlines() if re.search(r':(?:900[0-5]|19003)\b', x))
    report = {
        'schema': 1, 'time_utc': datetime.datetime.utcnow().isoformat() + 'Z',
        'processes': processes, 'udp_sockets': sockets,
        'network_mode': command(['wslinfo', '--networking-mode']),
        'ipv4_routes': command(['ip', '-4', 'route']),
        'ssh_version': command(['ssh', '-V']),
        'python_numpy_found': importlib.util.find_spec('numpy') is not None,
        'lz4_library': ctypes.util.find_library('lz4'), 'installations': installations,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
