#!/usr/bin/env python3
"""切换安装版本前检查旧GUI，仅停止旧版本自己的本地只读遥测后台。"""
import argparse
import json
from pathlib import Path
import subprocess


def prepare(home, user, target, proc_root=Path('/proc'), run=subprocess.run):
    base = home / '.local/opt'
    old = []
    for prefix in base.glob('research-flight-groundstation*'):
        if prefix.is_symlink() or not prefix.is_dir() or prefix.resolve() == target.resolve():
            continue
        try:
            marker = json.loads((prefix / '.installation.json').read_text())
            if marker.get('product') == 'research-flight-groundstation':
                old.append(prefix)
        except (OSError, ValueError):
            continue
    binaries = {(p / 'build/linux/customgeometry').resolve() for p in old}
    for proc in proc_root.iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if (proc / 'exe').resolve() in binaries:
                raise RuntimeError('请先关闭旧安装版地面站，再运行安装器；尚未停止任何后台。')
        except OSError:
            pass
    for prefix in old:
        for relative in ('tools/preflight/bridge.py', 'tools/pointcloud-only/manage.py'):
            script = prefix / 'dianyunGCS-master' / relative
            if not script.is_file():
                continue
            # 旧管理器的stop会核对PID、启动时间和脚本归属，不调用机载控制。
            result = run(['runuser', '-u', user, '--', 'env', 'HOME=' + str(home),
                          '/usr/bin/python3', str(script), 'stop'],
                         capture_output=True, text=True, timeout=25)
            if result.returncode:
                raise RuntimeError('旧版只读桥接未停止，未切换快捷方式：' + str(script) + '\n' + result.stderr[-1000:])
    return len(old)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--home', type=Path, required=True)
    parser.add_argument('--user', required=True)
    parser.add_argument('--target', type=Path, required=True)
    args = parser.parse_args()
    count = prepare(args.home, args.user, args.target)
    if count:
        print('已检查旧安装并停止其本地只读桥接；旧程序、日志和用户数据均保留。')


if __name__ == '__main__':
    main()
