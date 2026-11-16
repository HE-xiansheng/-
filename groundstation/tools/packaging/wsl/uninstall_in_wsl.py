#!/usr/bin/env python3
"""仅卸载指定普通用户已登记的地面站；用户数据先备份，不操作机载或发行版。"""
import argparse
import datetime
import json
import os
from pathlib import Path, PurePosixPath
import pwd
import re
import shutil
import subprocess
import uuid

PRODUCT = 'research-flight-groundstation'


def installations(home, uid):
    base = home / '.local/opt'
    if not base.exists():
        return []
    if base.resolve() != base:
        raise RuntimeError('安装目录父路径包含符号链接，未自动删除。')
    found = []
    for prefix in base.iterdir():
        if not re.fullmatch(r'research-flight-groundstation(?:-wsl-[A-Za-z0-9._-]+)?', prefix.name):
            continue
        marker = prefix / '.installation.json'
        if prefix.is_symlink() or not prefix.is_dir() or not marker.is_file() or marker.is_symlink():
            continue
        receipt = json.loads(marker.read_text())
        if receipt.get('product') != PRODUCT:
            continue
        if prefix.stat().st_uid != uid or marker.stat().st_uid != uid:
            raise RuntimeError('安装目录归属与指定用户不符，未删除：' + str(prefix))
        manifest_file = prefix / 'payload-manifest.json'
        if manifest_file.is_symlink():
            raise RuntimeError('安装清单为符号链接，未删除：' + str(prefix))
        manifest = json.loads(manifest_file.read_text())
        shipped = set(manifest['files'])
        for relative in shipped:
            path = PurePosixPath(relative)
            if path.is_absolute() or '..' in path.parts or not path.parts:
                raise RuntimeError('安装清单包含越界路径，未删除：' + str(prefix))
        found.append((prefix, receipt, shipped, marker.stat().st_mtime_ns))
    return sorted(found, key=lambda item: item[3], reverse=True)


def assert_closed(items, proc_root):
    binaries = {(item[0] / 'build/linux/customgeometry').resolve() for item in items}
    for proc in proc_root.iterdir():
        if not proc.name.isdigit():
            continue
        try:
            running = (proc / 'exe').resolve()
        except OSError:
            continue
        if running in binaries:
            raise RuntimeError('请先关闭地面站窗口，再双击一键卸载；尚未删除程序。')


def preserve_extra_files(prefix, shipped, destination):
    """不跟随符号链接；备份失败时保留原程序，不执行删除。"""
    saved = []
    for directory, dirs, files in os.walk(prefix, followlinks=False):
        links = [name for name in dirs if (Path(directory) / name).is_symlink()]
        dirs[:] = [name for name in dirs if name not in links]
        for name in files + links:
            source = Path(directory) / name
            relative = source.relative_to(prefix)
            if str(relative) in shipped or str(relative) in ('.installation.json', 'payload-manifest.json'):
                continue
            if source.is_symlink():
                # 已打包库的内部符号链接无需备份；其他链接只保存链接本身。
                try:
                    if str(source.resolve().relative_to(prefix)) in shipped:
                        continue
                except ValueError:
                    pass
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            if source.is_symlink():
                target.symlink_to(os.readlink(source))
            else:
                shutil.copy2(source, target)
            saved.append(str(relative))
    return saved


def restore_shortcuts(home, receipt):
    for entry in receipt.get('shortcuts', []):
        path = Path(entry['path'])
        if path.name not in ('research-flight-groundstation.desktop', '科研飞行平台-地面站.desktop') or path.is_symlink():
            continue
        try:
            path.resolve().relative_to(home)
        except ValueError:
            continue
        if path.is_file() and path.read_text() == entry['installed']:
            if entry.get('previous') is None:
                path.unlink()
            else:
                path.write_text(entry['previous'])


def uninstall(home, user, uid, proc_root=Path('/proc'), run=subprocess.run):
    home = home.resolve()
    items = installations(home, uid)
    assert_closed(items, proc_root)
    # 所有版本均先检查窗口，再逐个回收各自有PID归属检查的只读遥测后台。
    for prefix, _, _, _ in items:
        for relative in ('preflight/bridge.py', 'pointcloud-only/manage.py'):
            script = prefix / 'dianyunGCS-master/tools' / relative
            if script.is_file() and not script.is_symlink():
                result = run(['/usr/bin/python3', str(script), 'stop'],
                             capture_output=True, text=True, timeout=25)
                if result.returncode:
                    raise RuntimeError('本地只读后台未停止，未删除程序：' + str(script) + '\n' + result.stderr[-1000:])
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:8]
    backup = home / '.local/share' / PRODUCT / ('uninstall-' + stamp)
    saved = []
    # 先备份全部目录的额外文件；任何备份失败都不会删除任何版本。
    for prefix, receipt, shipped, _ in items:
        destination = backup / prefix.name
        extra = preserve_extra_files(prefix, shipped, destination)
        if extra:
            saved.append({'prefix': str(prefix), 'directory': str(destination), 'files': extra})
    assert_closed(items, proc_root)
    removed = []
    for prefix, receipt, _, _ in items:
        restore_shortcuts(home, receipt)
        shutil.rmtree(prefix)
        removed.append(str(prefix))
    return {'success': True, 'user': user, 'removed': removed, 'saved': saved,
            'preserved': 'WSL、Ubuntu、Linux用户、系统依赖、SSH配置、应用设置及安装目录外数据均保留'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--user', required=True)
    parser.add_argument('--result-file', type=Path, required=True)
    args = parser.parse_args()
    if not re.fullmatch(r'[a-z_][a-z0-9_-]*\$?', args.user) or args.user == 'root':
        raise RuntimeError('目标普通用户无效。')
    try:
        account = pwd.getpwnam(args.user)
    except KeyError:
        result = {'success': True, 'user': args.user, 'removed': [], 'saved': []}
    else:
        if account.pw_uid == 0:
            raise RuntimeError('不能以root用户卸载地面站。')
        if os.geteuid() != account.pw_uid:
            if os.geteuid() != 0:
                raise RuntimeError('当前用户与目标安装用户不同，未删除。')
            # 仅账户发现使用root，实际备份/删除始终降到原安装用户。
            subprocess.run(['runuser', '-u', args.user, '--', 'env', 'HOME=' + account.pw_dir,
                            '/usr/bin/python3', str(Path(__file__).resolve()), '--user', args.user,
                            '--result-file', str(args.result_file)], check=True)
            return
        result = uninstall(Path(account.pw_dir), args.user, account.pw_uid)
    args.result_file.write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
