#!/usr/bin/env python3
"""安装打包后的完整地面站到用户目录，并登记桌面入口。"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys

DEPS=['python3-numpy','openssh-client','gstreamer1.0-tools','gstreamer1.0-plugins-base',
      'gstreamer1.0-plugins-good','gstreamer1.0-plugins-bad','gstreamer1.0-libav','fonts-noto-cjk']

def dependency_errors():
    missing=[]
    for cmd in ['ssh','setsid','gst-launch-1.0','gst-inspect-1.0']:
        if not shutil.which(cmd):missing.append(cmd)
    if subprocess.run(['/usr/bin/python3','-c','import numpy'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:missing.append('python3-numpy')
    if shutil.which('gst-inspect-1.0'):
        for plugin in ['rtspsrc','rtph264depay','h264parse','avdec_h264','videoconvert','videoscale','fdsink']:
            if subprocess.run(['gst-inspect-1.0',plugin],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:missing.append(plugin)
    return missing

def desktop_entry(prefix):
    launcher=prefix/'dianyunGCS-master/tools/desktop/launch_groundstation.py'
    # Desktop Entry 中的百分号是字段占位符；路径只作为普通参数传入。
    esc=lambda s:str(s).replace('\\','\\\\').replace('"','\\"').replace('`','\\`').replace('$','\\$').replace('%','%%')
    return ('[Desktop Entry]\nType=Application\nVersion=1.0\nName=科研飞行平台-地面站\n'
            'Comment=三维点云、飞行规划与数据回放\n'
            'Exec=/usr/bin/python3 "'+esc(launcher)+'"\n'
            'Icon='+str(prefix/'dianyunGCS-master/assets/icons/research-flight-gcs-v1.png')+'\n'
            'Terminal=false\nStartupNotify=true\nStartupWMClass=customgeometry\nCategories=Science;\n')

def install(source,prefix,apps,desktop):
    if prefix.exists():raise RuntimeError('安装目录已存在：'+str(prefix)+'。请先卸载旧安装版，或用 --prefix 指定新目录。')
    manifest=json.loads((source/'payload-manifest.json').read_text())
    for relative,sha in manifest['files'].items():
        p=source/relative
        if hashlib.sha256(p.read_bytes()).hexdigest()!=sha:raise RuntimeError('安装内容校验失败：'+relative)
    prefix.parent.mkdir(parents=True,exist_ok=True)
    shutil.copytree(source,prefix,symlinks=True)
    receipt={'product':'research-flight-groundstation','version':manifest['version'],'shortcuts':[]}
    (prefix/'.installation.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2))
    content=desktop_entry(prefix)
    for folder,name in [(apps,'research-flight-groundstation.desktop'),(desktop,'科研飞行平台-地面站.desktop')]:
        if folder is None:continue
        folder.mkdir(parents=True,exist_ok=True);p=folder/name
        record={'path':str(p),'installed':content,'previous':p.read_text() if p.exists() else None}
        p.write_text(content);p.chmod(0o755);receipt['shortcuts'].append(record)
        (prefix/'.installation.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2))
        if folder==desktop and shutil.which('gio'):subprocess.run(['gio','set',str(p),'metadata::trusted','true'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    if shutil.which('update-desktop-database'):subprocess.run(['update-desktop-database',str(apps)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    print('安装完成：'+str(prefix));print('从桌面或应用菜单打开“科研飞行平台-地面站”。')
    print('卸载命令：/usr/bin/python3 "'+str(prefix/'install_user.py')+'" --uninstall')

def uninstall(prefix,data_root):
    marker=prefix/'.installation.json'
    if not marker.is_file():raise RuntimeError('目录不是本安装器登记的安装目录，未删除。')
    receipt=json.loads(marker.read_text())
    if receipt.get('product')!='research-flight-groundstation':raise RuntimeError('安装标记不匹配。')
    binary=(prefix/'build/linux/customgeometry').resolve()
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            if (p/'exe').resolve()==binary:raise RuntimeError('请先关闭此安装版地面站，再卸载。')
        except (OSError,PermissionError):pass
    for entry in receipt['shortcuts']:
        p=Path(entry['path'])
        if p.exists() and p.read_text()==entry['installed']:
            if entry['previous'] is None:p.unlink()
            else:p.write_text(entry['previous'])
    saved=[]
    for relative in ['.pointcloud-link','build/linux/logs']:
        p=prefix/relative
        if p.exists():
            dest=data_root/('uninstall-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))/relative
            dest.parent.mkdir(parents=True,exist_ok=True);shutil.move(str(p),str(dest));saved.append(str(dest))
    os.chdir(prefix.parent);shutil.rmtree(prefix)
    print('已卸载。'+(' 日志与通信记录保存在：'+', '.join(saved) if saved else ''))

def main():
    p=argparse.ArgumentParser(description='科研飞行平台-地面站 Ubuntu 用户安装器')
    p.add_argument('--prefix',type=Path,default=Path.home()/'.local/opt/research-flight-groundstation')
    p.add_argument('--applications-dir',type=Path,default=Path.home()/'.local/share/applications')
    p.add_argument('--desktop-dir',type=Path)
    p.add_argument('--no-desktop',action='store_true');p.add_argument('--install-deps',action='store_true')
    p.add_argument('--uninstall',action='store_true');p.add_argument('--source',type=Path,default=Path(__file__).resolve().parent)
    p.add_argument('--data-dir',type=Path,default=Path.home()/'.local/share/research-flight-groundstation')
    a=p.parse_args();prefix=a.prefix.expanduser().resolve()
    if a.uninstall:
        # 从已安装脚本启动卸载时，使用该脚本所属的安装目录。
        if (Path(__file__).resolve().parent/'.installation.json').is_file():prefix=Path(__file__).resolve().parent
        uninstall(prefix,a.data_dir);return
    if platform.machine() not in ['x86_64','amd64']:raise RuntimeError('本安装包适用于 x86_64/amd64 Ubuntu 桌面电脑。')
    if a.install_deps:
        subprocess.run(['sudo','apt-get','update'],check=True)
        subprocess.run(['sudo','apt-get','install','-y']+DEPS,check=True)
    missing=dependency_errors()
    if missing:raise RuntimeError('缺少运行依赖：'+', '.join(missing)+'\n请安装：sudo apt-get install '+' '.join(DEPS)+'\n也可重新运行安装包并加 --install-deps。')
    desktop=a.desktop_dir
    if not a.no_desktop and desktop is None and shutil.which('xdg-user-dir'):
        d=subprocess.check_output(['xdg-user-dir','DESKTOP'],text=True).strip()
        if d and Path(d)!=Path.home():desktop=Path(d)
    install(a.source,prefix,a.applications_dir,None if a.no_desktop else desktop)

if __name__=='__main__':
    try:main()
    except (RuntimeError,OSError,subprocess.CalledProcessError) as exc:print('安装/卸载未完成：'+str(exc),file=sys.stderr);sys.exit(1)
