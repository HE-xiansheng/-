#!/usr/bin/env python3
"""收集 Windows Release 构建与运行依赖，生成用户级安装 EXE 与便携 ZIP。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import struct
import zipfile

SRC = Path(__file__).resolve().parents[2]
ROOT = SRC.parent


def validate_executable(file):
    data=file.read_bytes()
    if len(data)<128 or data[:2]!=b'MZ': raise ValueError('无效Windows程序：'+str(file))
    pe=struct.unpack_from('<I',data,60)[0]
    if data[pe:pe+4]!=b'PE\0\0' or struct.unpack_from('<H',data,pe+4)[0]!=0x8664:
        raise ValueError('需要64位Windows PE程序：'+str(file))
    count=struct.unpack_from('<H',data,pe+6)[0];optional=struct.unpack_from('<H',data,pe+20)[0]
    image_size=struct.unpack_from('<I',data,pe+24+56)[0]
    for index in range(count):
        offset=pe+24+optional+40*index
        size,addr=struct.unpack_from('<II',data,offset+8)
        if addr+size>image_size:raise ValueError('PE节超出映像边界，请重新链接并移除调试节：'+str(file))


def copy_runtime(source, target):
    shutil.copytree(source, target, ignore=shutil.ignore_patterns(
        '__pycache__', '*.pyc', 'test*', '*.pdb', '*.a', '*.cpp', '*.h', '*.cmake', 'CMakeFiles', 'sqldrivers'))
    # Qt 的 debug DLL 不进入 Release 包。
    for file in target.rglob('*d.dll'):
        if file.with_name(file.name[:-5] + '.dll').exists(): file.unlink()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--version', default='1.0.20261109')
    p.add_argument('--output', type=Path, default=Path.home()/'下载/科研飞行平台-地面站Windows安装包')
    p.add_argument('--qt', type=Path, default=ROOT/'.deps/windows-qt')
    p.add_argument('--mingw-runtime', type=Path, required=True)
    p.add_argument('--makensis', type=Path, default=ROOT/'.deps/windows-cross/sysroot/usr/bin/makensis')
    args = p.parse_args(); output = args.output.resolve(); output.mkdir(parents=True, exist_ok=True)
    stage = output/('.stage-' + args.version)
    if stage.exists(): shutil.rmtree(stage)
    bundle = stage/'ResearchFlightGroundStation'; binary = bundle/'bin'; binary.mkdir(parents=True)
    for name in ['customgeometry.exe', 'radarsimulator.exe', 'ssh-askpass.exe']:
        validate_executable(ROOT/'build/windows'/name)
        shutil.copy2(ROOT/'build/windows'/name, binary/name)
    for file in args.qt.joinpath('bin').glob('*.dll'):
        if file.stem.endswith('d') and file.with_name(file.name[:-5]+'.dll').exists(): continue
        shutil.copy2(file, binary/file.name)
    for name in ['libstdc++-6.dll', 'libgcc_s_seh-1.dll', 'libwinpthread-1.dll']:
        shutil.copy2(args.mingw_runtime/name, binary/name)
    shutil.copy2(ROOT/'.deps/windows-cross/sysroot/usr/lib/gcc/x86_64-w64-mingw32/10-posix/libssp-0.dll', binary/'libssp-0.dll')
    for name in ['plugins', 'qml']: copy_runtime(args.qt/name, binary/name)
    (binary/'qt.conf').write_text('[Paths]\nPrefix=.\nPlugins=plugins\nQmlImports=qml\n')
    for name in ['preflight', 'pointcloud-only', 'communication']:
        copy_runtime(SRC/'tools'/name, bundle/'tools'/name)
    copy_runtime(ROOT/'.deps/windows-assets/python-runtime', bundle/'runtime/python')
    assets = ROOT/'.deps/windows-assets'
    openssh = bundle/'runtime/openssh'; openssh.mkdir(parents=True)
    with zipfile.ZipFile(assets/'OpenSSH-Win64.zip') as archive:
        for item in archive.infolist():
            name = Path(item.filename).name
            # 仅包含客户端与依赖，安装包不会安装 SSH 服务。
            if name in ['ssh.exe', 'ssh-keygen.exe', 'libcrypto.dll', 'LICENSE.txt', 'ThirdPartyNotices.txt']:
                (openssh/name).write_bytes(archive.read(item))
    ffmpeg = bundle/'runtime/ffmpeg'; ffmpeg.mkdir(parents=True)
    with zipfile.ZipFile(assets/'ffmpeg-release-essentials.zip') as archive:
        for item in archive.infolist():
            if Path(item.filename).name in ['ffmpeg.exe', 'LICENSE', 'README.txt']:
                (ffmpeg/Path(item.filename).name).write_bytes(archive.read(item))
    licenses = bundle/'licenses'; copy_runtime(SRC/'tools/packaging/licenses', licenses)
    shutil.copy2(ROOT/'.deps/windows-cross/codecs/lz4-1.9.4/lib/LICENSE', licenses/'LZ4-LICENSE.txt')
    shutil.copy2(ROOT/'.deps/windows-cross/codecs/bzip2-1.0.8/LICENSE', licenses/'BZIP2-LICENSE.txt')
    icon = SRC/'assets/icons/research-flight-gcs-v1.ico'; resources = bundle/'resources'; resources.mkdir()
    shutil.copy2(icon, resources/'groundstation.ico')
    instructions = ('科研飞行平台-地面站 Windows '+args.version+'\n'
        '适用：Windows 10/11，64位。安装到当前用户目录，无需Qt或Python开发环境。\n'
        '双击安装程序，完成后从桌面启动。便携版请完整解压，在bin目录运行customgeometry.exe。\n'
        '在“机载通信”填写IP（默认192.168.1.235）、用户名orangepi对应的密码，再点击“应用并连接”。\n'
        '首次连接使用SSH主机身份登记，指纹变化时仍需核对设备；密码不写入配置或日志。\n'
        'Windows防火墙首次询问时，允许程序及随包Python在专用网络通信；电脑与机载须在同一可达局域网。\n'
        '启动参数已由桌面入口设置。直接运行时默认监听点云9000、位姿9001及话题监视9003，栅格9004、原始遥测9005。\n'
        '机载端需已有匹配的drone_platform运行接口；此安装包不会安装或启动机载算法、解锁或切换飞行模式。\n'
        '飞行记录可读取机载分包或打开本地.bag；支持未压缩、LZ4与BZ2。\n'
        '主程序已编译，未附带完整工程源码、密码、SSH私钥、录包或现场配置；包含运行必需的通信辅助脚本。\n'
        '卸载可从Windows应用设置或安装目录Uninstall.exe进行；日志复制到本机应用数据目录后保留。\n'
        '本包完成的检查及Windows实机验证范围见验证结果文件；兼容层检查不能替代实飞验收。\n')
    (bundle/'使用说明.txt').write_text(instructions, encoding='utf-8-sig')
    (bundle/'THIRD_PARTY_NOTICES.txt').write_text(
        'Qt 6.8.3 dynamically linked: https://download.qt.io/archive/qt/6.8/6.8.3/single/\n'
        'MinGW-w64 GCC runtime 13.1: https://gcc.gnu.org/onlinedocs/libstdc++/manual/license.html\n'
        'Python 3.11.9: https://www.python.org/downloads/release/python-3119/\n'
        'NumPy 1.26.4: https://pypi.org/project/numpy/1.26.4/\n'
        'LZ4 4.3.3 Python bindings: https://pypi.org/project/lz4/4.3.3/\n'
        'Windows OpenSSH v9.8.3.0p2: https://github.com/PowerShell/Win32-OpenSSH/releases/tag/v9.8.3.0p2-Preview\n'
        'FFmpeg essentials build: https://www.gyan.dev/ffmpeg/builds/ (GPL; source links in runtime/ffmpeg/README.txt)\n'
        'LZ4 C 1.9.4: https://github.com/lz4/lz4/tree/v1.9.4\n'
        'bzip2 1.0.8: https://sourceware.org/bzip2/\n'
        'Licenses are included in licenses/ and corresponding runtime subdirectories.\n')
    manifest = {'version':args.version, 'architecture':'amd64', 'qt':'6.8.3', 'files':{
        str(file.relative_to(bundle)):hashlib.sha256(file.read_bytes()).hexdigest()
        for file in sorted(bundle.rglob('*')) if file.is_file()}}
    (bundle/'payload-manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
    portable = output/('ResearchFlightGroundStation-'+args.version+'-windows-x64.zip')
    with zipfile.ZipFile(portable, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for file in sorted(bundle.rglob('*')):
            if file.is_file(): archive.write(file, str(file.relative_to(stage)))
    installer = output/('科研飞行平台-地面站_'+args.version+'_windows_x64_Setup.exe')
    env = dict(os.environ, NSISDIR=str(ROOT/'.deps/windows-cross/sysroot/usr/share/nsis'))
    subprocess.run([str(args.makensis), '-INPUTCHARSET', 'UTF8', '-DBUNDLE='+str(bundle),
                    '-DOUTPUT='+str(installer), '-DVERSION='+args.version,
                    str(SRC/'tools/packaging/windows_installer.nsi')], env=env, check=True)
    (output/'安装说明.txt').write_text(instructions, encoding='utf-8-sig')
    (output/'SHA256SUMS').write_text(''.join(hashlib.sha256(file.read_bytes()).hexdigest()+'  '+file.name+'\n'
                                           for file in [installer, portable]))
    print(json.dumps({'installer':str(installer), 'portable_archive':str(portable),
                     'size_mb':round(installer.stat().st_size/1024**2,1), 'bundle':str(bundle)}, ensure_ascii=False))


if __name__ == '__main__': main()
