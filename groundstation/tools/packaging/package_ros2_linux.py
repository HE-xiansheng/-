#!/usr/bin/env python3
"""使用当前CMake安装规则生成独立ROS2地面站包，不改动已安装桌面入口。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import tarfile


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--version',required=True)
    parser.add_argument('--cmake',default='cmake')
    parser.add_argument('--extra-runtime',type=Path,help='已解包的Ubuntu补充运行库根目录，可含usr/lib与usr/share/doc')
    a=parser.parse_args()
    if platform.system()!='Linux' or platform.machine() not in ('x86_64','amd64'):
        parser.error('当前打包入口仅验证Linux amd64')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*',a.version):parser.error('version须为普通版本标识')
    build=a.build_dir.resolve();output=a.output.resolve()
    name='drone-gcs-ros2-'+a.version+'-linux-amd64'
    bundle=output/name;archive=output/(name+'.tar.gz')
    if bundle.exists() or archive.exists():parser.error('输出版本已存在，请使用新目录或新版本')
    output.mkdir(parents=True,exist_ok=True)
    subprocess.run([a.cmake,'--install',str(build),'--prefix',str(bundle)],check=True)
    here=Path(__file__).resolve().parent
    # 仅对当前进程设置依赖路径；第三方系统库继续由目标Ubuntu提供。
    launcher=bundle/'start-groundstation.sh'
    launcher.write_text('''#!/usr/bin/env bash
set -euo pipefail
GCS_BUNDLE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
unset QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH QML_IMPORT_PATH QML2_IMPORT_PATH
export LD_LIBRARY_PATH="$GCS_BUNDLE/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export QML_IMPORT_PATH="$GCS_BUNDLE/qml"
export QT_PLUGIN_PATH="$GCS_BUNDLE/plugins"
export PATH="$GCS_BUNDLE/bin:$PATH"
exec "$GCS_BUNDLE/bin/customgeometry" "$@"
''');launcher.chmod(0o755)
    # 保留原有云台RTSP代理绕过逻辑，不安装整个历史desktop目录。
    shutil.copy2(here.parent/'desktop/video-bin/gst-launch-1.0',bundle/'bin/gst-launch-1.0')
    (bundle/'bin/gst-launch-1.0').chmod(0o755)
    shutil.copytree(here/'licenses',bundle/'licenses')
    if a.extra_runtime:
        runtime=a.extra_runtime.resolve()
        libraries=runtime/'usr/lib/x86_64-linux-gnu'
        for source in libraries.glob('*.so*'):
            dest=bundle/'lib'/source.name
            if dest.exists() or dest.is_symlink():raise RuntimeError('补充库与部署内容重名: '+source.name)
            if source.is_symlink():dest.symlink_to(os.readlink(source))
            else:shutil.copy2(source,dest)
        if (runtime/'usr/share/doc').is_dir():shutil.copytree(runtime/'usr/share/doc',bundle/'licenses/ubuntu-runtime')
    (bundle/'README.md').write_text('''# 科研飞行平台 ROS2 地面站独立包

解压后运行 `./start-groundstation.sh`，可从任意工作目录启动。
不会注册桌面快捷方式，不会替换现有地面站；删除解压目录即可移除此包。
连接配置及日志使用独立应用标识和用户数据目录drone-gcs-ros2。
可设置GCS_STATE_DIR指定桥接进程状态目录。关闭程序后再删除包。

本产物基于Ubuntu20.04 amd64桌面构建；仅在当前电脑验证，其他系统需实测。
包含动态Qt运行库/QML/插件、两个桌面程序及ROS2桥接运行工具。
本机不需要ROS，机载需要Ubuntu22.04/Humble及已适配的drone_platform。
系统依赖包括Python3、numpy、OpenSSH、setsid、LZ4、bzip2、SQLite、
OpenGL/X11及GStreamer(H264解码和RTSP)。缺包时按系统实际缺项安装。
例如：python3-numpy openssh-client liblz4-1 libbz2-1.0 libsqlite3-0
以及gstreamer1.0-tools、plugins-base/good/bad、libav、fonts-noto-cjk。

启动程序不会自动解锁、起飞或启动机载任务。物理任务链路仍须现场验收。
旧ROS1安装器及开发测试不随包发布。完整迁移证据见GitHub仓库records。
Qt/ICU动态库可单独替换，授权条款和对应源码链接见licenses目录。
''',encoding='utf-8')
    entries={}
    for p in sorted(bundle.rglob('*')):
        relative=p.relative_to(bundle).as_posix()
        if p.is_symlink():
            if os.path.commonpath([str(p.resolve()),str(bundle)])!=str(bundle):
                raise RuntimeError('包中存在指向外部的符号链接: '+relative)
            entries[relative]={'symlink':os.readlink(p)}
        elif p.is_file():entries[relative]={'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'mode':oct(p.stat().st_mode&0o777)}
    manifest={'version':a.version,'architecture':'amd64','build_dir':str(build),'files':entries}
    (bundle/'payload-manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    with tarfile.open(archive,'w:gz') as tar:tar.add(bundle,arcname=name)
    sha=hashlib.sha256(archive.read_bytes()).hexdigest()
    (output/(name+'.sha256')).write_text(sha+'  '+archive.name+'\n')
    print(json.dumps({'bundle':str(bundle),'archive':str(archive),'sha256':sha},ensure_ascii=False))


if __name__=='__main__':main()
