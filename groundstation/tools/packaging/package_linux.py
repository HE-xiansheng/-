#!/usr/bin/env python3
"""复用已完成的 Linux 构建，收集运行依赖并生成可校验的自解压安装包。"""
import argparse,hashlib,json,os,shutil,subprocess,tarfile
from pathlib import Path

HERE=Path(__file__).resolve().parent;SRC=HERE.parents[1];ROOT=SRC.parent

def copytree(src,dst):
    shutil.copytree(src,dst,symlinks=True,ignore=shutil.ignore_patterns('__pycache__','*.pyc','test*','capture_cloud.py','*.o','*.a','CMakeFiles'))

def main():
    p=argparse.ArgumentParser();p.add_argument('--version',default='1.0.20261104');p.add_argument('--output',type=Path,default=Path.home()/'下载/科研飞行平台-地面站安装包');a=p.parse_args()
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=True);stage=out/('.stage-'+a.version)
    if stage.exists():shutil.rmtree(stage)
    bundle=stage/'research-flight-groundstation';(bundle/'build/linux').mkdir(parents=True)
    shutil.copy2(ROOT/'build/linux/customgeometry',bundle/'build/linux/customgeometry')
    shutil.copy2(ROOT/'build/linux/radarsimulator',bundle/'build/linux/radarsimulator')
    q=ROOT/'.deps/Qt';qlib=bundle/'.deps/Qt/lib';qlib.mkdir(parents=True)
    for source in q.joinpath('lib').glob('*.so*'):
        dest=qlib/source.name
        if source.is_symlink():dest.symlink_to(os.readlink(source))
        elif source.is_file():shutil.copy2(source,dest)
    for name in ['plugins','qml']:copytree(q/name,bundle/'.deps/Qt'/name)
    copytree(ROOT/'.deps/runtime',bundle/'.deps/runtime')
    for name in ['preflight','pointcloud-only','communication','desktop']:
        copytree(SRC/'tools'/name,bundle/'dianyunGCS-master/tools'/name)
    icons=bundle/'dianyunGCS-master/assets/icons';icons.mkdir(parents=True)
    shutil.copy2(SRC/'assets/icons/research-flight-gcs-v1.png',icons/'research-flight-gcs-v1.png')
    shutil.copy2(HERE/'install_user.py',bundle/'install_user.py')
    copytree(HERE/'licenses',bundle/'licenses')
    shutil.copy2(HERE/'start_bundle.sh',bundle/'start-groundstation.sh');(bundle/'start-groundstation.sh').chmod(0o755)
    (bundle/'build/linux/qt.conf').write_text('[Paths]\nPrefix=../../.deps/Qt\nLibraries=lib\nPlugins=plugins\nQmlImports=qml\n')
    (bundle/'使用说明.txt').write_text('科研飞行平台-地面站 '+a.version+'\n适用：Ubuntu 20.04 amd64 桌面。其他发行版未在本轮实机验证。\n安装：bash 科研飞行平台-地面站_'+a.version+'_ubuntu20.04_amd64.run\n如果提示缺少依赖，在同一命令末尾加 --install-deps。依赖安装需要联网和 sudo。\n安装后从桌面或应用菜单启动。机载IP在“机载通信”中填写，密码自行输入。\n默认安装到 ~/.local/opt/research-flight-groundstation；不包含现场密码、SSH密钥、机载源码、录制数据或用户连接配置。\n程序通过机载已有SSH/ROS接口工作；启动地面站不自动启动定位、规划或飞行控制。\n卸载：python3 ~/.local/opt/research-flight-groundstation/install_user.py --uninstall\n卸载后日志与通信记录保存在 ~/.local/share/research-flight-groundstation。\n')
    (bundle/'THIRD_PARTY_NOTICES.txt').write_text('Qt runtime '+subprocess.check_output([str(q/'bin/qmake'),'-query','QT_VERSION'],text=True).strip()+'\nQt: https://www.qt.io/licensing/\nQt source: https://download.qt.io/archive/qt/\nQt runtime is dynamically linked; its libraries are separately packaged and replaceable.\nICU: https://unicode-org.github.io/icu/\nSystem GStreamer, NumPy, OpenSSH, LZ4, bzip2 and OpenGL are supplied by the destination Ubuntu system.\n')
    files={str(f.relative_to(bundle)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(bundle.rglob('*')) if f.is_file() and not f.is_symlink()}
    manifest={'version':a.version,'architecture':'amd64','build_platform':'Ubuntu 20.04','qt':'6.8.3','files':files}
    (bundle/'payload-manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    archive=out/('research-flight-groundstation-'+a.version+'-ubuntu20.04-amd64.tar.gz')
    with tarfile.open(archive,'w:gz',compresslevel=6) as tar:tar.add(bundle,arcname=bundle.name)
    sha=hashlib.sha256(archive.read_bytes()).hexdigest()
    installer=out/('科研飞行平台-地面站_'+a.version+'_ubuntu20.04_amd64.run')
    with installer.open('wb') as f:f.write((HERE/'installer_header.sh').read_text().replace('@PAYLOAD_SHA256@',sha).encode());f.write(archive.read_bytes())
    installer.chmod(0o755)
    shutil.copy2(bundle/'使用说明.txt',out/'安装说明.txt')
    (out/'SHA256SUMS').write_text(''.join(hashlib.sha256(f.read_bytes()).hexdigest()+'  '+f.name+'\n' for f in [installer,archive]))
    print(json.dumps({'installer':str(installer),'portable_archive':str(archive),'size_mb':round(installer.stat().st_size/1024**2,1),'bundle':str(bundle)},ensure_ascii=False))

if __name__=='__main__':main()
