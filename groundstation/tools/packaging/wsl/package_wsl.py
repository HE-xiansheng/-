#!/usr/bin/env python3
"""打包Windows11 WSL一键配置脚本与独立Linux地面站安装文件。"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import shutil
import tarfile
import zipfile

HERE=Path(__file__).resolve().parent
SRC=HERE.parents[2]
ROOT=SRC.parent

UNINSTALL_FILES=('Common.ps1','Configure-Host.ps1','Setup-GroundStation.ps1',
                 'Uninstall-GroundStation.ps1','uninstall_in_wsl.py')

def copy_uninstaller(folder):
    folder.mkdir(parents=True,exist_ok=True)
    for name in UNINSTALL_FILES:
        if name.endswith('.ps1'):
            text=(HERE/name).read_text(encoding='utf-8-sig').replace('\r\n','\n')
            (folder/name).write_bytes(text.replace('\n','\r\n').encode('utf-8-sig'))
        else:shutil.copy2(HERE/name,folder/name)
    text=(HERE/'Uninstall-WSL-GCS.cmd').read_text().replace('\r\n','\n')
    (folder/'一键卸载.cmd').write_bytes(text.replace('\n','\r\n').encode('ascii'))
    (folder/'卸载说明.txt').write_text((HERE/'UNINSTALL.txt').read_text(),encoding='utf-8-sig')

def payload_info(path):
    raw=path.read_bytes();marker=b'__GCS_ARCHIVE_BELOW__\n'
    if marker not in raw:raise ValueError('不是地面站自解压安装包')
    archive=raw[raw.index(marker)+len(marker):]
    with tarfile.open(fileobj=io.BytesIO(archive),mode='r:gz') as tar:
        manifest_raw=tar.extractfile('research-flight-groundstation/payload-manifest.json').read()
        manifest=json.loads(manifest_raw)
    return {'schema':1,'file':path.name,'version':manifest['version'],'sha256':hashlib.sha256(raw).hexdigest(),
            'manifest_sha256':hashlib.sha256(manifest_raw).hexdigest(),'binary_sha256':manifest['files']['build/linux/customgeometry'],'distribution':'Ubuntu-20.04','architecture':'amd64'}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--payload',type=Path,required=True)
    p.add_argument('--wsl-msi',type=Path)
    p.add_argument('--ubuntu-image',type=Path)
    p.add_argument('--uninstaller-output',type=Path)
    p.add_argument('--output',type=Path,default=Path.home()/'下载/科研飞行平台-WSL一键安装包.zip')
    p.add_argument('--stage',type=Path,default=ROOT/'.planning/wsl-oneclick-20261110/delivery')
    args=p.parse_args();info=payload_info(args.payload)
    if args.wsl_msi:
        digest=hashlib.sha256(args.wsl_msi.read_bytes()).hexdigest()
        if digest!='2313efc5873e3f8552f42e77f950fabd913c0289078fa89d2100f3421a525eaa':
            raise ValueError('WSL MSI与微软官方3.0.5 x64发布摘要不一致')
        info['wsl']={'file':args.wsl_msi.name,'version':'3.0.5.0','sha256':digest,'source':'https://github.com/microsoft/WSL/releases/tag/3.0.5'}
    if args.ubuntu_image:
        # 来自Ubuntu官方SHA256SUMS，并已用Ubuntu归档密钥验证其GPG签名。
        digest=hashlib.sha256(args.ubuntu_image.read_bytes()).hexdigest()
        if digest!='a9073f3726aab9661076506603706eadf284b80a791dec3adfde8e1989906fa4':
            raise ValueError('Ubuntu镜像与官方20.04.6 WSL amd64签名摘要不一致')
        info['ubuntu']={'file':args.ubuntu_image.name,'version':'20.04.6','sha256':digest,
                        'source':'https://releases.ubuntu.com/focal/ubuntu-20.04.6-wsl-amd64.wsl'}
    folder=args.stage/'ResearchFlightGCS-WSL'
    if folder.exists():shutil.rmtree(folder)
    folder.mkdir(parents=True)
    copy_uninstaller(folder)
    for name in ['Common.ps1','Configure-Host.ps1','Setup-GroundStation.ps1']:
        text=(HERE/name).read_text(encoding='utf-8-sig').replace('\r\n','\n')
        (folder/name).write_bytes(text.replace('\n','\r\n').encode('utf-8-sig'))
    text=(HERE/'Install-WSL-GCS.cmd').read_text().replace('\r\n','\n')
    (folder/'一键安装.cmd').write_bytes(text.replace('\n','\r\n').encode('ascii'))
    shutil.copy2(HERE/'install-in-wsl.sh',folder/'install-in-wsl.sh')
    shutil.copy2(HERE/'upgrade_prepare.py',folder/'upgrade_prepare.py')
    shutil.copy2(SRC/'assets/icons/research-flight-gcs-v1.ico',folder/'groundstation.ico')
    shutil.copy2(args.payload,folder/args.payload.name)
    if args.wsl_msi:shutil.copy2(args.wsl_msi,folder/args.wsl_msi.name)
    if args.ubuntu_image:shutil.copy2(args.ubuntu_image,folder/args.ubuntu_image.name)
    (folder/'PayloadInfo.json').write_text(json.dumps(info,ensure_ascii=False,indent=2),encoding='utf-8-sig')
    (folder/'使用说明.txt').write_text((HERE/'README.txt').read_text(),encoding='utf-8-sig')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(args.output,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for file in sorted(folder.iterdir()):
            z.write(file,str(file.relative_to(args.stage)),compress_type=zipfile.ZIP_STORED if file.suffix in ('.run','.msi','.wsl') else zipfile.ZIP_DEFLATED)
    sha=hashlib.sha256(args.output.read_bytes()).hexdigest()
    args.output.with_suffix('.zip.sha256').write_text(sha+'  '+args.output.name+'\n')
    result={'bundle':str(args.output),'stage':str(folder),'size_mb':round(args.output.stat().st_size/1048576,1),'sha256':sha,'payload':info}
    if args.uninstaller_output:
        small=args.stage/'uninstall-only/ResearchFlightGCS-Uninstall'
        if small.exists():shutil.rmtree(small)
        copy_uninstaller(small)
        args.uninstaller_output.parent.mkdir(parents=True,exist_ok=True)
        with zipfile.ZipFile(args.uninstaller_output,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
            for file in sorted(small.iterdir()):z.write(file,str(file.relative_to(small.parent)))
        digest=hashlib.sha256(args.uninstaller_output.read_bytes()).hexdigest()
        args.uninstaller_output.with_suffix('.zip.sha256').write_text(digest+'  '+args.uninstaller_output.name+'\n')
        result['uninstaller']={'file':str(args.uninstaller_output),'sha256':digest,'size':args.uninstaller_output.stat().st_size,'stage':str(small)}
    print(json.dumps(result,ensure_ascii=False))

if __name__=='__main__':main()
