#!/usr/bin/env python3
"""只读取得.235原位姿转发二进制及私有测试依赖，不安装到系统或提交依赖库。"""
import subprocess,sys
from pathlib import Path
out=Path(sys.argv[1]);out.parent.mkdir(parents=True,exist_ok=True)
script=r'''
import os,re,subprocess,sys,tarfile
from pathlib import Path
binary=Path('/home/orangepi/drone_platform/02_drivers/sensors/lib/to_mavros/vision2mavros')
env=dict(os.environ,LD_LIBRARY_PATH='/opt/ros/noetic/lib')
lines=subprocess.check_output(['ldd',str(binary)],env=env,text=True)
files=[binary]
base=('libc.so','libpthread.so','libm.so','libgcc_s.so','libstdc++.so','libdl.so','librt.so','ld-linux')
for line in lines.splitlines():
 match=re.search(r'=> (/\S+)',line)
 if match:
  p=Path(match.group(1))
  if not p.name.startswith(base):files.append(p)
with tarfile.open(fileobj=sys.stdout.buffer,mode='w|gz',dereference=True) as archive:
 for p in files:archive.add(str(p),arcname=p.name,recursive=False)
'''
with out.open('wb') as target:
 subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','orangepi@192.168.1.235','python3 -'],input=script.encode(),stdout=target,check=True)
print(out,out.stat().st_size)
