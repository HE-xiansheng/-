#!/usr/bin/env python3
import hashlib, importlib.util, json, os, stat, subprocess, tempfile
from pathlib import Path
root = Path('/home/orangepi/drone_platform')
candidate = Path('/tmp/fastlio-receive-clock-fix-20261118-v3')
files = {
    root/'01_apps/fast_lio/workspace/src/fast_lio/src/laserMapping.cpp': (candidate/'source/fast_lio/src/laserMapping.cpp', '2c3f080533595144f0494d2df70efae00f24c2ee41a8ccce443b2b3a54963e9d', 'a29207cf657c71c0bae5bcc4264ac2a85b645532ff77cae3e8692e11a0fde2ba'),
    root/'01_apps/fast_lio/workspace/src/fast_lio/include/source_clock_alignment.hpp': (candidate/'source/fast_lio/include/source_clock_alignment.hpp', None, '0c4d759277d651e06cd5a2ce09b74dbb6162ff502734689522df2a203b2a96d8'),
    root/'01_apps/fast_lio/workspace/src/fast_lio/CMakeLists.txt': (candidate/'source/fast_lio/CMakeLists.txt', '311a153175fad04adb8ce55b2a1b45ec9f556add64318a40b3e547912ea8e806', '3b8df3ef3c1e0fee062d50178edd9716d5b8c4d163f1e3fa2e9f42cef2513f72'),
    root/'01_apps/fast_lio/config/lidar.yaml': (candidate/'config/lidar.yaml', '447aebf3ef449da3c37c6bf0a6bdcc824c083a8d112b94e99ebf29cdfb2d3b85', 'c6e24a89d3951bd0017d522b1d897ed65b9e299195bede34300fc674a1dad063'),
    root/'01_apps/fast_lio/workspace/install/fast_lio/lib/fast_lio/fastlio_mapping': (candidate/'install-v3/lib/fast_lio/fastlio_mapping', '7707f011a357aa241b11eb098ce86fdbe6e9a73de762c36a960db6937810cbcd', '06ff7402efd5030939f216d002ce6196f3827f564c92fa5e0b6ce45022e958c3'),
}
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def atomic(path,data,mode):
    fd,tmp=tempfile.mkstemp(prefix='.fastlio-v3-',dir=str(path.parent))
    try:
        with os.fdopen(fd,'wb') as f:
            f.write(data); os.fchmod(f.fileno(),mode); f.flush(); os.fsync(f.fileno())
        os.replace(tmp,path)
        d=os.open(str(path.parent),os.O_DIRECTORY)
        try: os.fsync(d)
        finally: os.close(d)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)
spec=importlib.util.spec_from_file_location('manage', root/'01_apps/fast_lio/manage_stack.py')
manage=importlib.util.module_from_spec(spec); spec.loader.exec_module(manage)
import rclpy
rclpy.init(args=[]); api=manage.ExistingStack()
try:
    with manage.locks():
        api.gate(); api.require_idle(); assert api.inventory()==(False,False)
        assert subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()=='7dd15a5be69148d4b124583f843d4bdfbe49fe46'
        assert not subprocess.check_output(['git','-C',str(root),'status','--porcelain'],text=True).strip()
        before={}
        for target,(source,old,new) in files.items():
            assert source.is_file() and sha(source)==new, str(source)
            if old is None:
                assert not target.exists(), str(target)
                before[str(target)] = None
            else:
                assert target.is_file() and sha(target)==old, str(target)
                before[str(target)]={'sha256':sha(target),'mode':stat.S_IMODE(target.stat().st_mode)}
        changed=[]
        try:
            for target,(source,old,new) in files.items():
                mode=before[str(target)]['mode'] if before[str(target)] else 0o644
                atomic(target,source.read_bytes(),mode); changed.append(target)
                assert sha(target)==new
        except BaseException:
            raise
        after={str(target):{'sha256':sha(target),'mode':stat.S_IMODE(target.stat().st_mode)} for target in files}
        print(json.dumps({'installed':True,'old':before,'new':after,'services_changed':False,'session_started':False,'candidate':'v3','binary_sha256':after[str(root/'01_apps/fast_lio/workspace/install/fast_lio/lib/fast_lio/fastlio_mapping')]['sha256']},ensure_ascii=False,indent=2),flush=True)
finally:
    api.close(); rclpy.try_shutdown()
