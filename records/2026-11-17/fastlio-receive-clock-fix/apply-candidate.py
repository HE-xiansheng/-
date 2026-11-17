#!/usr/bin/env python3
"""仅在用户确认且定位已正常停止后安装固定摘要候选；不负责停止或启动会话。"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import stat
import tempfile

root=Path('/home/orangepi/drone_platform')
candidate=Path('/tmp/fastlio-receive-clock-fix-184504-792c7c')
source=Path('01_apps/fast_lio/workspace/src/fast_lio/src/laserMapping.cpp')
header=Path('01_apps/fast_lio/workspace/src/fast_lio/include/source_clock_alignment.hpp')
binary=Path('01_apps/fast_lio/workspace/install/fast_lio/lib/fast_lio/fastlio_mapping')
files={
    Path('01_apps/fast_lio/workspace/src/fast_lio/src/laserMapping.cpp'): ('source/fast_lio/src/laserMapping.cpp', '2c3f080533595144f0494d2df70efae00f24c2ee41a8ccce443b2b3a54963e9d', '323b7c5cbed2f95c16ec4e650a8639891161ca97bc11c18b43bd86b0dab87961'),
    Path('01_apps/fast_lio/workspace/src/fast_lio/include/source_clock_alignment.hpp'): ('source/fast_lio/include/source_clock_alignment.hpp', None, '0c4d759277d651e06cd5a2ce09b74dbb6162ff502734689522df2a203b2a96d8'),
    Path('01_apps/fast_lio/workspace/src/fast_lio/CMakeLists.txt'): ('source/fast_lio/CMakeLists.txt', '311a153175fad04adb8ce55b2a1b45ec9f556add64318a40b3e547912ea8e806', '3b8df3ef3c1e0fee062d50178edd9716d5b8c4d163f1e3fa2e9f42cef2513f72'),
    Path('01_apps/fast_lio/config/lidar.yaml'): ('config/lidar.yaml', '447aebf3ef449da3c37c6bf0a6bdcc824c083a8d112b94e99ebf29cdfb2d3b85', '3a83b8b2663c513a669aee662ea545a4ab49e49b2da1982d36c7f41973da63a6'),
    Path('01_apps/fast_lio/workspace/install/fast_lio/lib/fast_lio/fastlio_mapping'): ('install/lib/fast_lio/fastlio_mapping', '7707f011a357aa241b11eb098ce86fdbe6e9a73de762c36a960db6937810cbcd', '17af7094f3191be589c8f04132ee86e46035f6f6fc7b51c7f25ad698c9750a97'),
}

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def replace(path,data,mode):
    fd,name=tempfile.mkstemp(prefix='.source-clock-',dir=path.parent)
    try:
        with os.fdopen(fd,'wb') as output:
            output.write(data);os.fchmod(output.fileno(),mode);output.flush();os.fsync(output.fileno())
        os.replace(name,path)
        fd=os.open(str(path.parent),os.O_DIRECTORY)
        try:os.fsync(fd)
        finally:os.close(fd)
    finally:
        if os.path.exists(name):os.unlink(name)

spec=importlib.util.spec_from_file_location('clock_deploy_gate',root/'01_apps/fast_lio/manage_stack.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
import rclpy
rclpy.init(args=[]);api=module.ExistingStack()
try:
    with module.locks():
        api.gate();api.require_idle()
        import subprocess
        assert subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()=='7dd15a5be69148d4b124583f843d4bdfbe49fe46'
        assert not subprocess.check_output(['git','-C',str(root),'status','--porcelain'],text=True).strip(), 'deployed source tree changed'
        assert api.inventory()==(False,False),'stop owned localization and relay first'
        for p in Path('/proc').glob('[0-9]*/exe'):
            try:
                executable=p.resolve(strict=True)
                assert executable.name!='fastlio_mapping','another FAST-LIO process is still running'
            except (FileNotFoundError,PermissionError):pass
        for target,(name,old,new) in files.items():
            assert digest(candidate/name)==new,'candidate changed: '+name
            assert digest(root/target)==old if old else not (root/target).exists(), 'old runtime changed: '+str(target)
        original={target:((root/target).read_bytes(),stat.S_IMODE((root/target).stat().st_mode)) if (root/target).exists() else None for target in files}
        changed=[]
        try:
            for target,(name,old,new) in files.items():
                mode=original[target][1] if original[target] else 0o644
                replace(root/target,(candidate/name).read_bytes(),mode);changed.append(target)
                assert digest(root/target)==new
        except BaseException:
            for target in reversed(changed):
                if original[target]:replace(root/target,*original[target])
                else:(root/target).unlink()
            raise
        print(json.dumps(dict(installed=True,files={str(path):digest(root/path) for path in files},
                              parameters_changed={'common.imu_queue_depth':400},services_changed=False,session_started=False)))
finally:
    api.close();rclpy.try_shutdown()
