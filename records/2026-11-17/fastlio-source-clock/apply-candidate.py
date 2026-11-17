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
candidate=Path('/tmp/fastlio-source-clock-candidate')
source=Path('01_apps/fast_lio/workspace/src/fast_lio/src/laserMapping.cpp')
header=Path('01_apps/fast_lio/workspace/src/fast_lio/include/source_clock_alignment.hpp')
binary=Path('01_apps/fast_lio/workspace/install/fast_lio/lib/fast_lio/fastlio_mapping')
files={source:('source/fast_lio/src/laserMapping.cpp','2c3f080533595144f0494d2df70efae00f24c2ee41a8ccce443b2b3a54963e9d','1bd1cb006c51b1409b343d4c14a483dd80819a0daa288257ba3f35eb8f029686'),
       header:('source/fast_lio/include/source_clock_alignment.hpp',None,'0c4d759277d651e06cd5a2ce09b74dbb6162ff502734689522df2a203b2a96d8'),
       binary:('install/lib/fast_lio/fastlio_mapping','7707f011a357aa241b11eb098ce86fdbe6e9a73de762c36a960db6937810cbcd','26a503fb47180dc777271e0128348a5e5a91f4cfc11e435c530b34dbc66ac611')}

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
                              parameters_changed=False,services_changed=False,session_started=False)))
finally:
    api.close();rclpy.try_shutdown()
