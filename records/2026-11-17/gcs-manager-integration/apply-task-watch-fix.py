#!/usr/bin/env python3
"""只安装已核对摘要的任务监视退出修复；保留原文件权限/所有者。"""
import hashlib
import json
import os
from pathlib import Path
import stat
import tempfile

target = Path('/home/orangepi/drone_platform/01_apps/mission_demo/tools/gcs_task_manager.py')
source = Path('/tmp/gcs-manager-api-check/platform/01_apps/mission_demo/tools/gcs_task_manager.py')
old_sha = 'd0dcf18ccf10bddb427f39716400fd4889cb28ec48da3ec771f077279d2c0ea0'
new_sha = '43caf322c15d9ba9b4f325796c3c1d7cfe843729207be43f6a143eb88f78ffef'
assert os.geteuid() == 0
assert hashlib.sha256(target.read_bytes()).hexdigest() == old_sha
data = source.read_bytes()
assert hashlib.sha256(data).hexdigest() == new_sha
compile(data, str(target), 'exec')
info = target.stat()
fd, name = tempfile.mkstemp(prefix='.task-watch-fix-', dir=target.parent)
try:
    with os.fdopen(fd, 'wb') as output:
        output.write(data)
        os.fchown(output.fileno(), info.st_uid, info.st_gid)
        os.fchmod(output.fileno(), stat.S_IMODE(info.st_mode))
        output.flush()
        os.fsync(output.fileno())
    os.replace(name, target)
    fd = os.open(str(target.parent), os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)
finally:
    if os.path.exists(name):
        os.unlink(name)
print(json.dumps(dict(installed=True, target=str(target), old_sha256=old_sha,
                      new_sha256=hashlib.sha256(target.read_bytes()).hexdigest())))
