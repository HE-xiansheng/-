#!/usr/bin/env python3
"""回收指定轮次原始证据，并与独立SSH读取的逐文件摘要比较；不清理机载。"""
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import tarfile

out = Path(__file__).resolve().parent
ssh = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=6', 'orangepi@192.168.1.224']
jobs = []
for number in (7, 8, 9):
    root = json.loads((out/('live-run%d.json'%number)).read_text())['remote_root']
    jobs.append(('live-run%d'%number, root, ['observations.json', 'result.json', 'baseline.log', 'stall_recovery.log']))
jobs.append(('replay-v3', '/tmp/fastlio-receive-clock-fix-184504-792c7c',
             ['regression-result.json']+[case+'/'+file for case in
              ['candidate_gap', 'candidate_imu_behind', 'candidate_imu_ahead']
              for file in ['node.log', 'odometry.json', 'result.json']]))
for name, root, names in jobs:
    assert root.startswith(('/tmp/fastlio-receive-clock-test-', '/tmp/fastlio-receive-clock-fix-'))
    code = 'from pathlib import Path\nimport hashlib,json\nroot=Path('+repr(root)+')\nnames='+repr(names)+'\nprint(json.dumps({n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in names},indent=2))\n'
    manifest = json.loads(subprocess.check_output(ssh+['python3 -'], input=code.encode(), timeout=30))
    (out/(name+'-remote-manifest.json')).write_text(json.dumps(manifest, indent=2)+'\n')
    archive = out/(name+'-evidence.tar.gz')
    assert not archive.exists(), 'Preserve previous collection: '+str(archive)
    with archive.open('wb') as stream:
        subprocess.run(ssh+[shlex.join(['tar', '-czf', '-', '-C', root, *names])], stdout=stream, check=True, timeout=60)
    with tarfile.open(archive) as tar:
        assert set(tar.getnames()) == set(names)
        hashes = {n:hashlib.sha256(tar.extractfile(n).read()).hexdigest() for n in names}
    assert hashes == manifest
    result = dict(verified=True, remote_root=root, files=len(names), sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
    (out/(name+'-evidence-verification.json')).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(name=name, **result)), flush=True)
