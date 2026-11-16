from pathlib import Path
import json,subprocess,hashlib,tarfile,io,shlex
repo=Path('/home/tyzk/drone-migration-github');work=Path('/home/tyzk/drone-migration-work')
base=work/'aruco-consumer-ready-run3'
assert json.loads((base/'analysis.json').read_text())['passed']
rel=Path('01_apps/mission_demo/workspace/src/land_test')
names=['CMakeLists.txt','package.xml','include/land_test/Safina_land_test.h','src/Safina_land_test.cpp']
new={str(rel/name):(repo/'platform'/rel/name).read_bytes() for name in names}
old={path:subprocess.check_output(['git','-C',str(repo),'show','HEAD:platform/'+path]) for path in new}
with tarfile.open(base/'inputs.tar') as archive:
    for name in names:assert archive.extractfile('candidate-src/'+name).read()==new[str(rel/name)]
new['01_apps/mission_demo/workspace/install/land_test/lib/land_test/Safina_land_test']=(base/'aruco-mavros-wire/candidate-install/lib/land_test/Safina_land_test').read_bytes()
new['01_apps/mission_demo/workspace/install/land_test/share/land_test/package.xml']=new[str(rel/'package.xml')]
sha=lambda b:hashlib.sha256(b).hexdigest()
expected={p:sha(b) for p,b in old.items()}
expected['01_apps/mission_demo/workspace/install/land_test/lib/land_test/Safina_land_test']='ca44b93128497bca0cdaae1b9755b830366a179081c3512d90c3992cd385003a'
expected['01_apps/mission_demo/workspace/install/land_test/share/land_test/package.xml']=sha(old[str(rel/'package.xml')])
manifest={p:sha(b) for p,b in new.items()}
payload=io.BytesIO()
with tarfile.open(fileobj=payload,mode='w') as archive:
    for name,data in new.items():
        member=tarfile.TarInfo(name);member.size=len(data);archive.addfile(member,io.BytesIO(data))
code='''from pathlib import Path
import io,sys,tarfile,hashlib,subprocess,os,json
root=Path('/home/orangepi/drone_platform')
assert not subprocess.check_output(['git','-C',str(root),'status','--porcelain'])
assert subprocess.check_output(['git','-C',str(root),'rev-parse','--short','HEAD']).decode().strip()=='5b1d6b7'
assert not {'Safina_land_test','Safina_land_tes'} & set(subprocess.check_output(['ps','-eo','comm=']).decode().splitlines()), 'landing controller still running'
expected=EXPECTED
manifest=MANIFEST
old={};modes={}
for name,digest in expected.items():
    p=root/name;assert not p.is_symlink(), name+' is a symlink'
    old[name]=p.read_bytes();modes[name]=p.stat().st_mode
    assert hashlib.sha256(old[name]).hexdigest()==digest,name+' old bytes changed'
with tarfile.open(fileobj=io.BytesIO(sys.stdin.buffer.read()),mode='r') as archive:
    assert set(archive.getnames())==set(expected)
    data={name:archive.extractfile(name).read() for name in expected}
for name in data:assert hashlib.sha256(data[name]).hexdigest()==manifest[name]
def replace(name,blob):
    p=root/name;tmp=p.with_suffix(p.suffix+'.candidate-next');tmp.write_bytes(blob);os.chmod(tmp,modes[name]);os.replace(tmp,p)
try:
    for name,blob in data.items():replace(name,blob)
    for name in data:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==manifest[name]
except BaseException:
    for name,blob in old.items():replace(name,blob)
    raise
subprocess.run(['git','-C',str(root),'add']+TRACKED,check=True)
subprocess.run(['git','-C',str(root),'-c','user.name=tyzk','-c','user.email=940833041@qq.com','commit','-m','Gate landing startup on actual MAVROS consumer and router readiness'],check=True)
print(json.dumps(dict(head=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD']).decode().strip(),sha256=manifest,status=subprocess.check_output(['git','-C',str(root),'status','--porcelain']).decode()),indent=2))
'''.replace('EXPECTED',repr(expected)).replace('MANIFEST',repr(manifest)).replace('TRACKED',repr(list(old)))
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
r=subprocess.run(ssh+['python3 -c '+shlex.quote(code)],input=payload.getvalue(),capture_output=True,check=True)
record=repo/'records/2026-11-17/aruco-consumer-ready'
(record/'deployment.txt').write_bytes(r.stdout+r.stderr)
(record/'deployment-manifest.json').write_text(json.dumps(dict(old_sha256=expected,new_sha256=manifest),indent=2)+'\n')
print(r.stdout.decode())
