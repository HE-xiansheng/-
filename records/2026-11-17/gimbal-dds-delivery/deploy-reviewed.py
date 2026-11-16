from pathlib import Path
import subprocess,json,hashlib,tarfile,io,shlex
repo=Path('/home/tyzk/drone-migration-github');work=Path('/home/tyzk/drone-migration-work');record=repo/'records/2026-11-17/gimbal-dds-delivery'
base=work/'gimbal-delivery-live-run1';synthetic=work/'gimbal-delivery-candidate-run1'
for b in [base,synthetic]:
 assert json.loads((b/'runner-result.json').read_text())['remote_removed']
 result=json.loads((b/'gimbal-delivery-check/result.json').read_text())
 assert result['completed'] and len(result['rounds'])==2 and all(r['passed'] for r in result['rounds'])
rel=Path('01_apps/mission_demo/workspace/src/rtsp_test');names=['CMakeLists.txt','package.xml','src/rtsp_hw_pub_ultra.cpp','config/fastdds_large_images.xml']
new={str(rel/n):(repo/'platform'/rel/n).read_bytes() for n in names}
for b in [base,synthetic]:
 with tarfile.open(b/'inputs.tar') as a:
  for n in names:assert a.extractfile('candidate-src/'+n).read()==new[str(rel/n)]
installed=Path('01_apps/mission_demo/workspace/install/rtsp_test')
new[str(installed/'lib/rtsp_test/rtsp_hw_pub_ultra')]=(base/'gimbal-delivery-check/candidate-install/lib/rtsp_test/rtsp_hw_pub_ultra').read_bytes()
assert new[str(installed/'lib/rtsp_test/rtsp_hw_pub_ultra')]==(synthetic/'gimbal-delivery-check/candidate-install/lib/rtsp_test/rtsp_hw_pub_ultra').read_bytes()
new[str(installed/'share/rtsp_test/package.xml')]=new[str(rel/'package.xml')]
new[str(installed/'share/rtsp_test/config/fastdds_large_images.xml')]=new[str(rel/'config/fastdds_large_images.xml')]
sha=lambda data:hashlib.sha256(data).hexdigest()
expected={}
for n in names:
 path=str(rel/n)
 if n.startswith('config/'):expected[path]=None
 else:expected[path]=sha(subprocess.check_output(['git','-C',str(repo),'show','HEAD:platform/'+path]))
expected[str(installed/'lib/rtsp_test/rtsp_hw_pub_ultra')]=sha((work/'gimbal-delivery-before-deploy/rtsp_hw_pub_ultra').read_bytes())
expected[str(installed/'share/rtsp_test/package.xml')]=sha((work/'gimbal-delivery-before-deploy/package.xml').read_bytes())
expected[str(installed/'share/rtsp_test/config/fastdds_large_images.xml')]=None
manifest={n:sha(data) for n,data in new.items()}
payload=io.BytesIO()
with tarfile.open(fileobj=payload,mode='w') as a:
 for n,data in new.items():
  item=tarfile.TarInfo(n);item.size=len(data);a.addfile(item,io.BytesIO(data))
code='''from pathlib import Path
import subprocess,sys,io,tarfile,hashlib,json,os
root=Path('/home/orangepi/drone_platform')
assert not subprocess.check_output(['git','-C',str(root),'status','--porcelain'])
assert subprocess.check_output(['git','-C',str(root),'rev-parse','--short','HEAD']).decode().strip()=='73dd98c'
assert not any(c.startswith('rtsp_hw_pub') for c in subprocess.check_output(['ps','-eo','comm=']).decode().splitlines()),'MPP still active'
expected=EXPECTED
manifest=MANIFEST
old={};modes={}
for n,digest in expected.items():
 p=root/n;assert not p.is_symlink()
 if digest is None:assert not p.exists();old[n]=None;modes[n]=0o644
 else:
  old[n]=p.read_bytes();modes[n]=p.stat().st_mode;assert hashlib.sha256(old[n]).hexdigest()==digest,n
with tarfile.open(fileobj=io.BytesIO(sys.stdin.buffer.read())) as a:
 assert set(a.getnames())==set(manifest)
 data={n:a.extractfile(n).read() for n in manifest}
for n,blob in data.items():assert hashlib.sha256(blob).hexdigest()==manifest[n]
def replace(n,blob):
 p=root/n
 if blob is None:p.unlink(missing_ok=True);return
 p.parent.mkdir(parents=True,exist_ok=True);temp=p.with_suffix(p.suffix+'.candidate-next');temp.write_bytes(blob);os.chmod(temp,modes[n]);os.replace(temp,p)
try:
 for n,blob in data.items():replace(n,blob)
 for n,digest in manifest.items():assert hashlib.sha256((root/n).read_bytes()).hexdigest()==digest
except BaseException:
 for n,blob in old.items():replace(n,blob)
 raise
subprocess.run(['git','-C',str(root),'add']+TRACKED,check=True)
subprocess.run(['git','-C',str(root),'-c','user.name=tyzk','-c','user.email=940833041@qq.com','commit','-m','Use publisher scoped Fast DDS shared memory profile for MPP images'],check=True)
print(json.dumps(dict(head=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD']).decode().strip(),sha256=manifest,status=subprocess.check_output(['git','-C',str(root),'status','--porcelain']).decode()),indent=2))
'''.replace('EXPECTED',repr(expected)).replace('MANIFEST',repr(manifest)).replace('TRACKED',repr([str(rel/n) for n in names]))
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
p=subprocess.run(ssh+['python3 -c '+shlex.quote(code)],input=payload.getvalue(),capture_output=True)
(record/'deployment.txt').write_bytes(p.stdout+p.stderr)
(record/'deployment-manifest.json').write_text(json.dumps(dict(old_sha256=expected,new_sha256=manifest),indent=2)+'\n')
print(p.stdout.decode()+p.stderr.decode());p.check_returncode()
