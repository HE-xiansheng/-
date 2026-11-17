from pathlib import Path
import hashlib,json,os,subprocess
root=Path('/tmp/mpp-shutdown-source-apply');product=Path('/home/orangepi/drone_platform')
m=json.loads((root/'manifest.json').read_text())
def git(*args):return subprocess.check_output(['git','-C',str(product),*args],text=True).strip()
assert git('rev-parse','HEAD')==m['expected_head']
assert not git('status','--porcelain')
for p in Path('/proc').glob('[0-9]*'):
 try:
  args=(p/'cmdline').read_bytes().split(b'\0')
  assert not (args and args[0].endswith(b'/rtsp_hw_pub_ultra')),'MPP executable still active'
 except (FileNotFoundError,PermissionError):pass
source=product/m['relative_path'];binary=product/'01_apps/mission_demo/workspace/install/rtsp_test/lib/rtsp_test/rtsp_hw_pub_ultra'
before=source.read_bytes();payload=(root/'after.cpp').read_bytes()
assert hashlib.sha256(before).hexdigest()==m['before_sha256']
assert hashlib.sha256(payload).hexdigest()==m['after_sha256']
(root/'installed-binary-before').write_bytes(binary.read_bytes())
(root/'source-before.cpp').write_bytes(before)
stage=source.with_name(source.name+'.migration-new')
with stage.open('wb') as f:f.write(payload);f.flush();os.fsync(f.fileno())
os.chmod(stage,source.stat().st_mode & 0o777);os.replace(stage,source)
print(json.dumps({'source_replaced':True,'before_binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest()}),flush=True)
