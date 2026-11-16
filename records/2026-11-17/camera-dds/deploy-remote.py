from pathlib import Path
import subprocess,hashlib,tarfile,io,sys,json,os
root=Path('/home/orangepi/drone_platform');base=root/'02_drivers/orbbec'
assert subprocess.check_output(['git','-C',str(root),'rev-parse','--short','HEAD']).decode().strip()=='d861281'
assert not subprocess.check_output(['git','-C',str(root),'status','--porcelain']).strip()
assert subprocess.run(['pgrep','-x','component_conta'],capture_output=True).returncode==1
commands=subprocess.check_output(['ps','-eo','args']).decode().splitlines()
assert not any('component_container' in s or ('ros2 launch' in s and 'gemini335l.launch.py' in s) for s in commands)
launch='launch/gemini335l.launch.py';profile='config/fastdds_large_images.xml'
assert hashlib.sha256((base/launch).read_bytes()).hexdigest()=='4e037309e7a3edb347985d7804963a0c1419a064185ac1f6c2b44473ba8e405c'
assert not (base/profile).exists()
expected={launch:'1a55e223520ce0c971af06c1861fe3f0bd786636c11e658e82047aa5bda70b80',profile:'73967e234c74e6ea468ec353f403597b637f599402a5eebd9fddcc246b4a38bb'}
with tarfile.open(fileobj=io.BytesIO(sys.stdin.buffer.read())) as archive:
 data={n:archive.extractfile(n).read() for n in expected}
for n,b in data.items():assert hashlib.sha256(b).hexdigest()==expected[n]
for n,b in data.items():
 p=base/n;tmp=p.with_name(p.name+'.migration-new');tmp.write_bytes(b);os.replace(tmp,p)
for n in expected:assert hashlib.sha256((base/n).read_bytes()).hexdigest()==expected[n]
subprocess.run(['git','-C',str(root),'add','--']+['02_drivers/orbbec/'+n for n in expected],check=True)
subprocess.run(['git','-C',str(root),'-c','user.name=tyzk','-c','user.email=940833041@qq.com','commit','-m','Scope Gemini large-image DDS transport to camera launch'],check=True)
print(json.dumps({'head':subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD']).decode().strip(),'sha256':expected,'git_clean':not subprocess.check_output(['git','-C',str(root),'status','--porcelain']).strip(),'services_changed':False,'driver_binary_changed':False},indent=2))
