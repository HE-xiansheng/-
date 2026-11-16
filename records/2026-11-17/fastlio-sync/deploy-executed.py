"""Deploy the tested candidate while localization is stopped; run over SSH stdin."""
from pathlib import Path
import hashlib,os,shutil,subprocess,json
root=Path('/home/orangepi/drone_platform')
source='01_apps/fast_lio/workspace/src/fast_lio/src/laserMapping.cpp'
binary='01_apps/fast_lio/workspace/install/fast_lio/lib/fast_lio/fastlio_mapping'
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert subprocess.check_output(['git','-C',str(root),'rev-parse','--short','HEAD']).decode().strip()=='623e8d5'
assert not subprocess.check_output(['git','-C',str(root),'status','--porcelain']).strip()
assert subprocess.run(['pgrep','-f',r'/lib/fast_lio/fastlio_mapping( |$)'],capture_output=True).returncode==1
old={source:'837e484a8dd06f1065079b89443723d2f3c0f1f483171616f234b82a6c31044b',binary:'04cda48cae7220bafe733dfaa5dd45b5776fa0b3d1f583c9dfa91ec774f0b8c8'}
candidate={source:Path('/tmp/fastlio-sync-candidate/src/src/laserMapping.cpp'),binary:Path('/tmp/fastlio-sync-candidate/install/lib/fast_lio/fastlio_mapping')}
new={source:'2c3f080533595144f0494d2df70efae00f24c2ee41a8ccce443b2b3a54963e9d',binary:'7707f011a357aa241b11eb098ce86fdbe6e9a73de762c36a960db6937810cbcd'}
for name in old:
 assert digest(root/name)==old[name],name+' unexpected old content'
 assert digest(candidate[name])==new[name],name+' unexpected candidate'
for name in old:
 target=root/name;temporary=target.with_name(target.name+'.migration-new')
 shutil.copy2(candidate[name],temporary);os.replace(temporary,target)
 assert digest(target)==new[name]
subprocess.run(['git','-C',str(root),'add','--',source],check=True)
subprocess.run(['git','-C',str(root),'-c','user.name=tyzk','-c','user.email=940833041@qq.com','commit','-m','Fix FAST-LIO self-sync IMU seconds conversion'],check=True)
print(json.dumps({'head':subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD']).decode().strip(),'old':old,'new':new,'git_clean':not subprocess.check_output(['git','-C',str(root),'status','--porcelain']).strip(),'services_changed':False,'physical_fc_used':False},indent=2))
candidate=Path('/tmp/fastlio-sync-candidate')
old={source:'837e484a8dd06f1065079b89443723d2f3c0f1f483171616f234b82a6c31044b',binary:'04cda48cae7220bafe733dfaa5dd45b5776fa0b3d1f583c9dfa91ec774f0b8c8'}
new={source:'2c3f080533595144f0494d2df70efae00f24c2ee41a8ccce443b2b3a54963e9d',binary:'7707f011a357aa241b11eb098ce86fdbe6e9a73de762c36a960db6937810cbcd'}
inputs={source:candidate/'src/src/laserMapping.cpp',binary:candidate/'install/lib/fast_lio/fastlio_mapping'}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert subprocess.check_output(['git','-C',str(root),'rev-parse','--short','HEAD']).decode().strip()=='623e8d5'
assert not subprocess.check_output(['git','-C',str(root),'status','--porcelain']).strip()
assert subprocess.run(['pgrep','-f','^.*/lib/fast_lio/fastlio_mapping( |$)'],capture_output=True).returncode==1
for name in inputs:
 assert sha(root/name)==old[name],name
 assert sha(inputs[name])==new[name],name
for name in inputs:
 target=root/name;temp=target.with_name(target.name+'.candidate')
 shutil.copy2(inputs[name],temp);os.replace(temp,target)
 assert sha(target)==new[name]
subprocess.run(['git','-C',str(root),'add','--',source,binary],check=True)
subprocess.run(['git','-C',str(root),'-c','user.name=tyzk','-c','user.email=940833041@qq.com','commit','-m','Fix FAST-LIO self-sync IMU seconds conversion'],check=True)
print(json.dumps(dict(head=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD']).decode().strip(),old_sha256=old,new_sha256={name:sha(root/name) for name in inputs},services_changed=False)))
