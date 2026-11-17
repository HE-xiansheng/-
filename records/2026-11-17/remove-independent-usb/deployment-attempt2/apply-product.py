import hashlib,json,shutil,subprocess
from pathlib import Path
root=Path('/tmp/remove-independent-usb-ros2');product=Path('/home/orangepi/drone_platform')
manifest=json.loads((root/'deployment-manifest.json').read_text())
def git(*args):return subprocess.check_output(['git','-C',str(product),*args],text=True).strip()
assert git('rev-parse','HEAD')=='e35a2f5c8915a20369842fc82725f7171d0235fe'
assert not git('status','--porcelain')
for name,digest in manifest['changed'].items():assert hashlib.sha256((root/'product'/name).read_bytes()).hexdigest()==digest,name
assert not (product/'02_drivers/usb_camera').is_symlink()
for name in manifest['changed']:
 target=product/name;staged=target.with_name(target.name+'.usb-removal-stage')
 assert not staged.exists()
 shutil.copy2(root/'product'/name,staged)
 staged.replace(target)
shutil.rmtree(product/'02_drivers/usb_camera')
(product/'03_tools/config script/start_usb_camera.sh').unlink()
paths=list(manifest['changed'])+manifest['deleted']
subprocess.run(['git','-C',str(product),'add','--',*paths],check=True)
subprocess.run(['git','-C',str(product),'-c','user.name=Migration','-c','user.email=migration@localhost','commit','-m','Remove independent USB camera ROS module and startup dependency'],check=True,stdout=subprocess.DEVNULL)
assert not git('status','--porcelain')
print(json.dumps({'head':git('rev-parse','HEAD'),'clean':True,'removed':manifest['deleted'],'changed':manifest['changed']},indent=2))
