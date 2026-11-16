#!/usr/bin/env python3
"""安装已在隔离ROS验证的ARM构建；只检查地面状态，不发布飞控或目标指令。"""
import hashlib,json,os,shutil,subprocess,tempfile,time
from pathlib import Path
import rospy
from mavros_msgs.msg import State,ExtendedState
ws=Path.home()/'drone_platform/01_apps/ego_planner';stage=ws/'.automatic-navigation-build'
manifest=json.loads((stage/'install-manifest.json').read_text())

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def ground():
 s=rospy.wait_for_message('/mavros/state',State,timeout=5)
 e=rospy.wait_for_message('/mavros/extended_state',ExtendedState,timeout=5)
 now=rospy.Time.now().to_sec()
 if not(s.connected and not s.armed and s.mode!='OFFBOARD' and e.landed_state==1 and -.1<=now-s.header.stamp.to_sec()<=2 and -.1<=now-e.header.stamp.to_sec()<=2):
  raise RuntimeError('当前不是新鲜的未解锁、在地面状态；未安装')
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:a=p.joinpath('cmdline').read_bytes().split(b'\0')
  except OSError:continue
  if not a:continue
  if a[0]==os.fsencode(ws/'devel/lib/diff_planner/diff_planner_node') or os.fsencode(ws/'px4_execution/adapter.py') in a:
   raise RuntimeError('规划器或执行器正在运行；未安装')
def atomic(src,dst):
 dst.parent.mkdir(parents=True,exist_ok=True)
 fd,tmp=tempfile.mkstemp(prefix='.'+dst.name+'.auto-',dir=str(dst.parent));os.close(fd)
 try:
  shutil.copy2(src,tmp)
  if dst.exists():os.chmod(tmp,dst.stat().st_mode)
  with open(tmp,'rb') as f:os.fsync(f.fileno())
  os.replace(tmp,dst)
 finally:
  if os.path.exists(tmp):os.unlink(tmp)
rospy.init_node('gcs_auto_navigation_install_check',anonymous=True,disable_rosout=True)
for f in manifest['files']:
 dst=ws/f['path'];src=stage/f['staged']
 if sha(dst)!=f['expected_old'] or sha(src)!=f['sha256']:raise RuntimeError('基线或构建哈希不匹配：'+f['path'])
for name in ['ros-validation.json','adapter-validation.json']:
 if not json.loads((stage/name).read_text()).get('passed'):raise RuntimeError('机载隔离验证未通过')
ground()
backup=Path.home()/'drone_platform/05_文档资料/恢复备份/ego_planner'/('automatic-navigation-'+time.strftime('%Y%m%d-%H%M%S'));backup.mkdir(parents=True)
for f in manifest['files']:
 dst=backup/f['path'];dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ws/f['path'],dst)
(backup/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
changed=[]
try:
 ground()
 for f in manifest['files']:
  atomic(stage/f['staged'],ws/f['path']);changed.append(f)
 for f in manifest['files']:
  if sha(ws/f['path'])!=f['sha256']:raise RuntimeError('安装校验失败：'+f['path'])
 for name in ['diff_planner_node','candidate_preview_worker']:
  r=subprocess.run(['ldd',str(ws/'devel/lib/diff_planner'/name)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
  if r.returncode or 'not found' in r.stdout:raise RuntimeError('安装后动态库缺失：'+r.stdout[-1000:])
 ground()
except Exception:
 for f in reversed(changed):atomic(backup/f['path'],ws/f['path'])
 raise
receipt={'installed':True,'backup':str(backup),'files':{f['path']:sha(ws/f['path']) for f in manifest['files']},'flight_commands_sent':False,'time':time.time()}
(stage/'installed.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2));print(json.dumps(receipt,ensure_ascii=False))
