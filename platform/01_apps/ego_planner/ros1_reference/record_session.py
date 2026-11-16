#!/usr/bin/env python3
"""Passive, supervised rosbag recorder; never publishes commands or calls flight services."""
import argparse
import hashlib
import json
import os
import platform
from pathlib import Path
import shutil
import signal
import subprocess
import tarfile
import time
import xmlrpc.client

TOPICS = [
 '/factory_fastlio/cloud_world', '/factory_fastlio/odometry',
 '/ego_planner/navigation/preview_goal', '/ego_planner/navigation/confirm',
 '/mavros/vision_pose/pose', '/goal', '/move_base_simple/goal',
 '/mandatory_stop_to_planner', '/ego_planner/flight_execution', '/ego_planner/manual_preview/status',
 '/ego_planner/manual_preview/trajectory', '/ego_planner/diagnostics/events',
 '/ego_planner/px4_execution/status', '/mavros/state', '/mavros/extended_state',
 '/mavros/estimator_status', '/mavros/local_position/pose',
 '/mavros/local_position/odom', '/mavros/local_position/velocity_local',
 '/mavros/imu/data', '/mavros/rc/in', '/mavros/battery',
 '/mavros/setpoint_raw/local', '/mavros/setpoint_raw/target_local',
 '/mavros/timesync_status', '/mavros/statustext/recv', '/tf', '/tf_static', '/rosout',
 '/drone_0_ego_planner_node/grid_map/occupancy',
 '/drone_0_ego_planner_node/grid_map/occupancy_inflate']

def atomic(path, value):
 tmp = path.with_suffix('.tmp'); tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2)); tmp.replace(path)

def execution_alive(root):
 try:
  info=json.loads((root/'.px4-execution.json').read_text())
  stat=Path('/proc/%d/stat'%info['pid']).read_text().split(') ',1)[1].split()
  cmd=Path('/proc/%d/cmdline'%info['pid']).read_bytes().replace(b'\0',b' ').decode()
  alive=stat[0]!='Z' and stat[19]==info['start_tick'] and str(root/'px4_execution/adapter.py') in cmd
  if not alive:return False
  # A persistent adapter waiting in pilot mode must not prevent a requested bag stop.
  # Unknown/stale status stays conservative: preserve the hold/fault tail.
  try:
   status=json.loads((Path(info['run'])/'status.json').read_text())
   if 0<=time.time()-status['stamp']<=4 and status.get('mode') not in (None,'OFFBOARD'):
    return False
  except (OSError,ValueError,KeyError,TypeError):pass
  return True
 except (OSError,ValueError,KeyError,IndexError):return False

def archive(root, run):
 files = list((root/'src').rglob('*'))
 files += list((root/'px4_execution').glob('*.py')) + list(root.glob('*.py')) + list(root.glob('*.sh'))
 files += list((root/'devel/lib').glob('lib*.so'))
 files += list((root/'devel/lib/ego_planner').glob('planner_node'))
 manifest = {}
 with tarfile.open(str(run/'sources.tgz'), 'w:gz',dereference=True) as tar:
  for p in files:
   if not p.is_file() or '__pycache__' in p.parts or '.git' in p.parts: continue
   name = str(p.relative_to(root))
   manifest[name] = hashlib.sha256(p.read_bytes()).hexdigest()
   tar.add(str(p), arcname=name, recursive=False)
 atomic(run/'manifest.json', {'schema':1,'started_wall':time.time(),
   'started_monotonic':time.monotonic(),'platform':platform.platform(),'architecture':platform.machine(),
   'master':os.environ.get('ROS_MASTER_URI'), 'topics':TOPICS, 'sha256':manifest,
   'limitations':['ROS bag receipt timing is not exact callback scheduling.',
     'Bag recording starts/stops only on explicit operator requests; unlatched messages before the request are absent.',
     'Map snapshots restore collision queries, not all transient mapping caches.',
     'Recorder health and final bag inventory must be checked before claiming complete recording.']})
 with (run/'runtime.txt').open('w') as runtime:
  for cmd in [['uname','-a'],['rosversion','-d'],['rosversion','roscpp'],['rosversion','mavros'],
    ['ldd',str(root/'devel/lib/ego_planner/planner_node')],
    ['dpkg-query','-W','libeigen3-dev','libpcl-dev','ros-noetic-rosbag']]:
   runtime.write('\n$ '+' '.join(cmd)+'\n');runtime.flush()
   try:subprocess.run(cmd,stdout=runtime,stderr=subprocess.STDOUT,timeout=10)
   except (OSError,subprocess.TimeoutExpired) as exc:runtime.write(str(exc)+'\n')

def subscriptions(node):
 master = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI','http://127.0.0.1:11311'))
 code, _, uri = master.lookupNode('/flight_record_supervisor', node)
 if code != 1: return set()
 code, _, values = xmlrpc.client.ServerProxy(uri).getSubscriptions('/flight_record_supervisor')
 return {v[0] for v in values} if code == 1 else set()

class FlightTrigger:
 """Read-only MAVROS observations. A trigger requires two fresh source messages."""
 def __init__(self):
  self.state=None;self.landed=None

 def on_state(self,msg):
  self.state=(time.monotonic(),msg.header.stamp.to_sec(),bool(msg.connected),bool(msg.armed),msg.mode)

 def on_landed(self,msg):
  self.landed=(time.monotonic(),msg.header.stamp.to_sec(),msg.landed_state)

 def check(self,ros_now):
  now=time.monotonic();state=self.state;landed=self.landed
  data={'connected':state[2] if state else False,'armed':state[3] if state else False,
    'mode':state[4] if state else None,'landed_state':landed[2] if landed else None,
    'state_stamp':state[1] if state else None,'landed_stamp':landed[1] if landed else None}
  fresh=all(m is not None and m[1]>0 and -.1<=ros_now-m[1]<=2 and 0<=now-m[0]<=2
    for m in (state,landed))
  data['fresh']=fresh
  # MAVROS ExtendedState.LANDED_STATE_IN_AIR == 2; UNKNOWN/ON_GROUND do not qualify.
  return bool(fresh and state[2] and state[3] and state[4]=='OFFBOARD' and landed[2]==2),data

def read_control(run):
 try:
  value=json.loads((run/'recording-control.json').read_text())
  if value.get('action') in ('start','stop') and isinstance(value.get('id'),str):return value
 except (OSError,ValueError):pass
 return {}


def finish_bag(proc,run,status,log):
 """沿用SIGINT、active文件检查和rosbag info盘点，不影响飞控或规划。"""
 status.update(phase='STOPPING',stamp=time.time())
 atomic(run/'recording-status.json',status)
 deadline=time.monotonic()+25
 if proc and proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=20)
  except subprocess.TimeoutExpired:raise RuntimeError('rosbag did not finalize in 20 seconds')
 while list(run.glob('*.active')) and time.monotonic()<deadline:time.sleep(.1)
 if list(run.glob('*.active')):raise RuntimeError('active bag remains after finalization; inspect recording')
 inventory=[]
 for bag in sorted(run.glob('*.bag')):
  info=bag.with_name(bag.name+'.info.yaml')
  result=subprocess.run(['rosbag','info','--yaml',str(bag)],stdout=subprocess.PIPE,stderr=log,timeout=30)
  info.write_bytes(result.stdout)
  if result.returncode:raise RuntimeError('rosbag info failed: '+bag.name)
  inventory.append({'file':bag.name,'bytes':bag.stat().st_size,'info_ok':True})
 status['bag_bytes']=sum(b['bytes'] for b in inventory if b['file'].startswith('inputs-'+status.get('clip_id','')))
 status.update(phase='STOPPED',bags=inventory,active_bags=[],bag_pid=None,returncode=proc.poll() if proc else None,stamp=time.time())
 atomic(run/'recording-status.json',status)
 if status.get('clip_id'):atomic(run/('segment-'+status['clip_id']+'.json'),dict(status))


def main():
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
 p.add_argument('--root',type=Path,default=Path(__file__).resolve().parent)
 a=p.parse_args();run=a.run;run.mkdir(parents=True,exist_ok=True)
 # 状态归档与规划诊断仍复用原会话目录；录包触发只取显式请求，与模式无关。
 stopping=[False]
 for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stopping.__setitem__(0,True))
 status={'schema':2,'policy':'manual','phase':'PREPARING','stamp':time.time(),'run':str(run),
         'duration_s':0.,'bag_bytes':0,'bag_pid':None,'trigger_policy':'explicit operator start/stop only'}
 atomic(run/'recording-status.json',status)
 proc=None;bag_started=None;last_request=None
 import rospy
 from mavros_msgs.msg import State,ExtendedState
 rospy.init_node('ego_planner_manual_record_supervisor',anonymous=True,disable_signals=True,disable_rosout=True)
 observed=FlightTrigger()
 subscribers=[rospy.Subscriber('/mavros/state',State,observed.on_state,queue_size=1),
              rospy.Subscriber('/mavros/extended_state',ExtendedState,observed.on_landed,queue_size=1)]
 node='/ego_planner_flight_bag_'+str(os.getpid())
 with (run/'rosbag.log').open('ab') as log:
  try:
   if shutil.disk_usage(run).free<2*1024**3:raise RuntimeError('free disk below 2 GiB')
   archive(a.root,run)
   with (run/'params-before.yaml').open('w') as out:
    subprocess.run(['rosparam','dump','/dev/stdout'],stdout=out,stderr=log,timeout=15,check=True)
   status.update(phase='WAIT_MANUAL',ready_before_planner=True)
   atomic(run/'recording-status.json',status)
   while not stopping[0] and not rospy.is_shutdown():
    try:
     command=read_control(run);new_request=command.get('id') and command['id']!=last_request
     if new_request:
      last_request=command['id'];status['request_id']=last_request
     if new_request and command['action']=='stop':
      if proc:
       status['duration_s']=time.monotonic()-bag_started
       finish_bag(proc,run,status,log);proc=None;bag_started=None
      else:status.update(phase='STOPPED',bag_pid=None)
     if new_request and command['action']=='start' and proc is None:
      if shutil.disk_usage(run).free<2*1024**3:raise RuntimeError('free disk below 2 GiB')
      if list(run.glob('*.active')):raise RuntimeError('unfinalized active bag exists; inspect it before recording again')
      status.pop('error',None);status.pop('active_bags',None)
      clip=command['id'];status.update(phase='STARTING_BAG',clip_id=clip,duration_s=0.,bag_bytes=0,bag_pid=None,
          requested_wall=command.get('stamp'),recording_started_wall=time.time())
      atomic(run/'recording-trigger.json',dict(status));atomic(run/'recording-status.json',status)
      with (run/('params-start-'+clip+'.yaml')).open('w') as out:
       subprocess.run(['rosparam','dump','/dev/stdout'],stdout=out,stderr=log,timeout=15,check=True)
      proc=subprocess.Popen(['rosbag','record','--lz4','--split','--repeat-latched','--size=256','--buffsize=128',
           '--min-space=2G','-O',str(run/('inputs-'+clip)),*TOPICS,'__name:='+node[1:]],
           stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
      bag_started=time.monotonic();status.update(bag_pid=proc.pid,recording_started_wall=time.time())
      ready=False;deadline=time.monotonic()+12
      while proc.poll() is None and time.monotonic()<deadline and not stopping[0]:
       control=read_control(run)
       if control.get('action')=='stop' and control.get('id')!=last_request:break
       try:ready=set(TOPICS).issubset(subscriptions(node))
       except (OSError,xmlrpc.client.Error):pass
       if ready:break
       status.update(stamp=time.time(),duration_s=time.monotonic()-bag_started)
       atomic(run/'recording-status.json',status);time.sleep(.1)
      if not ready:
       control=read_control(run)
       if stopping[0] or (control.get('action')=='stop' and control.get('id')!=last_request):
        status['duration_s']=time.monotonic()-bag_started;finish_bag(proc,run,status,log);proc=None;bag_started=None
        if control.get('id'):last_request=control['id'];status['request_id']=last_request
       else:raise RuntimeError('rosbag subscriptions not ready in 12 seconds')
      else:status.update(phase='RECORDING')
     if proc and proc.poll() is not None:raise RuntimeError('rosbag exited unexpectedly: '+str(proc.returncode))
     free=shutil.disk_usage(run).free
     if proc and free<2*1024**3:raise RuntimeError('free disk below 2 GiB; recording stopped, no flight actions')
     _,flight=observed.check(rospy.Time.now().to_sec())  # 只显示飞行观测，绝不用于触发。
     if proc:status['duration_s']=time.monotonic()-bag_started
     prefix='inputs-'+status.get('clip_id','')
     status.update(stamp=time.time(),free_bytes=free,flight_observed=flight,
                   bag_bytes=sum(x.stat().st_size for x in run.glob(prefix+'*.bag*') if not x.name.endswith('.yaml')),
                   stop_waiting_for_execution=False)
     with (run/'rosbag.log').open('rb') as reader:
      reader.seek(max(0,(run/'rosbag.log').stat().st_size-20000));text=reader.read().decode(errors='replace')
     status['warnings']=[x for x in text.splitlines() if any(k in x.lower() for k in ('buffer exceeded','dropping','insufficient','error','exception'))][-10:]
     atomic(run/'recording-status.json',status)
    except Exception as exc:
     reason=str(exc)
     if proc:
      status['duration_s']=time.monotonic()-bag_started
      try:finish_bag(proc,run,status,log)
      except Exception as final:reason+='; '+str(final)
      if proc.poll() is not None:proc=None;bag_started=None
     status.update(phase='ERROR',error=reason,stamp=time.time(),bag_pid=proc.pid if proc and proc.poll() is None else None,
                   active_bags=[x.name for x in run.glob('*.active')])
     atomic(run/'recording-status.json',status)
    time.sleep(.2)
  except Exception as exc:
   status.update(phase='ERROR',error=str(exc),stamp=time.time());atomic(run/'recording-status.json',status)
  finally:
   if proc:
    status['duration_s']=time.monotonic()-bag_started
    try:finish_bag(proc,run,status,log)
    except Exception as exc:status.update(phase='ERROR',error=str(exc),stamp=time.time());atomic(run/'recording-status.json',status)
   for sub in subscribers:sub.unregister()


if __name__=='__main__':main()
