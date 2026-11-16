#!/usr/bin/env python3
"""Native waypoint+photo with fake PX4, localhost domain87; never real flight."""
import os,time,json,subprocess,signal,math,sys
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='87' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import yaml,rclpy
from rclpy.node import Node
from mavros_msgs.msg import State,RCIn,ExtendedState,PositionTarget
from mavros_msgs.srv import SetMode,CommandBool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_prefix,get_package_share_directory
rclpy.init(); node=Node('waypoint_fake_px4')
root=Path('/tmp/waypoint-verification');root.mkdir(exist_ok=True)
bindir=Path(get_package_prefix('waypoint_mission'))/'lib/waypoint_mission'
outputs=[];all_outputs=[];statuses=[];calls=[];photo_results=[];triggers=[];results=[];procs=[];logs=[]
def check(name,value):
 results.append(dict(name=name,passed=bool(value)));print(('PASS ' if value else 'FAIL ')+name,flush=True)
def output(m):
 outputs.append((time.monotonic(),m));all_outputs.append(m)
node.create_subscription(PositionTarget,'/mavros/setpoint_raw/local',output,100)
node.create_subscription(String,'/waypoint_mission/status',lambda m:statuses.append((time.monotonic(),m.data)),100)
node.create_subscription(String,'/waypoint_mission/photo_done',lambda m:photo_results.append(m.data),10)
node.create_subscription(String,'/waypoint_mission/photo_trigger',lambda m:triggers.append(m.data),10)
v={}
delayed=[]
async def mode(req,res):
 if v.get("delay_mode",False):
  calls.append(("mode",req.custom_mode))
  future=rclpy.task.Future();delayed.append(future);await future
  res.mode_sent=False;return res
 calls.append(('mode',req.custom_mode));res.mode_sent=v.get('accept',True)
 if res.mode_sent:v['mode']=req.custom_mode
 return res
def arm(req,res):
 calls.append(('arm',req.value));res.success=v.get('accept',True)
 if res.success:v['armed']=req.value
 return res
from rclpy.callback_groups import ReentrantCallbackGroup
mode_group=ReentrantCallbackGroup()
node.create_service(SetMode,'/mavros/set_mode',mode,callback_group=mode_group);node.create_service(CommandBool,'/mavros/cmd/arming',arm)
pubs={k:node.create_publisher(t,n,10) for k,t,n in [('state',State,'/mavros/state'),('rc',RCIn,'/mavros/rc/in'),('ext',ExtendedState,'/mavros/extended_state'),('pose',PoseStamped,'/mavros/local_position/pose'),('odom',Odometry,'/factory_fastlio/odometry'),('img',Image,'/rtsp_cam/image_color'),('trigger',String,'/waypoint_mission/photo_trigger')]}
def reset():
 v.clear();v.update(mode='POSCTL',armed=False,accept=True,pwm=1100,rssi=100,frame='map',poseage=0.,odomage=0.,rcage=0.,stateage=0.,sendpose=True,sendodom=True,sendrc=True,sendstate=True,sendimage=True,imageage=0.,x=0.,y=0.,z=1.,nan=False)
 outputs.clear();statuses.clear();calls.clear();photo_results.clear();triggers.clear()
def stamp(age=0.):return rclpy.time.Time(nanoseconds=max(0,node.get_clock().now().nanoseconds-int(age*1e9))).to_msg()
def pump(duration):
 start=time.monotonic();last=0.
 while time.monotonic()-start<duration:
  now=time.monotonic()
  if now-last>.025:
   last=now
   if v['sendstate']:
    m=State();m.header.stamp=stamp(v['stateage']);m.connected=True;m.armed=v['armed'];m.mode=v['mode'];pubs['state'].publish(m)
   if v['sendrc']:
    m=RCIn();m.header.stamp=stamp(v['rcage']);m.rssi=v['rssi'];m.channels=[1500]*8;m.channels[4]=v['pwm'];pubs['rc'].publish(m)
   m=ExtendedState();m.header.stamp=stamp();m.landed_state=2 if v['armed'] else 1;pubs['ext'].publish(m)
   if v['sendpose']:
    m=PoseStamped();m.header.stamp=stamp(v['poseage']);m.header.stamp.sec=-1 if v.get('negative_stamp') else m.header.stamp.sec;m.header.stamp.nanosec=1000000000 if v.get('invalid_nsec') else m.header.stamp.nanosec;m.header.frame_id=v['frame'];m.pose.orientation.w=1.;m.pose.position.x=float('nan') if v['nan'] else v['x'];m.pose.position.y=v['y'];m.pose.position.z=v['z'];pubs['pose'].publish(m)
   if v['sendodom']:
    m=Odometry();m.header.stamp=stamp(v['odomage']);m.header.frame_id='camera_init';m.child_frame_id='body';m.pose.pose.orientation.w=1.;pubs['odom'].publish(m)
   if v['sendimage']:
    m=Image();m.header.stamp=stamp(v['imageage']);m.header.frame_id='gimbal_camera_optical_frame';m.width=64;m.height=48;m.encoding='bgr8';m.step=192;m.data=bytes([90,140,200]*64*48);pubs['img'].publish(m)
  rclpy.spin_once(node,timeout_sec=.003)
def start(name,manual=False,bench=False,photo=False,land=False,point=None,frame="local",selected=False):
 reset();v['armed']=manual
 if selected:v['pwm']=1900
 cfg={'mission':dict(name=name,frame='local',manual_airborne=manual,takeoff_z=1.,state_timeout=.8,stable_seconds=.15,return_to_start=land,auto_land=land,photo_timeout=1.,gimbal_settle=0.,rc=dict(channel=5,offboard_slot=6,min=1075,trim=1494,max=1913,off_hold_seconds=.3,on_hold_seconds=.2),waypoints=[dict(name='test',x=0.,y=0.,z=1.,yaw='hold',hold=.1,photo=photo)])}
 if point is not None:cfg['mission']['waypoints'][0].update(point)
 cfg['mission']['frame']=frame
 file=root/(name+'.yaml');file.write_text(yaml.safe_dump(cfg))
 logfile=open(root/(name+'.log'),'w');logs.append(logfile)
 proc=subprocess.Popen([str(bindir/'waypoint_mission'),'--ros-args','-p','mission_file:='+str(file),'-p','bench_mode:='+str(bench).lower(),'-p','photo_dir:='+str(root/'photos'),'-p','log_dir:='+str(root/'csv'),'-r','/Odometry:=/factory_fastlio/odometry'],stdout=logfile,stderr=subprocess.STDOUT);procs.append(proc)
 if selected:
  pump(1.2)
 else:
  deadline=time.monotonic()+5.
  while time.monotonic()<deadline and not any('gate_ready=1' in text for _,text in statuses): pump(.1)
  check(name+' observes ready baseline before edge',any('gate_ready=1' in text for _,text in statuses))
 pump(.1);return proc

def stop(proc):
 proc.send_signal(signal.SIGINT)
 try:proc.wait(timeout=4.)
 except subprocess.TimeoutExpired:proc.kill();proc.wait();check('bounded exit',False)
 check('normal node exit',proc.returncode==0);procs.remove(proc);pump(.25)
def fresh_edge():v['pwm']=1900;pump(.5)
def phase_seen(name):return any('phase='+name+' ' in s for _,s in statuses)
def no_output_after(delay=.7):
 pump(delay);t=time.monotonic();pump(.5);return not any(a>t for a,_ in outputs)
try:
 if '--pending-only' not in sys.argv:
  # Parser uses original file format; malformed numeric values must fail closed.
  original=Path(get_package_share_directory('waypoint_mission'))/'missions/abc_photo.yaml'
  run=subprocess.run([str(bindir/'waypoint_mission'),'--ros-args','-p','dry_run:=true','-p','mission_file:='+str(original)],capture_output=True,text=True)
  check('source mission parses with slot6/right-forward',run.returncode==0 and 'offboard_slot 6' in run.stdout and 'x_right_y_forward' in run.stdout)
  cfg=yaml.safe_load(original.read_text());cfg['mission']['waypoints'][0]['x']=.3;cfg['mission']['waypoints'][0]['y']=.4
  for label,key,val in [('NaN hold','hold',float('nan')),('trailing yaw','yaw','20oops'),('NaN yaw','yaw','.nan')]:
   bad=yaml.safe_load(yaml.safe_dump(cfg));bad['mission']['waypoints'][0][key]=val;file=root/'invalid.yaml';file.write_text(yaml.safe_dump(bad));r=subprocess.run([str(bindir/'waypoint_mission'),'--ros-args','-p','dry_run:=true','-p','mission_file:='+str(file)],capture_output=True,text=True);check('parser rejects '+label,r.returncode==2)
  file=root/'convert.yaml';file.write_text(yaml.safe_dump(cfg));r=subprocess.run([str(bindir/'waypoint_mission'),'--ros-args','-p','dry_run:=true','-p','mission_file:='+str(file)],capture_output=True,text=True);check('right-forward converted to local ENU',r.returncode==0 and '[local 0.4, -0.3]' in r.stdout)
  # Photo process is independent of flight executor, including stale frame failures.
  log=open(root/'photo.log','w');logs.append(log);photo_proc=subprocess.Popen([str(bindir/'waypoint_photo')],stdout=log,stderr=subprocess.STDOUT);procs.append(photo_proc)
  proc=start('auto_photo_return_land',photo=True,land=True)
  check('idle no commands or requests',not outputs and not calls)
  fresh_edge();pump(4.)
  check('auto path requests OFFBOARD then ARM',calls[:2]==[('mode','OFFBOARD'),('arm',True)])
  check('native photo saved and correlated',bool(triggers) and any(s=='OK '+triggers[0]+'.jpg' for s in photo_results) and Path(triggers[0]+'.jpg').exists())
  check('return then landing requested',('mode','AUTO.LAND') in calls and phase_seen('RETURN'))
  check('AUTO.LAND stops setpoints',phase_seen('WAIT_DISARM') and no_output_after(.2))
  v['armed']=False;pump(.5);check('landing disarm reaches DONE',phase_seen('DONE'))
  check('continuous control while task active',len(outputs)>40 and max([b[0]-a[0] for a,b in zip(outputs,outputs[1:])] or [999])<.25)
  stop(proc)
  # Isolated photo freshness, future stamp, no recent receipt, and decode errors.
  for name,changes in [('stale',dict(imageage=2.)),('future',dict(imageage=-3.)),('dropout',dict(imageage=0.,sendimage=False))]:
   v.update(sendimage=True,imageage=0.);pump(.2);v.update(changes);pump(.7)
   base=str(root/name);pubs['trigger'].publish(String(data=base));pump(.3)
   check('photo rejects '+name,any(s.startswith('FAIL '+base+' ') for s in photo_results) and not Path(base+'.jpg').exists())
  v.update(sendimage=True,imageage=0.)
  proc=start('bench',bench=True);fresh_edge();pump(1.5);check('bench gate has no service or setpoint',not calls and not outputs);stop(proc)
  proc=start('manual',manual=True);fresh_edge();pump(2.8);check('manual airborne skips arm and takeoff',calls==[('mode','OFFBOARD')] and phase_seen('HOLD_COMPLETE') and not phase_seen('TAKEOFF'))
  v['pwm']=1100;check('pilot RC cancel stops outputs',no_output_after());c=len(calls);pump(1.2);check('cancel sends no late service requests',len(calls)==c and phase_seen('ABORT'));stop(proc)
  # Inputs are refreshed in receipt time but with invalid source data.
  for name,changes in [('stale_pose',dict(poseage=3.)),('future_pose',dict(poseage=-3.)),('zero_pose_stamp',dict(poseage=1e12)),('negative_pose_stamp',dict(negative_stamp=True)),('invalid_pose_nsec',dict(invalid_nsec=True)),('frame_change',dict(frame='wrong')),('nan_pose',dict(nan=True)),('stale_odom',dict(odomage=3.)),('stale_rc',dict(rcage=3.)),('rssi_zero',dict(rssi=0)),('stale_state',dict(stateage=3.))]:
   proc=start(name,manual=True);fresh_edge();pump(1.5);check(name+' first active',bool(outputs));v.update(changes);check(name+' aborts and stops',no_output_after(1.));c=len(calls);pump(.3);check(name+' no reclaim',len(calls)==c and phase_seen('ABORT'));stop(proc)
  proc=start('selected_at_start',manual=True,selected=True);pump(1.5);check('selected on startup cannot start task',not calls and not outputs);stop(proc)
  proc=start('right_forward_motion',manual=True,point=dict(x=.8,y=0.),frame='x_right_y_forward');fresh_edge();pump(2.2)
  moving=[m for _,m in outputs if m.type_mask & PositionTarget.IGNORE_PX]
  check('right maps to negative ENU y with bounded speed',bool(moving) and any(m.velocity.y<-.1 for m in moving) and all(abs(m.velocity.x)<.001 and abs(m.velocity.y)<=.601 for m in moving))
  v['mode']='POSCTL';check('pilot mode loss stops output',no_output_after());stop(proc)
  proc=start('reject_mode');v['accept']=False;fresh_edge();pump(2.5);check('mode rejection retries without arming',len(calls)>=2 and all(c==('mode','OFFBOARD') for c in calls));v['pwm']=1100;check('cancel rejected mode stops output',no_output_after());n=len(calls);pump(1.3);check('cancel clears retries',len(calls)==n);stop(proc)
  stop(photo_proc)
  check('all commands finite and ROS ENU frame',all(m.coordinate_frame==PositionTarget.FRAME_LOCAL_NED and m.header.frame_id=='map' and all(math.isfinite(x) for x in [m.position.x,m.position.y,m.position.z,m.velocity.x,m.velocity.y,m.velocity.z,m.yaw,m.yaw_rate]) for m in all_outputs))
 else:
  proc=start('pending_service');v['delay_mode']=True;fresh_edge();pump(3.8)
  check('unanswered service times out and retries',len(calls)>=2 and not any(c[0]=='arm' for c in calls))
  check('setpoints continue during unanswered service',len(outputs)>70 and max([b[0]-a[0] for a,b in zip(outputs,outputs[1:])] or [999])<.25)
  v['pwm']=1100;check('RC cancels while service pending',no_output_after())
  count=len(calls)
  for future in delayed: future.set_result(None)
  pump(1.5)
  check('late service responses cannot rearm or reclaim',len(calls)==count and phase_seen('ABORT') and not any(c[0]=='arm' for c in calls))
  stop(proc)

finally:
 for p in procs:
  if p.poll() is None:
   p.send_signal(signal.SIGINT)
   try:p.wait(timeout=3.)
   except subprocess.TimeoutExpired:p.kill();p.wait()
 for f in logs:f.close()
 (root/('results-pending.json' if '--pending-only' in sys.argv else 'results.json')).write_text(json.dumps(results,ensure_ascii=False,indent=2))
 print(json.dumps({'passed':sum(r['passed'] for r in results),'total':len(results)},ensure_ascii=False),flush=True)
 node.destroy_node();rclpy.shutdown()
sys_failed=any(not r['passed'] for r in results)
raise SystemExit(1 if sys_failed else 0)
