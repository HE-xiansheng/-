#!/usr/bin/env python3
"""ROS2 precision-landing integration with synthetic markers and fake PX4 only."""
import os,time,sys,json,math,csv
assert os.environ.get('ROS_DOMAIN_ID')=='87' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseStamped,TwistStamped
from mavros_msgs.msg import State,ExtendedState,PositionTarget
from mavros_msgs.srv import SetMode,CommandBool
from std_srvs.srv import SetBool
from std_msgs.msg import String
from nav_msgs.msg import Path
rclpy.init();node=Node('land_fake_px4');rows=[];headers=[];commands=[];accepted=[];paths=[];calls=[];arms=[];checks=[]
retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String,'/aruco_landing/status_header',lambda m:headers.append(m.data),retained)
def status(m):
 if headers:rows.append((time.monotonic(),dict(zip(next(csv.reader([headers[-1]])),next(csv.reader([m.data]))))))
node.create_subscription(String,'/aruco_landing/status',status,retained)
node.create_subscription(PositionTarget,'/mavros/setpoint_raw/local',lambda m:commands.append((time.monotonic(),m)),100)
node.create_subscription(PoseStamped,'/aruco_landing/accepted_target',lambda m:accepted.append((time.monotonic(),m)),10)
node.create_subscription(Path,'/aruco_landing/guidance_path',lambda m:paths.append((time.monotonic(),m)),10)
v=dict(mode='POSCTL',armed=True,landed=2,x=2.,y=3.,z=1.5,yaw=0.,mx=0.,my=0.,depth=None,
 marker=True,markerage=0.,markerframe='gimbal_camera_optical_frame',poseframe='map',poseage=0.,
 sendstate=True,sendpose=True,sendvel=True,sendlanded=True,stateage=0.,integrate=False)
def mode(req,res):
 calls.append(req.custom_mode);res.mode_sent=len(calls)>1
 if res.mode_sent:v['mode']=req.custom_mode
 return res
def arm(req,res):arms.append(req.value);res.success=False;return res
node.create_service(SetMode,'/mavros/set_mode',mode);node.create_service(CommandBool,'/mavros/cmd/arming',arm)
client=node.create_client(SetBool,'/aruco_landing/enable')
pubs={key:node.create_publisher(typ,topic,2) for key,typ,topic in [
 ('state',State,'/mavros/state'),('pose',PoseStamped,'/mavros/local_position/pose'),
 ('vel',TwistStamped,'/mavros/local_position/velocity_local'),('landed',ExtendedState,'/mavros/extended_state'),
 ('marker',PoseStamped,'/aruco_poseresult')]}

def phase(label,duration=1.,**changes):
 v.update(changes);start=last=time.monotonic();sent=0.
 while time.monotonic()-start<duration:
  now=time.monotonic();dt=now-last;last=now
  cmd=commands[-1][1] if commands and now-commands[-1][0]<.15 else None
  if v['integrate'] and cmd is not None and v['mode']=='OFFBOARD':
   v['x']+=cmd.velocity.x*dt;v['y']+=cmd.velocity.y*dt;v['z']=max(.005,v['z']+cmd.velocity.z*dt)
  if now-sent>.04:
   sent=now
   def header(m,frame,age=0.):m.header.stamp=rclpy.time.Time(nanoseconds=max(0,node.get_clock().now().nanoseconds-int(age*1e9))).to_msg();m.header.frame_id=frame
   if v['sendstate']:
    m=State();header(m,'',v['stateage']);m.connected=True;m.armed=v['armed'];m.mode=v['mode'];pubs['state'].publish(m)
   if v['sendpose']:
    m=PoseStamped();header(m,v['poseframe'],v['poseage']);m.pose.position.x=v['x'];m.pose.position.y=v['y'];m.pose.position.z=v['z'];m.pose.orientation.z=math.sin(v['yaw']/2);m.pose.orientation.w=math.cos(v['yaw']/2);pubs['pose'].publish(m)
   if v['sendvel']:
    m=TwistStamped();header(m,v['poseframe']);
    if v['integrate'] and cmd is not None:m.twist.linear.x=cmd.velocity.x;m.twist.linear.y=cmd.velocity.y;m.twist.linear.z=cmd.velocity.z
    pubs['vel'].publish(m)
   if v['sendlanded']:
    m=ExtendedState();header(m,'');m.landed_state=v['landed'];pubs['landed'].publish(m)
   if v['marker']:
    m=PoseStamped();header(m,v['markerframe'],v['markerage']);m.pose.position.x=v['mx'];m.pose.position.y=v['my'];m.pose.position.z=v['z'] if v['depth'] is None else v['depth'];m.pose.orientation.w=1.;pubs['marker'].publish(m)
  rclpy.spin_once(node,timeout_sec=.005)
 tail=time.monotonic()-.3;a=[m for t,m in commands if t>tail];b=[r for t,r in rows if t>tail]
 print('PHASE',label,'commands',len(a),'phase',b[-1].get('phase') if b else None,'marker',b[-1].get('marker_reason') if b else None,flush=True)
 return a,b

def check(label,ok):checks.append(dict(check=label,passed=bool(ok)));print(json.dumps(checks[-1]),flush=True)
def service(value):
 req=SetBool.Request();req.data=value;future=client.call_async(req)
 end=time.monotonic()+2
 while not future.done() and time.monotonic()<end:phase('service wait',.1)
 return future.result() if future.done() else None

a,b=phase('healthy standby',2.);check('POSCTL prestream zero with task disabled',bool(a) and all(abs(m.velocity.x)+abs(m.velocity.y)+abs(m.velocity.z)+abs(m.yaw_rate)<1e-8 for m in a) and bool(b) and all(r['enabled']=='0' for r in b));check('retained diagnostics schema available',bool(headers) and 'marker_pair_dt' in headers[-1]);check('no service writes before trigger',not calls and not arms)
res=service(True);check('PX4 mode workflow refuses service enable',res is not None and not res.success)
# A competing graph endpoint has no data; its presence must block standby.
rogue=node.create_publisher(PoseStamped,'/mavros/setpoint_position/local',1)
a,b=phase('competing publisher',1.9);check('competing controller graph blocks standby',not a)
node.destroy_publisher(rogue);phase('conflict removed',1.7)
a,b=phase('90 degree optical transform',yaw=math.pi/2,mx=.2,my=-.3,duration=1.1)
recent=[m for t,m in accepted if t>time.monotonic()-.4]
check('optical offsets rotate into ENU using paired aircraft yaw',bool(recent) and all(abs(m.pose.position.x-2.2)<.02 and abs(m.pose.position.y-3.3)<.02 for m in recent))
a,b=phase('bad marker frame',markerframe='wrong');check('wrong marker frame rejected',bool(b) and all(r['marker_reason']=='unexpected_marker_frame' for r in b))
a,b=phase('stale marker',markerframe='gimbal_camera_optical_frame',markerage=3.);check('stale marker source rejected',bool(b) and all(r['target_valid']=='0' for r in b))
a,b=phase('future marker',markerage=-3.);check('future marker source rejected',bool(b) and all(r['marker_reason']=='source_stamp_stale' for r in b))
a,b=phase('negative marker',markerage=0.,depth=-1.);check('detector no-target sentinel rejected',bool(b) and all(r['marker_reason']=='detector_no_selected_target' for r in b))
phase('reset target filter',marker=False,duration=1.)
phase('ready centered',marker=True,depth=None,yaw=0.,mx=0.,my=0.,duration=1.)
a,b=phase('manual OFFBOARD takeover',mode='OFFBOARD',duration=2.2);check('mode edge starts native landing task',any(r['enabled']=='1' for r in b));check('airborne task never asks arm or OFFBOARD',not arms and 'OFFBOARD' not in calls)
a,b=phase('marker lost',marker=False,duration=1.2);check('marker loss pauses descent and enters lost hold',bool(b) and b[-1]['phase'] in ('LOST_HOLD','ACQUIRE') and bool(a) and all(abs(m.velocity.z)<.03 for m in a))
a,b=phase('marker reacquired',marker=True,duration=1.5);check('marker return resumes alignment or descent',bool(b) and b[-1]['phase'] in ('TRANSLATE','STABILIZE','DESCEND','TERMINAL_ALIGN','TERMINAL_DESCEND'))
a,b=phase('pilot cancels',mode='POSCTL',duration=.8);check('pilot mode exit cancels active task',bool(b) and all(r['enabled']=='0' for r in b));check('cancel clears guidance path',bool(paths) and len(paths[-1][1].poses)==0)
a,b=phase('stale state',stateage=3.,duration=.6);check('stale FC source stamp blocks standby',not a)
phase('state restored ready',stateage=0.,duration=1.6)
a,b=phase('changed pose frame',poseframe='wrong',duration=.7);check('wrong local pose frame blocks standby',not a)
phase('fresh pose ready',poseframe='map',duration=1.6)
phase('second takeover',mode='OFFBOARD',duration=2.)
a,b=phase('controlled simulated descent',integrate=True,duration=16.)
check('native pipeline reaches AUTO.LAND handoff',len(calls)>=2 and all(c=='AUTO.LAND' for c in calls) and v['mode']=='AUTO.LAND')
check('handoff rejects then retries without arming',len(calls)==2 and not arms)
a,b=phase('landed confirmation',armed=False,landed=1,integrate=False,duration=.7);check('AUTO.LAND confirms landed/disarmed completion',bool(b) and b[-1]['phase']=='COMPLETE' and not a)
check('full control stream ENU velocity and finite bounded values',bool(commands) and all(m.coordinate_frame==m.FRAME_LOCAL_NED and all(math.isfinite(x) for x in [m.velocity.x,m.velocity.y,m.velocity.z,m.yaw_rate]) and math.hypot(m.velocity.x,m.velocity.y)<=.151 and abs(m.velocity.z)<=.281 and abs(m.yaw_rate)<=.101 for _,m in commands))
print(json.dumps(dict(passed=sum(r['passed'] for r in checks),total=len(checks),results=checks)),flush=True)
node.destroy_node();rclpy.shutdown();sys.exit(0 if all(r['passed'] for r in checks) else 1)
