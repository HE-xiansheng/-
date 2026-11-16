#!/usr/bin/env python3
"""Actual RKNN detector -> native controller -> MAVROS -> disarmed isolated PX4, synthetic sensors."""
import os,time,sys,json,math,signal,subprocess,struct
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='87' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import cv2,numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped,TwistStamped
from mavros_msgs.msg import State,RCIn,ExtendedState,PositionTarget,Mavlink
from mavros_msgs.srv import SetMode,CommandBool
from yolo_ultra_test.msg import TrackedTarget
out=Path(sys.argv[1]);processes=[];logs=[];actual_states=[];sink=[];image_stamps=set();phase_ranges=[];completed=False
rclpy.init();node=Node('person_chain_fixture');bridge=CvBridge();commands=[];targets=[];calls=[];results=[]
node.create_subscription(PositionTarget,'/mavros/setpoint_raw/local',lambda m:commands.append((time.monotonic(),m)),100)
node.create_subscription(State,'/mavros/state',actual_states.append,qos_profile_sensor_data)
def observe_sink(m):
 if m.msgid==84:
  payload=b''.join(struct.pack('<Q',int(v)) for v in m.payload64)
  sink.append(dict(time_boot_ms=struct.unpack_from('<I',payload)[0],received_monotonic=time.monotonic()))
node.create_subscription(Mavlink,'/uas1/mavlink_sink',observe_sink,QoSProfile(depth=500,reliability=ReliabilityPolicy.BEST_EFFORT))

node.create_subscription(TrackedTarget,'/Safina_track_stamped',lambda m:targets.append((time.monotonic(),m)),20)
def service(req,res):calls.append(type(req).__name__);return res
node.create_service(SetMode,'/fixture/set_mode',service);node.create_service(CommandBool,'/fixture/arming',service)
pubs={k:node.create_publisher(t,name,2) for k,t,name in [
 ('pose',PoseStamped,'/fixture/pose'),('vel',TwistStamped,'/fixture/velocity'),
 ('state',State,'/fixture/state'),('rc',RCIn,'/fixture/rc'),('landed',ExtendedState,'/fixture/landed'),
 ('rgb',Image,'/camera/color/image_raw'),('depth',Image,'/camera/depth/image_raw')]}
bus=cv2.resize(cv2.imread(str(out/'bus.jpg')),(1280,720));blank=np.zeros_like(bus)
v=dict(mode='POSCTL',armed=True,pwm=1500,depth=3000,rgb=bus,sendstate=True,sendrc=True,senddepth=True)
def phase(name,duration=2.,**changes):
 v.update(changes);start=time.monotonic();last_tele=last_image=0
 while time.monotonic()-start<duration:
  assert all(p.poll() is None for p in processes), 'owned child exited early'
  assert not any(m.armed for m in actual_states), 'actual PX4 armed'
  now=time.monotonic()
  if now-last_tele>.045:
   last_tele=now
   def header(m,frame):m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id=frame
   m=PoseStamped();header(m,'map');m.pose.position.x=2.;m.pose.position.y=3.;m.pose.position.z=1.;m.pose.orientation.w=1.;pubs['pose'].publish(m)
   m=TwistStamped();header(m,'map');pubs['vel'].publish(m)
   m=ExtendedState();header(m,'map');m.landed_state=2;pubs['landed'].publish(m)
   if v['sendstate']:
    m=State();header(m,'');m.connected=True;m.armed=v['armed'];m.mode=v['mode'];pubs['state'].publish(m)
   if v['sendrc']:
    m=RCIn();header(m,'');m.channels=[1500]*9;m.channels[8]=v['pwm'];pubs['rc'].publish(m)
  if now-last_image>.15:
   last_image=now
   if v['senddepth']:
    m=bridge.cv2_to_imgmsg(np.full((480,848),v['depth'],np.uint16),'16UC1');header(m,'depth_optical');pubs['depth'].publish(m)
   if v['rgb'] is not None:
    m=bridge.cv2_to_imgmsg(v['rgb'],'bgr8');header(m,'color_optical');image_stamps.add(m.header.stamp.sec*10**9+m.header.stamp.nanosec);pubs['rgb'].publish(m)
  rclpy.spin_once(node,timeout_sec=.005)
 tail=time.monotonic()-.5;a=[m for t,m in commands if t>tail];b=[m for t,m in targets if t>tail]
 phase_ranges.append(dict(name=name,start_index=next((i for i,(t,_) in enumerate(commands) if t>=start),len(commands)),end_index=len(commands),mode=v['mode'],armed=v['armed'],pwm=v['pwm']))
 print('PHASE',name,'commands',len(a),'targets',len(b),flush=True);return a,b

def check(name,ok):results.append(dict(check=name,passed=bool(ok)));print(json.dumps(results[-1]),flush=True)
def body(m):return m.coordinate_frame==m.FRAME_BODY_NED
def hold(a):return bool(a) and all(m.coordinate_frame==m.FRAME_LOCAL_NED and abs(m.position.x-2)<.01 and abs(m.position.y-3)<.01 for m in a)

def start(name,args):
 log=(out/(name+'.log')).open('wb');logs.append(log)
 p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
mavros=None
try:
 mavros=start('mavros',['ros2','launch',str(out/'mavros_platform.launch.py'),'fcu_url:=udp://127.0.0.1:14640@127.0.0.1:14641'])
 deadline=time.monotonic()+25
 while time.monotonic()<deadline and not (actual_states and actual_states[-1].connected):
  assert mavros.poll() is None
  rclpy.spin_once(node,timeout_sec=.05)
 check('actual PX4 connects disarmed',bool(actual_states) and actual_states[-1].connected and not actual_states[-1].armed)
 assert results[-1]['passed']
 install=Path('/home/orangepi/drone_platform/01_apps/mission_demo/workspace/install')
 for pkg,exe,cfg in [('yolo_ultra_test','Safina_yoloUltra_test','yolo_track.yaml'),('track_test','Safina_track_test','track.yaml')]:
  executable=out/'candidate-install/lib/track_test/Safina_track_test' if pkg=='track_test' and (out/'candidate-install').exists() else install/pkg/'lib'/pkg/exe
  args=[str(executable),'--ros-args','--params-file',str(install/pkg/'share'/pkg/'config'/cfg)]
  for source,dest in {'/mavros/state': '/fixture/state', '/mavros/local_position/pose': '/fixture/pose', '/mavros/local_position/velocity_local': '/fixture/velocity', '/mavros/rc/in': '/fixture/rc', '/mavros/extended_state': '/fixture/landed', '/mavros/set_mode': '/fixture/set_mode', '/mavros/cmd/arming': '/fixture/arming'}.items():args+=['-r',source+':='+dest]
  start(pkg,args)
 a,b=phase('POSCTL',duration=3.);check('POSCTL prestream holds and detector session inactive',hold(a) and bool(b) and all(not m.session_active for m in b))
 a,b=phase('follow positive error',mode='OFFBOARD',duration=3.);check('actual NPU target reaches controller and forward speed',bool(b) and any(m.state==0 and m.distance_source==2 and abs(m.distance_m-3)<.03 for m in b) and bool(a) and all(body(m) and m.velocity.x>.28 for m in a))
 check('actual NPU horizontal error produces nonzero body yaw rate',bool(a) and any(abs(m.yaw_rate)>.01 for m in a) and bool(b) and any(abs(m.offset_y)>.01 for m in b))
 check('inferred horizontal error drives bounded nonzero body yaw rate',bool(a) and bool(b) and abs(b[-1].offset_y)>.01 and all(m.yaw_rate*b[-1].offset_y>0 and abs(m.yaw_rate)<=1.201 for m in a))
 a,b=phase('depth standoff',depth=2000);check('NPU depth at configured 2m gives zero forward speed',bool(a) and all(body(m) and abs(m.velocity.x)<.02 for m in a))
 a,b=phase('close target',depth=1000);check('NPU close depth drives capped reverse',bool(a) and all(body(m) and abs(m.velocity.x+.15)<.02 for m in a))
 a,b=phase('blank image',rgb=blank);check('blank image ends movement with target absent',hold(a) and bool(b) and all(m.state==-1 for m in b))
 a,b=phase('person returns',rgb=bus,depth=3000);check('returning detected person resumes movement',bool(a) and all(body(m) and m.velocity.x>.25 for m in a))
 a,b=phase('RGB dropout',rgb=None);check('RGB dropout ends detector session and controller holds',hold(a) and bool(b) and all(m.state==-1 and not m.session_active for m in b))
 a,b=phase('RGB returns',rgb=bus);check('RGB return starts new ID session',bool(b) and any(m.state==0 and m.target_id==1 for m in b))
 a,b=phase('SWA protection',pwm=1100);check('protection holds controller while preserving detector IDs',hold(a) and bool(b) and any(m.session_active and m.target_id==1 for m in b))
 a,b=phase('SWA follow',pwm=1500);check('SWA follow resumes without target replacement',bool(a) and all(body(m) and m.velocity.x>.25 for m in a))
 a,b=phase('mode exit',mode='POSCTL');check('pilot mode exit holds and resets detector session',hold(a) and bool(b) and all(not m.session_active for m in b))
 a,b=phase('disarmed OFFBOARD',mode='OFFBOARD',armed=False);check('disarmed controller ignores valid inferred target',hold(a))
 check('no mode or arming calls in tracking chain',not calls)
 check('all control messages finite',bool(commands) and all(all(math.isfinite(v) for v in [m.position.x,m.position.y,m.position.z,m.velocity.x,m.velocity.y,m.velocity.z,m.yaw_rate]) for _,m in commands))

 check('actual PX4 remains disarmed and nonoffboard',bool(actual_states) and all(not m.armed and m.mode!='OFFBOARD' for m in actual_states))
 check('actual NPU preserves RGB source timestamps',bool(targets) and all(m.header.stamp.sec*10**9+m.header.stamp.nanosec in image_stamps for _,m in targets if m.state==0))

 completed=True
finally:
 order=[p for p in reversed(processes) if p is not mavros]+([mavros] if mavros is not None else [])
 for p in order:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
  deadline=time.monotonic()+8
  while p.poll() is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
  if p.poll() is None:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3)
  until=time.monotonic()+.25
  while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
 for log in logs:log.close()
 check('owned native processes exit normally',all(p.returncode==0 for p in processes))
 record=[dict(received_monotonic=received,stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,frame=m.header.frame_id,coordinate_frame=m.coordinate_frame,type_mask=m.type_mask,position=[m.position.x,m.position.y,m.position.z],velocity=[m.velocity.x,m.velocity.y,m.velocity.z],acceleration=[m.acceleration_or_force.x,m.acceleration_or_force.y,m.acceleration_or_force.z],yaw=m.yaw,yaw_rate=m.yaw_rate) for received,m in commands]
 (out/'result.json').write_text(json.dumps(dict(passed=completed and bool(results) and all(r['passed'] for r in results),checks={r['check']:r['passed'] for r in results},commands=record,phases=phase_ranges,flight_service_calls=calls,mavros_sink_commands=sink,actual_states=[dict(armed=m.armed,mode=m.mode,connected=m.connected) for m in actual_states],targets=[dict(stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,state=m.state,offset_x=m.offset_x,offset_y=m.offset_y,target_id=m.target_id,session_active=m.session_active,distance_m=m.distance_m,distance_source=m.distance_source) for _,m in targets],returncodes=[p.returncode for p in processes]),indent=2)+'\n')
 node.destroy_node();rclpy.shutdown()
sys.exit(0 if completed and results and all(r['passed'] for r in results) else 1)
