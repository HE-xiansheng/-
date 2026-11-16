#!/usr/bin/env python3
"""Actual RKNN detector -> native controller -> fake PX4, localhost domain 87."""
import os,time,sys,json,math
assert os.environ.get('ROS_DOMAIN_ID')=='87' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import cv2,numpy as np,rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped,TwistStamped
from mavros_msgs.msg import State,RCIn,ExtendedState,PositionTarget,Mavlink
from mavros_msgs.srv import SetMode,CommandBool
from yolo_ultra_test.msg import TrackedTarget
rclpy.init();node=Node('setpoint_raw',namespace='/mavros');bridge=CvBridge();commands=[];targets=[];calls=[];results=[]
node.declare_parameter('thrust_scaling',1.0)
router=node.create_publisher(Mavlink,'/uas1/mavlink_sink',10)
node.create_subscription(PositionTarget,'/mavros/setpoint_raw/local',lambda m:commands.append((time.monotonic(),m)),100)
node.create_subscription(TrackedTarget,'/Safina_track_stamped',lambda m:targets.append((time.monotonic(),m)),20)
def service(req,res):calls.append(type(req).__name__);return res
node.create_service(SetMode,'/mavros/set_mode',service);node.create_service(CommandBool,'/mavros/cmd/arming',service)
pubs={k:node.create_publisher(t,name,2) for k,t,name in [
 ('pose',PoseStamped,'/mavros/local_position/pose'),('vel',TwistStamped,'/mavros/local_position/velocity_local'),
 ('state',State,'/mavros/state'),('rc',RCIn,'/mavros/rc/in'),('landed',ExtendedState,'/mavros/extended_state'),
 ('rgb',Image,'/camera/color/image_raw'),('depth',Image,'/camera/depth/image_raw')]}
bus=cv2.resize(cv2.imread(sys.argv[1]),(1280,720));blank=np.zeros_like(bus)
v=dict(mode='POSCTL',armed=True,pwm=1500,depth=3000,rgb=bus,sendstate=True,sendrc=True,senddepth=True)
def phase(name,duration=2.,**changes):
 v.update(changes);start=time.monotonic();last_tele=last_image=0
 while time.monotonic()-start<duration:
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
    m=bridge.cv2_to_imgmsg(v['rgb'],'bgr8');header(m,'color_optical');pubs['rgb'].publish(m)
  rclpy.spin_once(node,timeout_sec=.005)
 tail=time.monotonic()-.5;a=[m for t,m in commands if t>tail];b=[m for t,m in targets if t>tail]
 print('PHASE',name,'commands',len(a),'targets',len(b),flush=True);return a,b

def check(name,ok):results.append(dict(check=name,passed=bool(ok)));print(json.dumps(results[-1]),flush=True)
def body(m):return m.coordinate_frame==m.FRAME_BODY_NED
def hold(a):return bool(a) and all(m.coordinate_frame==m.FRAME_LOCAL_NED and abs(m.position.x-2)<.01 and abs(m.position.y-3)<.01 for m in a)
a,b=phase('POSCTL',duration=3.);check('POSCTL prestream holds and detector session inactive',hold(a) and bool(b) and all(not m.session_active for m in b))
a,b=phase('follow positive error',mode='OFFBOARD',duration=3.);check('actual NPU target reaches controller and forward speed',bool(b) and any(m.state==0 and m.distance_source==2 and abs(m.distance_m-3)<.03 for m in b) and bool(a) and all(body(m) and m.velocity.x>.28 for m in a))
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
print(json.dumps(dict(passed=sum(x['passed'] for x in results),total=len(results),results=results)),flush=True)
node.destroy_node();rclpy.shutdown();sys.exit(0 if all(x['passed'] for x in results) else 1)
