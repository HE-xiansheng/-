#!/usr/bin/env python3
"""Native controller + fake PX4 integration; localhost DDS 87 only, no real MAVROS."""
import os,sys,time,math,json
assert os.environ.get('ROS_DOMAIN_ID')=='87' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,TwistStamped
from mavros_msgs.msg import State,RCIn,ExtendedState,PositionTarget
from mavros_msgs.srv import SetMode,CommandBool
from std_msgs.msg import Float32MultiArray
from yolo_ultra_test.msg import TrackedTarget

rclpy.init();node=Node('track_fake_px4');outputs=[];mode_calls=[];arm_calls=[];results=[]
node.create_subscription(PositionTarget,'/mavros/setpoint_raw/local',lambda m:outputs.append((time.monotonic(),m)),100)
def mode(req,res):
 mode_calls.append(req.custom_mode);res.mode_sent=len(mode_calls)>1;return res
def arm(req,res):
 arm_calls.append(req.value);res.success=False;return res
node.create_service(SetMode,'/mavros/set_mode',mode)
node.create_service(CommandBool,'/mavros/cmd/arming',arm)
pubs={key:node.create_publisher(typ,topic,2) for key,typ,topic in [
 ('pose',PoseStamped,'/mavros/local_position/pose'),('vel',TwistStamped,'/mavros/local_position/velocity_local'),
 ('state',State,'/mavros/state'),('rc',RCIn,'/mavros/rc/in'),('landed',ExtendedState,'/mavros/extended_state'),
 ('target',TrackedTarget,'/Safina_track_stamped'),('legacy',Float32MultiArray,'/Safina_track')]}
values=dict(mode='POSCTL',armed=False,connected=True,pwm=1100,z=1.,landed=2,distance=3.,source=2,valid=True,
 x=.4,y=.2,targetstate=0,session=True,targetage=0.,poseage=0.,stateage=0.,rcage=0.,velage=0.,vz=0.,frame='map',
 sendpose=True,sendstate=True,sendrc=True,sendvel=True,sendlanded=True,sendtarget=True,legacy=False)
def phase(label,duration=.8,**changes):
 values.update(changes);start=len(outputs);t0=time.monotonic();last=0.
 while time.monotonic()-t0<duration:
  if time.monotonic()-last>.045:
   last=time.monotonic();now=node.get_clock().now().nanoseconds
   def stamp(age=0.):return rclpy.time.Time(nanoseconds=max(0,now-int(age*1e9))).to_msg()
   if values['sendpose']:
    m=PoseStamped();m.header.stamp=stamp(values['poseage']);m.header.frame_id=values['frame'];m.pose.position.x=2.;m.pose.position.y=3.;m.pose.position.z=values['z'];m.pose.orientation.w=1.;pubs['pose'].publish(m)
   if values['sendvel']:
    m=TwistStamped();m.header.stamp=stamp(values['velage']);m.header.frame_id=values['frame'];m.twist.linear.z=values['vz'];pubs['vel'].publish(m)
   if values['sendstate']:
    m=State();m.header.stamp=stamp(values['stateage']);m.connected=values['connected'];m.armed=values['armed'];m.mode=values['mode'];pubs['state'].publish(m)
   if values['sendrc']:
    m=RCIn();m.header.stamp=stamp(values['rcage']);m.channels=[1500]*9;m.channels[8]=values['pwm'];pubs['rc'].publish(m)
   if values['sendlanded']:
    m=ExtendedState();m.header.stamp=stamp();m.landed_state=values['landed'];pubs['landed'].publish(m)
   if values['sendtarget']:
    m=TrackedTarget();m.header.stamp=stamp(values['targetage']);m.header.frame_id='camera_color_optical_frame';m.state=values['targetstate'];m.offset_x=values['x'];m.offset_y=values['y'];m.target_id=1;m.distance_m=values['distance'];m.distance_source=values['source'];m.distance_valid=values['valid'];m.session_active=values['session'];pubs['target'].publish(m)
   if values['legacy']:
    m=Float32MultiArray();m.data=[0.,1.,1.,1.,6.,2.,1.];pubs['legacy'].publish(m)
  rclpy.spin_once(node,timeout_sec=.005)
 out=[m for t,m in outputs[start:] if t>=t0+max(.1,duration-.3)]
 print('PHASE',label,'tail samples',len(out),flush=True);return out

def check(label,ok):
 results.append(dict(check=label,passed=bool(ok)));print(json.dumps(results[-1]),flush=True)
def position(m):return m.coordinate_frame==PositionTarget.FRAME_LOCAL_NED and not(m.type_mask&PositionTarget.IGNORE_PX)
def velocity(m):return m.coordinate_frame==PositionTarget.FRAME_BODY_NED and not(m.type_mask&PositionTarget.IGNORE_VX)
def hold(a):return bool(a) and all(position(m) and abs(m.position.x-2)<1e-6 and abs(m.position.y-3)<1e-6 for m in a)
def neutral(a):return bool(a) and all(position(m) or (velocity(m) and abs(m.velocity.x)<.01 and abs(m.velocity.z)<.01 and abs(m.yaw_rate)<.01) for m in a)
phase('discovery',1.)
a=phase('POSCTL down',pwm=1900);check('outside OFFBOARD holds current position and ignores SWA down',hold(a) and not mode_calls)
a=phase('disarmed OFFBOARD',mode='OFFBOARD',pwm=1500);check('disarmed OFFBOARD never tracks',hold(a))
a=phase('landed guard',armed=True,landed=1);check('on ground blocks following',hold(a))
a=phase('minimum altitude',landed=2,z=.2);check('below minimum altitude blocks following',hold(a))
a=phase('protection',pwm=1100,z=1.);check('protection holds',hold(a))
a=phase('metric forward',pwm=1500,duration=1.2);check('metric positive error yields capped forward FLU velocity',bool(a) and all(velocity(m) and abs(m.velocity.x-.3)<.02 for m in a));check('left offset gives positive yaw rate',bool(a) and all(abs(m.yaw_rate-.3)<.02 for m in a))
check('ROS time used for command header',bool(a) and all(abs(node.get_clock().now().nanoseconds*1e-9-(m.header.stamp.sec+m.header.stamp.nanosec*1e-9))<1. for m in a))
a=phase('metric reverse',distance=1.,duration=1.1);check('negative error caps reverse speed',bool(a) and all(velocity(m) and abs(m.velocity.x+.15)<.02 for m in a))
a=phase('deadband',distance=2.1);check('metric deadband zero forward velocity',bool(a) and all(velocity(m) and abs(m.velocity.x)<.01 for m in a))
a=phase('pixel fallback',valid=False,x=.1);check('invalid metric distance uses pixel heuristic',bool(a) and all(velocity(m) and abs(m.velocity.x-.08)<.02 for m in a))
a=phase('altitude damping',vz=.5,velage=0.);check('fresh vertical velocity damps altitude',bool(a) and all(velocity(m) and abs(m.velocity.z+.1)<.02 for m in a))
a=phase('stale velocity',velage=3.);check('stale velocity excluded from altitude damping',bool(a) and all(velocity(m) and abs(m.velocity.z)<.01 for m in a))
phase('velocity restored',vz=0.,velage=0.,duration=.3)
a=phase('stale target',targetage=3.,duration=1.1);check('stale source target brakes then holds',hold(a))
a=phase('target timeout',sendtarget=False,legacy=True,duration=1.1);check('legacy topic cannot override native stamped source',hold(a))
a=phase('nonfinite target',sendtarget=True,legacy=False,targetage=0.,x=float('nan'),duration=.8);check('NaN target rejected',hold(a))
a=phase('inactive session',x=.4,session=False);check('inactive inference session cannot track',hold(a))
a=phase('resume',session=True,valid=True,distance=3.,duration=1.1);check('fresh target resumes following',bool(a) and all(velocity(m) and m.velocity.x>.25 for m in a))
a=phase('future target',targetage=-3.,duration=1.1);check('future target rejected',hold(a))
a=phase('zero stamp target',targetage=1e12,duration=.6);check('zero source stamp rejected',hold(a))
phase('resume target',targetage=0.,duration=.8)
a=phase('changed local frame',frame='wrong_frame',duration=1.2);check('local frame change blocks tracking',bool(a) and all(velocity(m) and abs(m.velocity.x)<.01 for m in a))
phase('restore frame',frame='map',duration=.8)
a=phase('stale pose',poseage=3.,duration=1.2);check('stale pose brakes to zero without stale position hold',bool(a) and all(velocity(m) and abs(m.velocity.x)<.01 for m in a))
a=phase('resume pose',poseage=0.,duration=1.)
a=phase('RC timeout',sendrc=False,duration=1.4);check('RC timeout stops following',neutral(a))
phase('resume RC',sendrc=True,duration=1.)
a=phase('missing landed state',sendlanded=False,duration=1.4);check('landed state timeout blocks tracking',hold(a))
phase('resume landed',sendlanded=True,duration=1.)
a=phase('state timeout',sendstate=False,duration=2.5);check('state timeout stops command publication',not a)
a=phase('disconnected',sendstate=True,connected=False);check('disconnected state stops commands',not a)
phase('POSCTL reset',connected=True,mode='POSCTL',pwm=1900,duration=.5)
a=phase('OFFBOARD down entry',mode='OFFBOARD',duration=.6);check('down at OFFBOARD entry cannot trigger landing',hold(a) and not mode_calls)
phase('tracking before landing',pwm=1500,duration=.5)
a=phase('landing',pwm=1900,duration=.7);check('landing locks XY and descends local ENU position',hold(a) and all(m.position.z<.95 for m in a));check('no premature AUTO.LAND',not mode_calls)
a=phase('landing pose unavailable',sendpose=False,duration=.9);check('landing pose dropout pauses descent',bool(a) and all(velocity(m) and abs(m.velocity.z)<.01 for m in a));check('stale pose prevents AUTO.LAND service',not mode_calls)
a=phase('low altitude landing',sendpose=True,z=.08,duration=2.2);check('AUTO.LAND reject then retry uses only AUTO.LAND',len(mode_calls)>=2 and all(m=='AUTO.LAND' for m in mode_calls))
a=phase('pilot mode exit',mode='POSCTL',duration=.5);calls=len(mode_calls);phase('remains pilot mode',duration=1.1);check('mode exit cancels landing retries',len(mode_calls)==calls)
check('no arming service calls',not arm_calls)
check('all outputs finite and bounded',bool(outputs) and all(all(math.isfinite(v) for v in [m.position.x,m.position.y,m.position.z,m.velocity.x,m.velocity.y,m.velocity.z,m.yaw,m.yaw_rate]) and abs(m.velocity.x)<=.301 and abs(m.velocity.z)<=.401 and abs(m.yaw_rate)<=1.201 for _,m in outputs))
pairs=[((tb-ta),a,b) for (ta,a),(tb,b) in zip(outputs,outputs[1:]) if velocity(a) and velocity(b) and 0<tb-ta<.12]
check('consecutive body commands obey acceleration limits',bool(pairs) and all(abs(b.velocity.x-a.velocity.x)<=.6*dt+.04 and abs(b.velocity.z-a.velocity.z)<=.5*dt+.04 and abs(b.yaw_rate-a.yaw_rate)<=1.0*dt+.06 for dt,a,b in pairs))
print(json.dumps(dict(passed=sum(x['passed'] for x in results),total=len(results),results=results)),flush=True)
node.destroy_node();rclpy.shutdown();sys.exit(0 if all(x['passed'] for x in results) else 1)
