#!/usr/bin/env python3
"""Isolated real-NPU test. Run only with ROS_DOMAIN_ID=87 and ROS_LOCALHOST_ONLY=1."""
import os,time,json,sys,copy
assert os.environ.get('ROS_DOMAIN_ID')=='87' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import cv2,numpy as np,rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from mavros_msgs.msg import State,RCIn
from std_msgs.msg import Float32MultiArray
from yolo_ultra_test.msg import TrackedTarget
from cv_bridge import CvBridge
rclpy.init();n=Node('yolo_bench');bridge=CvBridge();results=[];targets=[];arrays=[];images=[]
n.create_subscription(TrackedTarget,'/Safina_track_stamped',lambda m: targets.append(m),10)
n.create_subscription(Float32MultiArray,'/Safina_track',lambda m: arrays.append(m),10)
n.create_subscription(Image,'/yoloUltra_imgresult',lambda m: images.append(m),10)
pubs={name:n.create_publisher(typ,topic,2) for name,typ,topic in [('image',Image,'/camera/color/image_raw'),('depth',Image,'/camera/depth/image_raw'),('rc',RCIn,'/mavros/rc/in'),('state',State,'/mavros/state')]}
bus=cv2.resize(cv2.imread(sys.argv[1]),(1280,720));blank=np.zeros_like(bus)
def phase(name,duration=1.5,image=bus,depth='valid',rc=1500,mode='OFFBOARD',send_rc=True,send_state=True,image_age=0.0):
 start=len(targets);im_start=len(images);t0=time.monotonic();last=0
 while time.monotonic()-t0<duration:
  now=time.monotonic()
  if now-last>.13:
   last=now;stamp=n.get_clock().now().to_msg()
   if send_state:
    m=State();m.header.stamp=copy.deepcopy(stamp);m.connected=True;m.mode=mode;pubs['state'].publish(m)
   if send_rc:
    m=RCIn();m.header.stamp=copy.deepcopy(stamp);m.channels=[1500]*9;m.channels[8]=rc;pubs['rc'].publish(m)
   if depth:
    m=bridge.cv2_to_imgmsg(np.full((480,848),2000,np.uint16),'16UC1');m.header.frame_id='depth_optical';m.header.stamp=copy.deepcopy(stamp)
    if depth=='stale': m.header.stamp.sec-=3
    if depth=='future': m.header.stamp.sec+=3
    if depth=='zero': m.header.stamp.sec=0;m.header.stamp.nanosec=0
    pubs['depth'].publish(m)
   if image is not None:
    m=bridge.cv2_to_imgmsg(image,'bgr8');m.header.frame_id='color_optical';m.header.stamp=copy.deepcopy(stamp)
    m.header.stamp.sec-=int(image_age);pubs['image'].publish(m)
  rclpy.spin_once(n,timeout_sec=.01)
 out=targets[start:];print('PHASE',name,'samples',len(out),'visible',sum(m.state==0 for m in out),flush=True)
 return out,images[im_start:]
def check(name,value):
 results.append({'check':name,'pass':bool(value)});print(json.dumps(results[-1]),flush=True)
phase('discovery',2)
a,im=phase('positive',2)
v=[m for m in a if m.state==0];check('real NPU detects and assigns ID',bool(v) and all(m.target_id>0 for m in v));check('depth distance 2m',bool(v) and any(m.distance_source==2 and abs(m.distance_m-2)<.03 for m in v));check('stamped image frame preserved',bool(im) and all(m.header.frame_id=='color_optical' and m.header.stamp.sec>0 for m in im));check('target source header preserved',bool(v) and all(m.header.frame_id=='color_optical' for m in v));check('legacy seven fields',bool(arrays) and all(len(m.data)==7 for m in arrays))
for d in ('stale','future','zero'):
 a,_=phase(d,depth=d);v=[m for m in a[len(a)//2:] if m.state==0];check(d+' depth rejected',bool(v) and all(m.distance_source!=2 for m in v))
a,_=phase('valid depth again');check('depth resumes without mixing bbox history',any(m.state==0 and m.distance_source==2 and abs(m.distance_m-2)<.03 for m in a))
a,_=phase('blank',image=blank);check('absent person clears offsets',bool(a) and all(m.state==-1 and m.offset_x==0 and m.offset_y==0 for m in a[len(a)//2:]))
a,_=phase('land',rc=1900);check('land switch ends session',bool(a) and all(not m.session_active and m.state==-1 for m in a[len(a)//2:]))
a,_=phase('mid resets');check('session restart IDs begin at one',any(m.state==0 and m.target_id==1 for m in a))
a,_=phase('protection',rc=1100);check('protection retains session IDs',any(m.session_active and m.target_id==1 for m in a))
a,_=phase('mode exit',mode='POSCTL');check('mode exit clears session',bool(a) and all(not m.session_active and m.state==-1 for m in a[len(a)//2:]))
phase('restart');a,_=phase('image timeout',duration=1.5,image=None);check('image dropout clears output',bool(a) and all(m.state==-1 and not m.session_active for m in a[len(a)//2:]))
phase('restart');a,_=phase('RC timeout',duration=2,send_rc=False);check('RC dropout clears session',bool(a) and all(m.state==-1 and not m.session_active for m in a[-3:]))
phase('restart');a,_=phase('state timeout',duration=3,send_state=False);check('state dropout clears session',bool(a) and all(m.state==-1 and not m.session_active for m in a[-3:]))
a,_=phase('stale image',image_age=3);check('stale image rejected',bool(a) and all(m.state==-1 for m in a))
print(json.dumps({'results':results,'passed':sum(r['pass'] for r in results),'total':len(results)}),flush=True)
n.destroy_node();rclpy.shutdown();sys.exit(0 if all(r['pass'] for r in results) else 1)
