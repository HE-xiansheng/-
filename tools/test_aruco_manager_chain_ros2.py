#!/usr/bin/env python3
"""Real image->native ArUco->ROS2 task status; no landing/controller started."""
import os,time,sys,json,signal,subprocess,importlib.util
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='87' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import cv2,numpy as np
from sensor_msgs.msg import Image
from mavros_msgs.msg import PositionTarget
base=Path('/home/orangepi/drone_platform/01_apps/mission_demo')
sp=importlib.util.spec_from_file_location('manager',base/'tools/gcs_task_manager.py');mod=importlib.util.module_from_spec(sp);sp.loader.exec_module(mod)
ros=mod.RosState();node=ros.node
pub=node.create_publisher(Image,'/rtsp_cam/image_raw',2)
commands=[];node.create_subscription(PositionTarget,'/mavros/setpoint_raw/local',lambda m:commands.append(m),10)
root=Path('/tmp/aruco-manager-validation');root.mkdir(exist_ok=True);results=[]
def check(name,value):results.append(dict(name=name,passed=bool(value)));print(('PASS ' if value else 'FAIL ')+name,flush=True)
exe=base/'workspace/install/aruco_test/lib/aruco_test/Safina_aruco_test'
log=(root/'detector.log').open('w')
proc=subprocess.Popen([str(exe),'--ros-args','--params-file',str(base/'workspace/install/aruco_test/share/aruco_test/config/aruco.yaml'),'-p','preview_color:=false'],stdout=log,stderr=subprocess.STDOUT)
camera=np.array([[2042.0851788597752,0,959.0675902004489],[0,2042.8993693539358,467.50991502443503],[0,0,1.]])
distortion=np.array([-.4849208954761301,.3575460777522052,.0024881576093297698,-.0002765905391136276,0.])
marker=cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL),0,400)
size=.035
corners=np.float32([[-size/2,size/2,0],[size/2,size/2,0],[size/2,-size/2,0],[-size/2,-size/2,0]])
pixels,_=cv2.projectPoints(corners,np.array([np.pi,0.,0.]),np.array([0.,0.,1.5]),camera,distortion)
transform=cv2.getPerspectiveTransform(np.float32([[0,0],[399,0],[399,399],[0,399]]),pixels.reshape(4,2).astype(np.float32))
image=cv2.warpPerspective(marker,transform,(1920,1080),borderValue=255)
def pump(image,age=0.,frame='gimbal_camera_optical_frame',duration=1.5):
 end=time.monotonic()+duration;observations=[]
 while time.monotonic()<end:
  m=Image();m.header.frame_id=frame;m.height,m.width=image.shape;m.encoding='mono8';m.step=m.width;m.data=image.tobytes()
  # Preparing a 2MB Python ROS message is slow; stamp the ready synthetic frame.
  ns=max(0,node.get_clock().now().nanoseconds-int(age*1e9));m.header.stamp.sec=ns//1000000000;m.header.stamp.nanosec=ns%1000000000
  pub.publish(m);time.sleep(.12);observations.append(ros.summary()['aruco'])
 return observations

try:
 end=time.monotonic()+8
 while not pub.get_subscription_count() and time.monotonic()<end:time.sleep(.05)
 check('native detector discovered',pub.get_subscription_count()>0)
 observations=pump(image,duration=2.)
 valid=[x for x in observations if x['valid']]
 with ros.lock:
  (root/'pair-debug.json').write_text(json.dumps(dict(marker=ros.cache.get('marker_pose'),detector=ros.cache.get('detector'),summary=observations[-1]),indent=2))
 check('real image produces valid groundstation marker',bool(valid) and all(x['id']==0 and abs(x['offset']['z']-1.5)<.15 for x in valid))
 check('metrics exactly pair with native pose',bool(valid) and all(isinstance(x['detector'].get('pose_stamp_ns'),int) for x in valid))
 observations=pump(image,age=3.);check('old image source never reported valid',not any(x['valid'] for x in observations[-5:]))
 observations=pump(image,age=-3.);check('future image source never reported valid',not any(x['valid'] for x in observations[-5:]))
 observations=pump(image,frame='wrong');check('wrong optical frame rejected',not any(x['valid'] for x in observations[-5:]))
 observations=pump(image);check('fresh correctly framed image recovers',any(x['valid'] for x in observations[-5:]))
 observations=pump(np.full((1080,1920),255,dtype=np.uint8));check('blank image clears marker validity',not any(x['valid'] for x in observations[-5:]))
 check('image/status chain has no flight outputs',not commands)
finally:
 if proc.poll() is None:proc.send_signal(signal.SIGINT)
 try:proc.wait(timeout=5)
 except subprocess.TimeoutExpired:proc.kill();proc.wait()
 check('detector normal exit',proc.returncode==0)
 log.close();ros.close();(root/'results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2));print(json.dumps(dict(passed=sum(x['passed'] for x in results),total=len(results))),flush=True)
raise SystemExit(1 if any(not x['passed'] for x in results) else 0)
