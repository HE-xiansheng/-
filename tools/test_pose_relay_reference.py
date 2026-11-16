#!/usr/bin/env python3
"""在独立ROS1 master用.235原二进制采集黑盒参考；不连接生产ROS图或飞控。"""
import os,sys,socket,subprocess,tempfile,time,json,math,xmlrpc.client,hashlib
from pathlib import Path
sock=socket.socket();sock.bind(('0.0.0.0',0));port=sock.getsockname()[1];sock.close()
os.environ.update(ROS_MASTER_URI='http://192.168.1.89:%d'%port,ROS_IP='192.168.1.89')
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
ROOT=Path(sys.argv[1]);ROOT.mkdir(parents=True,exist_ok=True)
SSH=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
def quaternion(r,p,y):
 cr,sr,cp,sp,cy,sy=math.cos(r/2),math.sin(r/2),math.cos(p/2),math.sin(p/2),math.cos(y/2),math.sin(y/2)
 return [sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]

def values(i):
 if i<100:p=[1+.001*i,2-.0005*i,3+.0002*i];rpy=[.12+.0001*i,-.15,.3]
 else:
  j=i-100;p=[1+.01*j,2+.05*math.sin(j*.2),3+.002*j]
  rpy=[.12+.04*math.sin(j*.3),1.55 if j>=50 else -.15+.02*math.cos(j),(-3.1 if j%2 else 3.1) if j>=30 else .3+.005*j]
 return p+quaternion(*rpy)

procs=[];samples=[];mapping={};sent=[]
with tempfile.TemporaryDirectory(prefix='pose-reference-') as tmp:
 os.environ['ROS_HOME']=tmp;os.environ['ROS_LOG_DIR']=str(ROOT/'ros-logs')
 with (ROOT/'master.log').open('w') as log,(ROOT/'binary.log').open('w') as err:
  try:
   master=subprocess.Popen(['roscore','-p',str(port)],stdout=log,stderr=log);procs.append(master)
   for _ in range(100):
    try:
     if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/probe')[0]==1:break
    except OSError:pass
    time.sleep(.05)
   command=('tar -xzf /tmp/gcs-localization-check/reference-runtime.tar.gz -C /tmp/gcs-localization-check/reference && '
            'timeout 28 env LD_LIBRARY_PATH=/tmp/gcs-localization-check/reference ROS_MASTER_URI='+os.environ['ROS_MASTER_URI']+' '
            'ROS_IP=192.168.1.224 ROS_LOG_DIR=/tmp/gcs-localization-check/ref-ros-logs '
            '/tmp/gcs-localization-check/reference/vision2mavros __name:=migration_reference_relay '
            '_vision_pose_topic:=/reference_source /mavros/vision_pose/pose:=/reference_pose '
            '_body_lidar_x:=0.05 _body_lidar_y:=-0.02 _body_lidar_z:=0.1')

   process=subprocess.Popen(SSH+[command],stdout=err,stderr=err);procs.append(process)
   rospy.init_node('pose_reference_fixture',anonymous=True,disable_signals=True)
   rospy.Subscriber('/reference_pose',PoseStamped,lambda m:samples.append(m),queue_size=200)
   pub=rospy.Publisher('/reference_source',Odometry,queue_size=1)
   end=time.monotonic()+8
   while not pub.get_num_connections() and time.monotonic()<end:time.sleep(.02)
   assert pub.get_num_connections()>0,'reference relay failed to subscribe'
   for i in range(170):
    m=Odometry();m.header.stamp=rospy.Time.now();m.header.frame_id='camera_init';m.child_frame_id='body'
    v=values(i);m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=v[:3]
    m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=v[3:]
    mapping[m.header.stamp.to_nsec()]=i;sent.append(v);pub.publish(m);time.sleep(.06)
   time.sleep(.3)
   expected={}
   for m in samples:
    index=mapping[m.header.stamp.to_nsec()];p,q=m.pose.position,m.pose.orientation
    expected[str(index)]={'pose':[p.x,p.y,p.z,q.x,q.y,q.z,q.w],'frame':m.header.frame_id}
   assert expected and min(map(int,expected))==99,'legacy initialization differs from 100 samples'
   assert len(expected)>50,'insufficient reference outputs'
   output={'source_binary_sha256':'04390ccaab975f2d3099994a9e919caad6c7f35a5cb97e4971605693017024a6','offset':[.05,-.02,.1],'inputs':sent,'expected':expected}
   (ROOT/'reference.json').write_text(json.dumps(output,indent=2)+'\n')
   print('PASS original .235 binary: 100-sample initialization, exact source timestamps, %d reference outputs'%len(expected),flush=True)
  finally:
   rospy.signal_shutdown('reference complete')
   # timeout给远端原始二进制明确期限；SSH提前退出也不依赖连接存活回收。
   if len(procs)>1:
    procs[-1].wait(timeout=40)
   if procs:
    procs[0].terminate()
    try:procs[0].wait(timeout=5)
    except subprocess.TimeoutExpired:procs[0].kill();procs[0].wait()
