#!/usr/bin/env python3
"""真实转发二进制 + 随机localhost ROS master；不连接真实飞控。"""
import os,sys,socket,subprocess,time,tempfile,math
from pathlib import Path
sock=socket.socket();sock.bind(('127.0.0.1',0));port=sock.getsockname()[1];sock.close()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d'%port,ROS_IP='127.0.0.1',ROS_HOSTNAME='localhost')
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
binary=str(Path(sys.argv[1]).resolve());procs=[]
with tempfile.TemporaryDirectory(prefix='relay-test-') as tmp:
 os.environ['ROS_HOME']=tmp
 log=open(tmp+'/process.log','w')
 try:
  procs.append(subprocess.Popen(['roscore','-p',str(port)],stdout=log,stderr=log))
  import xmlrpc.client
  for _ in range(100):
   try:
    if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/probe')[0]==1:break
   except OSError:pass
   time.sleep(.05)
  rospy.init_node('relay_timestamp_test',anonymous=True,disable_signals=True)
  got=[];rospy.Subscriber('/test/relay_out',PoseStamped,lambda m:got.append((time.monotonic(),m)),queue_size=100)
  pub=rospy.Publisher('/test/source',Odometry,queue_size=1)
  procs.append(subprocess.Popen([binary,'__name:=relay_fixture','_vision_pose_topic:=/test/source','/mavros/vision_pose/pose:=/test/relay_out'],stdout=log,stderr=log))
  until=time.monotonic()+5
  while pub.get_num_connections()==0 and time.monotonic()<until:time.sleep(.02)
  assert pub.get_num_connections()>0
  def msg(stamp=None,x=1.,y=2.,z=3.,yaw=.3):
   m=Odometry();m.header.stamp=stamp if stamp is not None else rospy.Time.now();m.header.frame_id='camera_init';m.child_frame_id='body'
   m.pose.pose.position.x=x;m.pose.pose.position.y=y;m.pose.pose.position.z=z
   m.pose.pose.orientation.z=math.sin(yaw/2);m.pose.pose.orientation.w=math.cos(yaw/2);return m
  # 重复源戳不能推进100帧初始化。
  repeated=msg()
  for _ in range(105):pub.publish(repeated);time.sleep(.003)
  time.sleep(.1);assert not got,'duplicate samples advanced initialization'
  sent=set()
  for i in range(240):
   m=msg();sent.add(m.header.stamp.to_nsec());pub.publish(m);time.sleep(.04)
   if i>=110 and got:break
  time.sleep(.1);assert got,'initialization did not complete'
  assert all(m.header.stamp.to_nsec() in sent for _,m in got),'source stamp replaced'
  got.clear()
  for _ in range(15):
   m=msg(rospy.Time.now()-rospy.Duration(.06),1.1,2.2,3.3,.4);sent.add(m.header.stamp.to_nsec());pub.publish(m);time.sleep(.1)
  time.sleep(.15);assert len(got)==15,('not one publish per unique 10Hz input',len(got))
  assert len(set(m.header.stamp.to_nsec() for _,m in got))==15
  for _,m in got:
   assert m.header.stamp.to_nsec() in sent
   assert abs(m.pose.position.x-.1)<1e-8 and abs(m.pose.position.y-.2)<1e-8 and abs(m.pose.position.z-.3)<1e-8
   assert abs(m.pose.orientation.z-math.sin(.05))<1e-8,'legacy transform changed'
  count=len(got);time.sleep(.4);assert len(got)==count,'stale pose repeated after source stopped'
  invalid=[msg(rospy.Time()),msg(rospy.Time.now()-rospy.Duration(.5)),msg(rospy.Time.now()+rospy.Duration(1)),msg(got[-1][1].header.stamp),msg(got[-1][1].header.stamp-rospy.Duration(.01)),msg(x=float('nan'))]
  wrong=msg();wrong.header.frame_id='wrong';invalid.append(wrong)
  wrong=msg();wrong.pose.pose.orientation.w=0;wrong.pose.pose.orientation.z=0;invalid.append(wrong)
  for m in invalid:pub.publish(m);time.sleep(.08)
  time.sleep(.1);assert len(got)==count,'invalid input published'
  m=msg();pub.publish(m);time.sleep(.1);assert len(got)==count+1,'fresh input failed to recover'
  assert got[-1][1].header.stamp==m.header.stamp
  print('PASS: isolated native relay initialization, original geometry, exact source stamps, 10Hz unique output, source stall, zero/stale/future/duplicate/out-of-order/NaN/frame/quaternion rejection, fresh recovery')
 finally:
  rospy.signal_shutdown('test complete')
  for proc in reversed(procs):
   proc.terminate()
   try:proc.wait(timeout=5)
   except subprocess.TimeoutExpired:proc.kill();proc.wait()
  log.flush()
  if sys.exc_info()[0]:print(Path(tmp+'/process.log').read_text()[-5000:])
