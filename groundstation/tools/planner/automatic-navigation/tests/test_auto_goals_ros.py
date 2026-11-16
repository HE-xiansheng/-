#!/usr/bin/env python3
"""独立ROS测试直接目标更新，不连接机载飞控。"""
import os,json,socket,subprocess,time,threading
from pathlib import Path
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2
from std_msgs.msg import Header,String,Bool
from traj_utils.msg import PolyTraj
root=Path(__file__).resolve().parent
assert os.environ['ROS_MASTER_URI']=='http://127.0.0.1:11392' and os.environ['ROS_IP']=='127.0.0.1'
with socket.socket() as probe:probe.bind(('127.0.0.1',11392))
master=subprocess.Popen(['roscore','-p','11392'],stdout=(root/'master.log').open('w'),stderr=subprocess.STDOUT)
planner=None;stop=threading.Event();latest={};polys=[];checks=[];sensor={'p':[0.,0.,1.],'v':[0.,0.,0.],'obstacles':[]}
def wait(pred,label,timeout=25):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  if pred():checks.append(label);print('PASS',label,flush=True);return
  if planner and planner.poll() is not None:raise RuntimeError('planner退出')
  time.sleep(.03)
 raise AssertionError(label+': '+json.dumps(latest))
try:
 time.sleep(2);rospy.init_node('auto_goals_test',disable_signals=True)
 planner=subprocess.Popen(['roslaunch',str(root/'native_ws/src/ego_planner/plan_manage/launch/realflight/run_planner_board.launch'),'manual_preview:=true'],stdout=(root/'planner.log').open('w'),stderr=subprocess.STDOUT)
 odom=rospy.Publisher('/factory_fastlio/odometry',Odometry,queue_size=1)
 cloud=rospy.Publisher('/factory_fastlio/cloud_world',point_cloud2.PointCloud2,queue_size=1)
 goal=rospy.Publisher('/diff_planner/navigation/preview_goal',PoseStamped,queue_size=1)
 execution=rospy.Publisher('/diff_planner/flight_execution',Bool,queue_size=1,latch=True)
 def status(m):latest.clear();latest.update(json.loads(m.data))
 subs=[rospy.Subscriber('/diff_planner/manual_preview/status',String,status),rospy.Subscriber('/diff_planner/manual_preview/trajectory',PolyTraj,lambda m:polys.append(m))]
 floor=[(x*.2,y*.2,-.6) for x in range(-25,26) for y in range(-25,26)]
 def feed():
  while not stop.is_set():
   now=rospy.Time.now()-rospy.Duration(sensor.get('age',0));o=Odometry();o.header.stamp=now;o.header.frame_id=sensor.get('frame','camera_init');o.child_frame_id=sensor.get('child','body');o.pose.pose.orientation.w=1.
   o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=sensor['p'];o.twist.twist.linear.x,o.twist.twist.linear.y,o.twist.twist.linear.z=sensor['v'];odom.publish(o)
   cloud.publish(point_cloud2.create_cloud_xyz32(Header(stamp=now,frame_id='camera_init'),floor+sensor['obstacles']));stop.wait(.08)
 threading.Thread(target=feed,daemon=True).start()
 def pick(x,y,z=1.):
  m=PoseStamped();m.header.stamp=rospy.Time.now();m.header.frame_id='world';m.pose.orientation.w=1.;m.pose.position.x=x;m.pose.position.y=y;m.pose.position.z=z;goal.publish(m)
 wait(lambda:latest.get('phase')=='WAIT_GOAL','初始等待目标')
 pick(8,0);wait(lambda:latest.get('phase')=='ACTIVE','远目标无需确认即可预览')
 assert latest['confirmation_required'] is False
 checks.append('无人工授权门槛')
 execution.publish(Bool(data=True));wait(lambda:latest.get('planning_mode')=='upstream_fsm' and latest.get('phase')=='ACTIVE' and polys,'原生执行状态接管')
 first=latest['goal_id'];sensor['p']=[.35,.2,1.];sensor['v']=[.3,0.,0.]
 pick(8,2);wait(lambda:latest.get('goal_id',0)>first and latest.get('goal')==[8.,2.,1.],'未到达且移动超过10cm时直接更新远目标')
 wait(lambda:latest.get('phase')=='ACTIVE' and latest.get('planning_mode')=='upstream_fsm','新目标原生重规划成功')
 assert latest['candidate']['phase']=='NONE' and latest['confirmation_required'] is False
 checks.append('不进入候选复查/令牌/地图范围工作流')
 sensor['p']=[.7,.3,1.];pick(7,-2)
 wait(lambda:latest.get('goal')==[7.,-2.,1.] and latest.get('phase')=='ACTIVE','同次执行中第二次连续选点')
 old=latest['goal_id'];pick(float('nan'),0);time.sleep(.3);assert latest['goal_id']==old;checks.append('非法数值不污染原生规划状态')
 execution.publish(Bool(data=False));wait(lambda:latest.get('planning_mode')=='measured_preview','人工模式可返回预览')
 sensor.update(age=8,frame='other_odom',child='other_body');pick(9,2)
 wait(lambda:latest.get('goal')==[9.,2.,1.] and latest.get('phase')=='ACTIVE','删除手动预览额外时效与坐标名白名单')
 sensor.update(age=0,frame='camera_init',child='body')
 sensor['v']=[0,0,0];sensor['p']=[0,0,1]
 sensor['obstacles']=[(2+x*.1,y*.1,1+z*.1) for x in range(-2,3) for y in range(-2,3) for z in range(-2,3)]
 time.sleep(1.5);pick(2,0)
 wait(lambda:latest.get('phase')=='GOAL_IN_COLLISION','保留膨胀地图碰撞拒绝')
 assert latest['confirmation_required'] is False;checks.append('碰撞失败不新增确认条件')
 (root/'ros-validation.json').write_text(json.dumps({'passed':True,'checks':checks},ensure_ascii=False,indent=2))
finally:
 stop.set()
 for proc in [planner,master]:
  if proc and proc.poll() is None:
   proc.terminate()
   try:proc.wait(timeout=8)
   except subprocess.TimeoutExpired:proc.kill();proc.wait()
