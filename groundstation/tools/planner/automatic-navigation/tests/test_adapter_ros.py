#!/usr/bin/env python3
"""独立master验证删去额外阻断，保留遥控模式与可计算数据要求。"""
import os,json,socket,subprocess,time,threading
from pathlib import Path
import rospy
from mavros_msgs.msg import State,ExtendedState,ParamValue,PositionTarget
from mavros_msgs.srv import ParamGet,ParamGetResponse
from geometry_msgs.msg import PoseStamped,TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from traj_utils.msg import PolyTraj
root=Path(__file__).resolve().parent
assert os.environ['ROS_MASTER_URI']=='http://127.0.0.1:11377' and os.environ['ROS_IP']=='127.0.0.1'
with socket.socket() as probe:probe.bind(('127.0.0.1',11377))
master=subprocess.Popen(['roscore','-p','11377'],stdout=(root/'adapter-master.log').open('w'),stderr=subprocess.STDOUT)
proc=None;stop=threading.Event();latest={};checks=[];fc_mode=['POSCTL'];armed=[False];airborne=[False]
def wait(pred,label,timeout=15):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  if pred():checks.append(label);print('PASS',label,flush=True);return
  if proc and proc.poll() is not None:raise AssertionError('adapter退出')
  time.sleep(.02)
 raise AssertionError(label+': '+json.dumps(latest))
try:
 time.sleep(2);rospy.init_node('auto_adapter_test',disable_signals=True)
 service=rospy.Service('/mavros/param/get',ParamGet,lambda req:ParamGetResponse(success=True,value=ParamValue(integer=4 if req.param_id=='COM_RC_IN_MODE' else 1,real=0)))
 pubs={k:rospy.Publisher(t,typ,queue_size=1) for k,t,typ in [('state','/mavros/state',State),('landed','/mavros/extended_state',ExtendedState),('fcpose','/mavros/local_position/pose',PoseStamped),('lio','/factory_fastlio/odometry',Odometry),('status','/diff_planner/manual_preview/status',String),('poly','/diff_planner/flight_execution/trajectory',PolyTraj),('other','/mavros/setpoint_velocity/cmd_vel',TwistStamped)]}
 def status(m):latest.clear();latest.update(json.loads(m.data))
 sub=rospy.Subscriber('/diff_planner/px4_execution/status',String,status)
 def feed():
  while not stop.is_set():
   old=rospy.Time.now()-rospy.Duration(8)
   s=State();s.header.stamp=old;s.connected=False;s.mode=fc_mode[0];s.armed=armed[0];pubs['state'].publish(s)
   e=ExtendedState();e.header.stamp=old;e.landed_state=2 if airborne[0] else 1;pubs['landed'].publish(e)
   f=PoseStamped();f.header.stamp=old;f.header.frame_id='non_whitelisted_frame';f.pose.orientation.w=1.;f.pose.position.z=1;pubs['fcpose'].publish(f)
   o=Odometry();o.header.stamp=old;o.header.frame_id='non_whitelisted_frame';o.child_frame_id='other_body';o.pose.pose=f.pose;pubs['lio'].publish(o)
   pubs['status'].publish(String(data=json.dumps({'stamp':old.to_sec(),'session_id':'s','goal_id':1,'has_goal':True,'goal':[4,0,1],'confirmation_required':True,'goal_confirmed':False,'planning_mode':'upstream_fsm','phase':'ACTIVE','trajectory_id':999})))
   t=PolyTraj();t.traj_id=1;t.start_time=rospy.Time.now()-rospy.Duration(.1);t.order=5;t.duration=[20];t.coef_x=[0]*6;t.coef_y=[0]*6;t.coef_z=[0,0,0,0,0,1];pubs['poly'].publish(t)
   pubs['other'].publish(TwistStamped());stop.wait(.05)
 threading.Thread(target=feed,daemon=True).start()
 run=root/'adapter-run';run.mkdir(exist_ok=True)
 adapter=Path(os.environ.get('AUTO_ADAPTER_PATH','/home/tyzk/dianyunGCS-review-20261026/dianyunGCS-master/tools/planner/automatic-navigation/px4-execution/adapter.py'))
 proc=subprocess.Popen(['/usr/bin/python3',str(adapter),'--run',str(run),'--publish','--isolated-test'],stdout=(root/'adapter.log').open('w'),stderr=subprocess.STDOUT)
 wait(lambda:latest.get('phase')=='WAIT_FLIGHT','未解锁时不进入执行')
 armed[0]=True;airborne[0]=True
 wait(lambda:latest.get('phase')=='READY','人工起飞后POSCTL待命')
 fc_mode[0]='OFFBOARD';wait(lambda:latest.get('phase')=='TRACK','遥控切OFFBOARD后跟踪原生输入')
 assert latest.get('hard_errors')==[]
 assert latest['planner_status']['goal_confirmed'] is False;checks.append('人工确认状态不再阻断')
 assert latest['trajectory_id']==1 and latest['planner_status']['trajectory_id']==999;checks.append('不再按额外轨迹ID校验拒绝')
 assert latest['input_ages']['lio']['source']>7.;checks.append('过期/非白名单坐标名/其他发布者不再额外检查或阻断')
 fc_mode[0]='POSCTL';wait(lambda:latest.get('phase')=='MANUAL_WAIT' and latest.get('tracking_output_enabled') is False,'遥控切回POSCTL接管')
 (root/'adapter-validation.json').write_text(json.dumps({'passed':True,'checks':checks},ensure_ascii=False,indent=2))
finally:
 stop.set()
 for child in [proc,master]:
  if child and child.poll() is None:
   child.terminate()
   try:child.wait(timeout=8)
   except subprocess.TimeoutExpired:child.kill();child.wait()
