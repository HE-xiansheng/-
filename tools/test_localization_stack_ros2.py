#!/usr/bin/env python3
"""真实FAST-LIO+Pose转发及会话管理隔离验证；只回放旧机传感器样本与伪FC。"""
import os
os.environ.update(ROS_DOMAIN_ID='196',ROS_LOCALHOST_ONLY='1')
import sys,time,json,gzip,copy,subprocess,signal,struct,math
from pathlib import Path
import rclpy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from validate_fastlio_replay import decode
ROOT=Path(sys.argv[1]);ROOT.mkdir(parents=True,exist_ok=True)
os.environ['ROS_LOG_DIR']=str(ROOT/'fixture-ros-logs')
PLATFORM=Path('/home/orangepi/drone_platform');MANAGER=PLATFORM/'01_apps/fast_lio/manage_stack.py'
env=dict(os.environ,LOCALIZATION_RUNTIME_DIR=str(ROOT/'runtime'),EGO_EXECUTION_RUNTIME_DIR=str(ROOT/'execution'),EGO_PREVIEW_RUNTIME_DIR=str(ROOT/'preview'))
with gzip.open(ROOT/'livox.jsonl.gz','rt') as stream:messages=[decode(json.loads(line)) for line in stream]
imu=[m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for t,_,m in messages if t=='/livox/imu'];minimum,maximum=min(imu),max(imu)
ordered=[]
for topic,_,m in messages:
 stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;event=stamp+(max((p.offset_time for p in m.points),default=0)*1e-9 if topic=='/livox/lidar' else 0)
 if topic=='/livox/imu' or minimum<=stamp<=event<=maximum:ordered.append((event-minimum,topic,m,int(round((stamp-minimum)*1e9))))
ordered.sort(key=lambda row:row[0]);period=maximum-minimum+.12
rclpy.init();n=rclpy.create_node('localization_stack_fixture');checks={};outputs=[];odometry=[];mode='POSCTL';armed=False;connected=True;age=0.;feed_sensors=True
pubs={'/livox/lidar':n.create_publisher(CustomMsg,'/livox/lidar',10),'/livox/imu':n.create_publisher(Imu,'/livox/imu',200)}
fc=n.create_publisher(State,'/mavros/state',1)
n.create_subscription(PoseStamped,'/mavros/vision_pose/pose',outputs.append,100)
n.create_subscription(Odometry,'/factory_fastlio/odometry',odometry.append,100)
start=time.monotonic()+.1;epoch=time.time_ns()+100000000;index=0;cycle=0;lastfc=0.;calls=[]
def check(name,value):
 checks[name]=bool(value);print(('PASS ' if value else 'FAIL ')+name,flush=True);assert value,name

def pump(seconds,predicate=None):
 global index,cycle,lastfc
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  now=time.monotonic()
  if now-lastfc>.05:
   lastfc=now;s=State();s.header.stamp=n.get_clock().now().to_msg();s.header.stamp.sec-=int(age);s.connected=connected;s.armed=armed;s.mode=mode;fc.publish(s)
  # 多轮保留原始测量间隔/点偏移，同步平移IMU和雷达源时刻。
  while feed_sensors and now>=start+cycle*period+ordered[index][0]:
   event,topic,m,relative=ordered[index];ns=epoch+int(cycle*period*1e9)+relative
   m.header.stamp.sec=ns//1000000000;m.header.stamp.nanosec=ns%1000000000
   if topic=='/livox/lidar':m.timebase=ns
   pubs[topic].publish(m);index+=1
   if index==len(ordered):index=0;cycle+=1
  rclpy.spin_once(n,timeout_sec=.002)
  if predicate and predicate():return True
 return bool(predicate and predicate())

def call(action,timeout=40):
 p=subprocess.Popen(['python3',str(MANAGER),action],env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
 if not pump(timeout,lambda:p.poll() is not None):
  p.kill();p.wait();raise RuntimeError('manager timeout '+action)
 out,err=p.communicate();text=out.decode();calls.append({'action':action,'code':p.returncode,'out':text,'err':err.decode()});print('MANAGER '+text.strip(),flush=True)
 return p.returncode,json.loads(text.splitlines()[-1])

def records():return {kind:json.loads((ROOT/'runtime'/(kind+'.json')).read_text()) for kind in ('localization','relay')}
fronts=[];front_logs=[]
sentinel=subprocess.Popen(['python3','-c','import time;time.sleep(180)'],start_new_session=True)
try:
 pump(.3);code,result=call('status');check('status_without_start',code==0 and not result['lio_running'] and not result['relay_running'])
 armed=True;pump(.2);code,result=call('start');check('armed_start_rejected',code!=0 and not (ROOT/'runtime/localization.json').exists())
 armed=False;foreign=n.create_publisher(Odometry,'/factory_fastlio/odometry',1);pump(.2)
 code,result=call('start');check('foreign_odometry_not_adopted',code!=0);n.destroy_publisher(foreign);pump(.2)
 control=n.create_publisher(PoseStamped,'/mavros/setpoint_position/local',1);pump(.2)
 code,result=call('start');check('control_publisher_blocks_start',code!=0);n.destroy_publisher(control);pump(.2)
 code,result=call('start');check('manager_starts_actual_fastlio_and_relay',code==0)
 first=records();check('actual_lio_outputs_expected_frames',bool(odometry) and odometry[-1].header.frame_id=='camera_init' and odometry[-1].child_frame_id=='body')
 check('relay_completes_100_sample_initialization_on_lio',pump(18,lambda:len(outputs)>3))
 check('relay_source_stamps_match_actual_lio',all(any(m.header.stamp==o.header.stamp for o in odometry) for m in outputs))
 code,result=call('start');second=records();check('healthy_start_reuses_both_processes_and_origins',code==0 and all(first[k]['pid']==second[k]['pid'] and first[k]['ticks']==second[k]['ticks'] for k in first))
 mode='OFFBOARD';pump(.2);code,result=call('stop');check('offboard_stop_rejected',code!=0 and all(Path('/proc/%s'%v['pid']).exists() for v in second.values()))
 mode='POSCTL';age=8.;pump(.2);code,result=call('stop');check('stale_fc_stop_rejected',code!=0)
 age=0.;planner=n.create_publisher(String,'/ego_planner/manual_preview/status',1);pump(.2)
 code,result=call('stop');check('active_planning_blocks_localization_stop',code!=0);n.destroy_publisher(planner);pump(.2)
 code,result=call('stop');check('controlled_stop_of_owned_stack',code==0)
 code,result=call('status');check('owned_sessions_report_stopped',code==0 and not result['lio_running'] and not result['relay_running'])
 check('unrelated_process_and_sensor_fixture_survive',sentinel.poll() is None and all(n.count_publishers(t)==1 for t in pubs))
 before=len(outputs);pump(.4);check('no_pose_output_after_owned_stop',len(outputs)==before)
 def foreground(kind):
  log=(ROOT/('foreground-'+kind+'.log')).open('w');front_logs.append(log)
  p=subprocess.Popen(['python3',str(MANAGER.parent/'manual_session.py'),kind],env=env,stdout=log,stderr=log,start_new_session=True);fronts.append(p);return p
 liofront=foreground('localization')
 check('foreground_lio_starts_without_relay',pump(12,lambda:(ROOT/'runtime/localization.json').exists()) and not (ROOT/'runtime/relay.json').exists())
 pump(2);lio_record=json.loads((ROOT/'runtime/localization.json').read_text())
 relayfront=foreground('relay')
 check('second_foreground_terminal_starts_only_relay',pump(12,lambda:(ROOT/'runtime/relay.json').exists()) and json.loads((ROOT/'runtime/localization.json').read_text())['pid']==lio_record['pid'])
 pump(1);relayfront.send_signal(signal.SIGINT)
 check('relay_foreground_interrupt_preserves_localization',pump(8,lambda:relayfront.poll() is not None) and not (ROOT/'runtime/relay.json').exists() and Path('/proc/%s'%lio_record['pid']).exists())
 armed=True;pump(.3);liofront.send_signal(signal.SIGINT)
 check('armed_foreground_interrupt_preserves_localization',pump(8,lambda:liofront.poll() is not None) and Path('/proc/%s'%lio_record['pid']).exists())
 armed=False;pump(.3);code,result=call('stop');check('foreground_preserved_session_can_be_stopped_after_disarm',code==0 and not (ROOT/'runtime/localization.json').exists())
finally:
 mode='POSCTL';armed=False;age=0.;connected=True
 try:call('stop')
 except Exception as e:print('CLEANUP_ERROR',e,flush=True)
 for front in fronts:
  if front.poll() is None:
   front.send_signal(signal.SIGINT)
   if not pump(8,lambda:front.poll() is not None):front.kill();front.wait()
 for log in front_logs:log.close()
 sentinel.terminate();sentinel.wait(timeout=3)
 (ROOT/'stack-result.json').write_text(json.dumps({'checks':checks,'passed':bool(checks) and all(checks.values()),'odom_samples':len(odometry),'pose_samples':len(outputs),'calls':calls,'isolation':{'domain':196,'localhost':True},'physical_flight_commands_sent':False},indent=2)+'\n')
 n.destroy_node();rclpy.shutdown()
