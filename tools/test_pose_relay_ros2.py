#!/usr/bin/env python3
"""原生ROS2转发与.235原二进制黑盒参考比较，全部话题隔离，无飞控服务。"""
import os
os.environ.update(ROS_DOMAIN_ID='197',ROS_LOCALHOST_ONLY='1')
import sys,time,json,math,subprocess,signal
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
ROOT=Path(sys.argv[1]);os.environ['ROS_LOG_DIR']=str(ROOT/'ros2-logs')
reference=json.loads((ROOT/'reference.json').read_text())
binary='/home/orangepi/drone_platform/02_drivers/sensors/workspace/install/to_mavros/lib/to_mavros/vision2mavros'
rclpy.init();n=rclpy.create_node('relay_ros2_fixture');received=[];checks={};sent={};processes=[];logs=[]
pub=n.create_publisher(Odometry,'/relay_test/source',1)
n.create_subscription(PoseStamped,'/relay_test/output',received.append,100)
def check(name,value):
 checks[name]=bool(value);print(('PASS ' if value else 'FAIL ')+name,flush=True);assert value,name

def pump(seconds,predicate=None):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.005)
  if predicate and predicate():return True
 return bool(predicate and predicate())

def start(extra=None):
 log=(ROOT/('node-%d.log'%len(logs))).open('w');logs.append(log)
 args=[binary,'--ros-args','-r','/factory_fastlio/odometry:=/relay_test/source','-r','/mavros/vision_pose/pose:=/relay_test/output','-p','body_lidar_x:=0.05','-p','body_lidar_y:=-0.02','-p','body_lidar_z:=0.1']+(extra or [])
 p=subprocess.Popen(args,stdout=log,stderr=log);processes.append(p)
 if not extra:
  check('subscription_%d'%len(processes),pump(5,lambda:pub.get_subscription_count()>0))
  pump(.5)  # DDS发现先于数据通道完全就绪，等待握手后才计数。
 return p

def stop(p):
 if p.poll() is None:
  p.send_signal(signal.SIGINT)
  try:p.wait(timeout=4)
  except subprocess.TimeoutExpired:p.kill();p.wait()
 pump(.3)

def message(values,age=0,stamp=None):
 m=Odometry();ns=n.get_clock().now().nanoseconds-int(age*1e9) if stamp is None else stamp
 m.header.stamp.sec=ns//1000000000;m.header.stamp.nanosec=ns%1000000000
 m.header.frame_id='camera_init';m.child_frame_id='body'
 m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=values[:3]
 m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=values[3:]
 return m

def nanoseconds(m):return m.header.stamp.sec*1000000000+m.header.stamp.nanosec

def pose(m):
 p,q=m.pose.position,m.pose.orientation
 return [p.x,p.y,p.z,q.x,q.y,q.z,q.w]

try:
 p=start();repeated=message(reference['inputs'][0])
 for _ in range(105):pub.publish(repeated);pump(.003)
 pump(.1);check('duplicates_do_not_advance_initialization',not received)
 stop(p);check('first_process_graceful_exit',p.returncode==0)
 pump(.5);received.clear();p=start()
 for i,values in enumerate(reference['inputs']):
  m=message(values);sent[nanoseconds(m)]=i;pub.publish(m);pump(.06)
 pump(.15)
 print('OUTPUT_INDICES '+str([sent.get(nanoseconds(m)) for m in received]),flush=True)
 check('initialization_exactly_100_unique_samples',bool(received) and sent[nanoseconds(received[0])]==99)
 check('timestamps_from_input_without_rewrite',all(nanoseconds(m) in sent for m in received))
 check('source_timestamp_published_at_most_once',len({nanoseconds(m) for m in received})==len(received))
 errors=[];compared=0
 for m in received:
  index=str(sent[nanoseconds(m)])
  if index not in reference['expected']:continue
  expected=reference['expected'][index]
  errors.append(max(abs(a-b) for a,b in zip(pose(m),expected['pose'])))
  check('frame_map_%s'%index,m.header.frame_id==expected['frame']) if m is received[0] else None
  compared+=1
 check('compared_over_50_original_binary_measurements',compared>50)
 check('legacy_geometry_component_difference_below_1e9',bool(errors) and max(errors)<1e-9)
 before=len(received);pump(.4);check('no_cached_pose_after_source_stall',len(received)==before)
 last=nanoseconds(received[-1]);values=reference['inputs'][-1]
 invalid=[message(values,stamp=0),message(values,age=.5),message(values,age=-1),message(values,stamp=last),message(values,stamp=last-1)]
 bad=message(values);bad.pose.pose.position.x=float('nan');invalid.append(bad)
 bad=message(values);bad.header.frame_id='map';invalid.append(bad)
 bad=message(values);bad.child_frame_id='lidar';invalid.append(bad)
 bad=message(values);bad.pose.pose.orientation.x=0.;bad.pose.pose.orientation.y=0.;bad.pose.pose.orientation.z=0.;bad.pose.pose.orientation.w=0.;invalid.append(bad)
 for m in invalid:pub.publish(m);pump(.07)
 pump(.1);check('zero_stale_future_duplicate_regression_nan_frames_quaternion_rejected',len(received)==before)
 exact=[]
 for _ in range(15):
  m=message(values,age=.06);exact.append(nanoseconds(m));pub.publish(m);pump(.1)
 pump(.1);actual=[nanoseconds(m) for m in received[before:]]
 check('10hz_source_is_10hz_unique_output_under_30hz_cap',actual==exact)
 check('fresh_input_recovers_after_rejected_samples',len(received)==before+15)
 before=len(received);pump(.4);check('source_stall_after_recovery_still_no_republish',len(received)==before)
 publishers=n.get_publishers_info_by_topic('/relay_test/output');check('only_pose_publisher',len(publishers)==1)
 topics=n.get_publisher_names_and_types_by_node(publishers[0].node_name,publishers[0].node_namespace)
 check('no_setpoint_or_mode_or_arming_output',not any('setpoint' in t or '/mavros/' in t for t,_ in topics))
 stop(p);check('relay_graceful_shutdown',p.returncode==0)
 p=start(['-p','publish_rate:=0.0']);pump(2);check('invalid_rate_exits_nonzero',p.poll()==1)
finally:
 for p in processes:stop(p)
 for log in logs:log.close()
 (ROOT/'ros2-result.json').write_text(json.dumps({'checks':checks,'compared':locals().get('compared',0),'maximum_geometry_error':max(errors) if locals().get('errors') else None,'isolation':{'domain':197,'localhost':True},'source_reference_binary_sha256':reference['source_binary_sha256']},indent=2)+'\n')
 n.destroy_node();rclpy.shutdown()
