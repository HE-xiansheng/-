#!/usr/bin/env python3
"""Controlled IMU clock offset against original and candidate native FAST-LIO.

Only recorded sensor inputs, isolated localhost DDS; no FC interfaces.
"""
import gzip,json,math,os,signal,subprocess,sys,time
from pathlib import Path
import rclpy
from sensor_msgs.msg import Imu,PointCloud2
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg
from validate_fastlio_replay import decode

out=Path(sys.argv[1]);assert out==Path('/tmp/fastlio-self-sync-check')
assert os.environ['ROS_LOCALHOST_ONLY']=='1' and os.environ['ROS_DOMAIN_ID']=='84'
root=Path('/home/orangepi/drone_platform')
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
with gzip.open(out/'livox.jsonl.gz','rt') as f:messages=[decode(json.loads(s)) for s in f]
imu=[stamp(m) for t,_,m in messages if t=='/livox/imu'];lo,hi=min(imu),max(imu);events=[]
for topic,_,m in messages:
 ns=stamp(m);event=ns+(max((p.offset_time for p in m.points),default=0) if topic=='/livox/lidar' else 0)
 if topic=='/livox/imu' or lo<=ns<=event<=hi:events.append(((event-lo)/1e9,topic,m,ns-lo))
events.sort(key=lambda e:e[0]);period=(hi-lo)/1e9+.12
rclpy.init();node=rclpy.create_node('self_sync_branch_observer')
pubs={'/livox/lidar':node.create_publisher(CustomMsg,'/livox/lidar',20),'/livox/imu':node.create_publisher(Imu,'/livox/imu',200)}
rows=[];clouds=[]
node.create_subscription(Odometry,'/Odometry',lambda m:rows.append((time.monotonic(),stamp(m),m.header.frame_id,m.child_frame_id,[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z,m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w])),100)
node.create_subscription(PointCloud2,'/cloud_registered_body',lambda m:clouds.append(stamp(m)),30)
results={}
try:
 for label,binary in [('original',root/'01_apps/fast_lio/workspace/install/fast_lio/lib/fast_lio/fastlio_mapping'),('candidate',Path('/tmp/fastlio-sync-candidate/install/lib/fast_lio/fastlio_mapping'))]:
  assert binary.is_file(),str(binary)
  rows.clear();clouds.clear();log=out/(label+'.log')
  with log.open('w') as f:
   p=subprocess.Popen([str(binary),'--ros-args','--params-file',str(root/'01_apps/fast_lio/config/lidar.yaml')],stdout=f,stderr=subprocess.STDOUT,start_new_session=True,cwd=out)
   try:
    deadline=time.monotonic()+15
    while time.monotonic()<deadline and not all(q.get_subscription_count() for q in pubs.values()):rclpy.spin_once(node,timeout_sec=.05)
    assert all(q.get_subscription_count() for q in pubs.values()),'missing LIO subscriptions'
    begin=time.monotonic()+.15;epoch=time.time_ns()+150_000_000
    for cycle in range(3):
     for offset,topic,m,relative in events:
      target=begin+cycle*period+offset
      while time.monotonic()<target:rclpy.spin_once(node,timeout_sec=min(.01,max(0,target-time.monotonic())))
      assert p.poll() is None,'LIO exited'
      ns=epoch+round(cycle*period*1e9)+relative-(2_000_000_000 if topic=='/livox/imu' else 0)
      m.header.stamp.sec=ns//10**9;m.header.stamp.nanosec=ns%10**9
      if topic=='/livox/lidar':m.timebase=ns
      pubs[topic].publish(m);rclpy.spin_once(node,timeout_sec=0)
    deadline=time.monotonic()+1
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
   finally:
    if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=15)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
  text=log.read_text();stamps=[r[1] for r in rows]
  expected_end=2*period+events[-1][0]
  checks=dict(sync_branch_exercised='Self sync IMU and LiDAR' in text,sustained_odometry=len(rows)>=100 and rows[-1][0]-begin>expected_end-.75,sustained_cloud=len(clouds)>=100,source_order=bool(stamps) and all(b>a for a,b in zip(stamps,stamps[1:])),valid_pose=bool(rows) and all(r[2]=='camera_init' and r[3]=='body' and all(math.isfinite(x) for x in r[4]) and abs(sum(x*x for x in r[4][3:])-1)<.01 for r in rows),clean_exit=p.returncode==0)
  results[label]=dict(checks=checks,odometry=len(rows),clouds=len(clouds),last_output_seconds=rows[-1][0]-begin if rows else None,expected_end_seconds=expected_end,returncode=p.returncode)
  (out/(label+'-odometry.json')).write_text(json.dumps(rows))
  print(json.dumps({label:results[label]}),flush=True)
  deadline=time.monotonic()+1
  while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
 checks=dict(original_reproduces_stall=results['original']['checks']['sync_branch_exercised'] and not results['original']['checks']['sustained_odometry'],candidate_all_checks=all(results['candidate']['checks'].values()))
 result=dict(passed=all(checks.values()),checks=checks,cases=results,imu_clock_offset_seconds=-2,scope='recorded LiDAR/IMU with controlled clock offset; localhost domain84; no FC')
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
finally:
 node.destroy_node();rclpy.try_shutdown()
if not result['passed']:raise SystemExit(1)
