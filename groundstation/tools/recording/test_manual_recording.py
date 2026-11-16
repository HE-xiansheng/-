#!/usr/bin/python3
"""真实rosbag的隔离验证：localhost11382、合成遥测，无真实飞控或控制发布。"""
import importlib.util,json,os,signal,subprocess,sys,threading,time
from pathlib import Path
assert os.environ.get('ROS_MASTER_URI')=='http://127.0.0.1:11382'
source=Path('/home/tyzk/diff-planner-comparison-20261016/deploy/ego_planner')
base=Path('/home/tyzk/dianyunGCS-review-20261026/.recording-upgrade')/('isolated-'+str(os.getpid()))
root=base/'workspace';(root/'src').mkdir(parents=True)
for name in ('record_session.py','recording_manage.py'):(root/name).write_bytes((source/name).read_bytes())
spec=importlib.util.spec_from_file_location('isolated_record_manager',root/'recording_manage.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
master=subprocess.Popen(['roscore','-p','11382'],stdout=(base/'master.log').open('w'),stderr=subprocess.STDOUT)
stop=threading.Event();thread=None;info=None;runs=[]
try:
 time.sleep(2)
 import rospy,rosbag,rosgraph
 from mavros_msgs.msg import State,ExtendedState
 from std_msgs.msg import Header
 from sensor_msgs import point_cloud2
 from sensor_msgs.msg import PointCloud2
 rospy.init_node('manual_recording_isolated_test',disable_signals=True)
 state=rospy.Publisher('/mavros/state',State,queue_size=1)
 landed=rospy.Publisher('/mavros/extended_state',ExtendedState,queue_size=1)
 cloud=rospy.Publisher('/factory_fastlio/cloud_world',PointCloud2,queue_size=1)
 values={'mode':'OFFBOARD','armed':True}
 def feed():
  while not stop.is_set():
   stamp=rospy.Time.now();s=State();s.header.stamp=stamp;s.connected=True;s.armed=values['armed'];s.mode=values['mode'];state.publish(s)
   e=ExtendedState();e.header.stamp=stamp;e.landed_state=2 if values['armed'] else 1;landed.publish(e)
   cloud.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id='camera_init'),[(0.,0.,0.),(1.,1.,1.)]));stop.wait(.1)
 thread=threading.Thread(target=feed);thread.start()
 def wait_phase(phases,timeout=65):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   value=m.health(root)
   if value['phase']=='ERROR':raise AssertionError(value)
   if value['phase'] in phases:return value
   time.sleep(.1)
  raise AssertionError('phase timeout: '+str(m.health(root)))
 info=m.prepare(root);idle=wait_phase({'WAIT_MANUAL'});time.sleep(1.5)
 assert not list(Path(idle['run']).glob('*.bag*')),'OFFBOARD alone started recording'
 first=m.request('start',root);active=wait_phase({'RECORDING'});first_run=Path(active['run']);runs.append(first_run)
 time.sleep(1.5);duplicate=m.request('start',root)
 assert duplicate['clip_id']==active['clip_id'],'duplicate start created another bag'
 values.update(mode='POSCTL',armed=False);time.sleep(1.5)
 assert m.health(root)['phase']=='RECORDING','mode exit/landing unexpectedly stopped manual recording'
 values.update(mode='OFFBOARD',armed=True);time.sleep(.3)
 m.request('stop',root);closed=wait_phase({'STOPPED'},45)
 frozen=closed['duration_s'];time.sleep(1);assert m.health(root)['duration_s']==frozen
 assert not list(first_run.glob('*.active')) and list(first_run.glob('*.bag'))
 second=m.request('start',root);active2=wait_phase({'RECORDING'});runs.append(Path(active2['run']));time.sleep(1.2)
 m.request('stop',root);wait_phase({'STOPPED'},45)
 third=m.request('start',root);m.request('stop',root);cancelled=wait_phase({'STOPPED'},65)
 assert not list(Path(third['run']).glob('*.bag*')),'cancel during prepare still recorded a bag'
 orphan=Path(third['run'])/'unfinalized.bag.active';orphan.write_bytes(b'isolated test fixture')
 old=m.current(root)
 try:m.request('start',root)
 except RuntimeError:pass
 else:raise AssertionError('unfinalized bag allowed a new context')
 assert m.current(root)['pid']==old['pid']
 orphan.unlink()
 counts={}
 for run in runs:
  for bag in run.glob('*.bag'):
   with rosbag.Bag(str(bag)) as opened:
    for topic,details in opened.get_type_and_topic_info().topics.items():counts[topic]=counts.get(topic,0)+details.message_count
 assert counts.get('/factory_fastlio/cloud_world',0)>10 and counts.get('/mavros/state',0)>10
 pubs=dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
 assert not any(t.startswith('/mavros/setpoint') or t=='/ego_planner/flight_execution' for t in pubs),'control publisher appeared'
 result={'passed':True,'checks':['OFFBOARD_does_not_auto_start','explicit_start','duplicate_start_singleton','mode_exit_does_not_stop','explicit_stop_in_OFFBOARD','finalized_bags','duration_freezes','repeat_recording','cancel_during_prepare','recorded_real_ROS_messages','no_control_publishers','unfinalized_bag_blocks_duplicate_recording'],
         'runs':[str(x) for x in runs],'topic_counts':counts,'duration_s':frozen}
 (base/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
finally:
 stop.set()
 if thread:thread.join(timeout=2)
 try:
  info=m.current(root)
  if info:
   os.kill(info['pid'],signal.SIGTERM)
   deadline=time.monotonic()+30
   while m.current(root) and time.monotonic()<deadline:time.sleep(.1)
 except (OSError,ValueError):pass
 master.send_signal(signal.SIGINT);master.wait(timeout=15)
