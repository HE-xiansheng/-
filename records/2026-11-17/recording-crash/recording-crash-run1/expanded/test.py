#!/usr/bin/env python3
"""Kill only owned synthetic recorder processes; recover a copy, never original data."""
import csv,hashlib,json,os,signal,sqlite3,subprocess,sys,threading,time,shutil
from pathlib import Path
out=Path(sys.argv[1]);assert out==Path('/tmp/recording-crash-check')
assert os.environ['ROS_DOMAIN_ID']=='89' and os.environ['ROS_LOCALHOST_ONLY']=='1'
root=out/'source';sys.path.insert(0,str(root));import recording_manage as manager
import rclpy,rosbag2_py
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rclpy.serialization import serialize_message,deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField
os.environ['RECORDING_MAX_BAG_SIZE']='1048576'
rclpy.init();node=rclpy.create_node('recorder_crash_synthetic_source')
q=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT)
pubs={name:node.create_publisher(typ,name,q) for name,typ in [('/factory_fastlio/odometry',Odometry),('/factory_fastlio/cloud_world',PointCloud2)]}
checks={};result={};source={};stop=threading.Event();worker=None;bag_pid=None

def check(name,value):
 checks[name]=bool(value);print(json.dumps({name:checks[name]}),flush=True)
 if not value:raise AssertionError(name)
def wait_phase(phase):
 end=time.monotonic()+35
 while time.monotonic()<end:
  health=manager.health(root)
  if health['phase']==phase:return health
  if health['phase']=='ERROR':raise RuntimeError(health)
  time.sleep(.1)
 raise TimeoutError(manager.health(root))
def inventory(path):
 return {str(p.relative_to(path)):hashlib.sha256(p.read_bytes()).hexdigest() for p in path.rglob('*') if p.is_file()}
def feed():
 seq=0
 while not stop.is_set():
  seq+=1;stamp=node.get_clock().now().to_msg()
  o=Odometry();o.header.stamp=stamp;o.header.frame_id='camera_init';o.child_frame_id='body';o.pose.pose.position.x=seq*.01;o.pose.pose.orientation.w=1.
  messages=[('/factory_fastlio/odometry',o)]
  if seq%5==0:
   c=PointCloud2();c.header=o.header;c.height=1;c.width=12000;c.point_step=16;c.row_step=c.width*c.point_step
   c.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(['x','y','z','intensity'])]
   import struct
   c.data=struct.pack('<ffff',seq*.01,2.,3.,.7)*c.width;messages.append(('/factory_fastlio/cloud_world',c))
  for name,msg in messages:
   digest=hashlib.sha256(serialize_message(msg)).hexdigest();source[(name,digest)]=stamp.sec*10**9+stamp.nanosec;pubs[name].publish(msg)
  rclpy.spin_once(node,timeout_sec=.001);stop.wait(.019)
thread=threading.Thread(target=feed);thread.start()
try:
 manager.request('start',root);health=wait_phase('RECORDING');run=Path(health['run']);worker=manager.current(root)
 bag_pid=health['bag_pid'];check('owned_recorder_identity',worker and manager.identity(bag_pid)[0] is not None and os.getpgid(bag_pid)==bag_pid)
 # Allow several real split databases plus an unfinished WAL-backed tail.
 time.sleep(8)
 check('records_before_crash',manager.health(root)['phase']=='RECORDING')
 os.killpg(worker['pid'],signal.SIGKILL);os.killpg(bag_pid,signal.SIGKILL)
 end=time.monotonic()+5
 while (manager.current(root) or manager.identity(bag_pid)[0] is not None) and time.monotonic()<end:time.sleep(.05)
 check('both_owned_processes_terminated',manager.current(root) is None and manager.identity(bag_pid)[0] is None)
 health=manager.health(root);check('crash_is_reported_as_error',not health['running'] and health['phase']=='ERROR')
 check('active_marker_retained',bool(list(run.glob('*.active'))))
 before=inventory(run)
 try:manager.request('start',root)
 except RuntimeError as exc:result['restart_rejection']=str(exc);check('new_recording_blocked',True)
 else:check('new_recording_blocked',False)
 check('blocked_start_preserves_all_original_files',before==inventory(run))
 clips=list(run.glob('inputs-*/'));check('one_owned_clip',len(clips)==1);clip=clips[0]
 check('split_databases_exist',len(list(clip.glob('*.db3')))>=2)
 check('unfinished_tail_has_no_final_metadata',not (clip/'metadata.yaml').exists())
 wal_files=[p for p in clip.glob('*.db3-wal') if p.stat().st_size>0];check('wal_sidecar_preserved',bool(wal_files))
 recovery=out/'recovered';shutil.copytree(clip,recovery)
 # Query only the COPY. SQLite must see its WAL before any reindex/checkpoint.
 rows=[];counts={};wal_rows={}
 for path in sorted(recovery.glob('*.db3')):
  with sqlite3.connect(str(path)) as db:
   check('integrity_'+path.name,db.execute('PRAGMA integrity_check').fetchone()[0]=='ok')
   data=db.execute('SELECT topics.name,topics.type,messages.timestamp,messages.data FROM messages JOIN topics ON topics.id=messages.topic_id ORDER BY messages.timestamp').fetchall()
   wal_rows[path.name]=len(data)
   for name,typ,stamp,payload in data:
    digest=hashlib.sha256(payload).hexdigest();assert (name,digest) in source,(name,digest)
    msg=deserialize_message(payload,Odometry if typ=='nav_msgs/msg/Odometry' else PointCloud2)
    assert msg.header.frame_id=='camera_init' and msg.header.stamp.sec*10**9+msg.header.stamp.nanosec==source[(name,digest)]
    if isinstance(msg,PointCloud2):assert len(msg.data)==msg.row_step*msg.height
    rows.append((name,stamp,digest));counts[name]=counts.get(name,0)+1
 check('all_committed_messages_match_real_source_cdr',len(rows)>100 and counts.get('/factory_fastlio/cloud_world',0)>20)
 result.update(committed_counts=counts,database_messages=wal_rows,source_messages=len(source),original_inventory=before)
 p=subprocess.run(['ros2','bag','reindex','-s','sqlite3',str(recovery)],capture_output=True,timeout=45);(out/'reindex.log').write_bytes(p.stdout+p.stderr);check('copied_bag_reindex_succeeds',p.returncode==0 and (recovery/'metadata.yaml').is_file())
 p=subprocess.run(['ros2','bag','info',str(recovery)],capture_output=True,timeout=15);(out/'bag-info.log').write_bytes(p.stdout+p.stderr);check('copied_bag_info_succeeds',p.returncode==0)
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(recovery),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
 read=[]
 while reader.has_next():
  name,payload,stamp=reader.read_next();read.append((name,stamp,hashlib.sha256(payload).hexdigest()))
 del reader
 check('reindexed_reader_exactly_matches_committed_rows',sorted(read)==sorted(rows))
 check('recovery_does_not_modify_originals',before==inventory(run))
 check('original_active_marker_still_requires_manual_review',bool(list(run.glob('*.active'))))
 check('no_flight_command_publishers',all(not node.get_publishers_info_by_topic(t) for t in ['/mavros/setpoint_raw/local','/ego_planner/flight_execution']))
 result['passed']=True
finally:
 stop.set();thread.join(5)
 if worker:
  for pid in [worker['pid'],bag_pid]:
   if pid and manager.identity(pid)[0] is not None:
    try:os.killpg(pid,signal.SIGKILL)
    except ProcessLookupError:pass
 node.destroy_node();rclpy.try_shutdown()
 with (out/'source-messages.csv').open('w') as f:
  writer=csv.writer(f);writer.writerow(['topic','source_stamp_ns','cdr_sha256']);writer.writerows((name,stamp,digest) for (name,digest),stamp in source.items())
 result.update(checks=checks,scope='SIGKILL of owned synthetic recorder processes only; preserves originals and reindexes a copied SQLite/WAL bag; NOT a physical power-cut or durability guarantee')
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(result,indent=2),flush=True)
