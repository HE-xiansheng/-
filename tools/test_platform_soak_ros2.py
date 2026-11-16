#!/usr/bin/env python3
"""Actual camera/NPU/LIO/MPP plus recorder under concurrent, non-flight load."""
import csv,gzip,json,math,os,re,signal,sqlite3,struct,subprocess,sys,threading,time
from pathlib import Path
import yaml
out=Path(sys.argv[1]);assert out==Path('/tmp/platform-soak-check')
assert os.environ['ROS_DOMAIN_ID']=='83' and os.environ['ROS_LOCALHOST_ONLY']=='1'
root=Path('/home/orangepi/drone_platform');sys.path.insert(0,str(out/'recording-source'))
import recording_manage as recording
from validate_fastlio_replay import decode
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image,Imu,PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from livox_ros_driver2.msg import CustomMsg
from collections import defaultdict
os.environ['RECORDING_MAX_BAG_SIZE']='33554432'
rclpy.init();node=rclpy.create_node('platform_concurrent_observer')
processes=[];logs=[];stats=defaultdict(list);checks={};resources=[];halt=threading.Event();feed_errors=[];pubs={};sent=defaultdict(list);record_root=out/'recording-source';completed=False

def start(name,args,env=None):
 f=(out/(name+'.log')).open('w');logs.append(f);p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,cwd=out,env=env);processes.append((name,p));return p
def stop(p,seconds=20):
 if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=seconds)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
def check(name,value):checks[name]=bool(value);print(json.dumps({name:bool(value)}),flush=True)
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def receive(key,m):
 ns=stamp(m);now=time.monotonic_ns();age=(time.time_ns()-ns)/1e9
 row=[now,ns,age,m.header.frame_id]
 if isinstance(m,Image):row += [m.width,m.height,m.encoding,len(m.data)]
 elif isinstance(m,Odometry):
  p=m.pose.pose.position;q=m.pose.pose.orientation
  row += [p.x,p.y,p.z,q.x,q.y,q.z,q.w,m.child_frame_id]
 stats[key].append(row)
for key,typ,topic in [('color',Image,'/camera/color/image_raw'),('depth',Image,'/camera/depth/image_raw'),('preview',Image,'/manual_person_follow/image'),('odom',Odometry,'/factory_fastlio/odometry'),('cloud',PointCloud2,'/factory_fastlio/cloud_world'),('vision',PoseStamped,'/mavros/vision_pose/pose')]:
 node.create_subscription(typ,topic,lambda m,k=key:receive(k,m),QoSProfile(depth=20))
with gzip.open(out/'livox.jsonl.gz','rt') as f:messages=[decode(json.loads(s)) for s in f]
imu_stamps=[stamp(m) for topic,_,m in messages if topic=='/livox/imu'];minimum,maximum=min(imu_stamps),max(imu_stamps);events=[]
for topic,_,m in messages:
 ns=stamp(m);event=ns+(max((p.offset_time for p in m.points),default=0) if topic=='/livox/lidar' else 0)
 if topic=='/livox/imu' or minimum<=ns<=event<=maximum:events.append(((event-minimum)/1e9,topic,m,ns-minimum))
events.sort(key=lambda r:r[0]);period=(maximum-minimum)/1e9+.12
pubs={'/livox/lidar':node.create_publisher(CustomMsg,'/livox/lidar',10),'/livox/imu':node.create_publisher(Imu,'/livox/imu',200)}
def feed():
 try:
  epoch=time.time_ns()+100_000_000;begin=time.monotonic()+.1;cycle=0
  while not halt.is_set():
   for offset,topic,m,relative in events:
    if halt.wait(max(0,begin+cycle*period+offset-time.monotonic())):return
    ns=epoch+round(cycle*period*1e9)+relative;m.header.stamp.sec=ns//10**9;m.header.stamp.nanosec=ns%10**9
    if topic=='/livox/lidar':m.timebase=ns
    pubs[topic].publish(m);sent[topic].append(ns)
   cycle+=1
 except Exception as exc:feed_errors.append(repr(exc))
def pump(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  for name,p in processes:
   if p.poll() is not None:raise RuntimeError(name+' unexpectedly exited '+str(p.returncode))
  if feed_errors:raise RuntimeError(feed_errors)
  rclpy.spin_once(node,timeout_sec=.01)
def resource():
 mem={s.split(':')[0]:int(s.split()[1])*1024 for s in Path('/proc/meminfo').read_text().splitlines() if s.split(':')[0] in ['MemTotal','MemAvailable']}
 temps={p.parent.name:int(p.read_text())/1000 for p in Path('/sys/class/thermal').glob('thermal_zone*/temp')}
 rss={}
 for name,p in processes:
  try:
   for s in (Path('/proc')/str(p.pid)/'status').read_text().splitlines():
    if s.startswith('VmRSS:'):rss[name]=int(s.split()[1])*1024
  except OSError:pass
 resources.append(dict(monotonic_ns=time.monotonic_ns(),cpu=list(map(int,Path('/proc/stat').read_text().splitlines()[0].split()[1:])),memory=mem,temperature_c=temps,rss=rss))
thread=None;observer=None;recorder=None;begin_ns=end_ns=None
try:
 config=yaml.safe_load((root/'03_tools/gimbal_relay/gimbal.yml').read_text());config['rtspAddress']='127.0.0.1:18554';config['rtspTransports']=['tcp'];config['paths']={'gimbal':{'source':'publisher'}};config['authInternalUsers']=[{'user':'any','ips':['127.0.0.1'],'permissions':[{'action':'publish','path':'gimbal'},{'action':'read','path':'gimbal'}]}]
 (out/'stream.yml').write_text(yaml.safe_dump(config))
 p=subprocess.run(['ffmpeg','-hide_banner','-loglevel','warning','-f','lavfi','-i','testsrc2=size=1920x1080:rate=30','-t','3','-an','-c:v','libx264','-preset','ultrafast','-tune','zerolatency','-threads','2','-pix_fmt','yuv420p','-g','30','-y',str(out/'video.mp4')],capture_output=True,timeout=40);(out/'encode.log').write_bytes(p.stdout+p.stderr);assert p.returncode==0
 start('stream-server',[str(root/'03_tools/gimbal_relay/mediamtx'),str(out/'stream.yml')]);pump(.5)
 publisher=start('video-source',['ffmpeg','-hide_banner','-loglevel','warning','-re','-stream_loop','-1','-i',str(out/'video.mp4'),'-an','-c:v','copy','-f','rtsp','-rtsp_transport','tcp','rtsp://127.0.0.1:18554/gimbal'])
 observer=start('native-observer',[str(out/'observer-build/observer'),'--ros-args','-p','output:='+str(out/'gimbal-headers.csv'),'-p','stop_file:='+str(out/'observer.stop'),'-p','include_camera:=true'])
 start('mpp',['ros2','run','rtsp_test','rtsp_hw_pub_ultra','--ros-args','-p','rtsp_url:=rtsp://127.0.0.1:18554/gimbal','--log-level','rtsp_hw_pub_ultra.frames:=debug'])
 camera_env=None
 if os.environ.get('PLATFORM_SOAK_CAMERA_PROFILE')=='1':
  camera_env=dict(os.environ)
  camera_env['FASTRTPS_DEFAULT_PROFILES_FILE']=str(root/'01_apps/mission_demo/workspace/install/rtsp_test/share/rtsp_test/config/fastdds_large_images.xml')
  assert Path(camera_env['FASTRTPS_DEFAULT_PROFILES_FILE']).is_file()
  (out/'camera-profile.json').write_text(json.dumps({'FASTRTPS_DEFAULT_PROFILES_FILE':camera_env['FASTRTPS_DEFAULT_PROFILES_FILE']}))
 camera_launch=os.environ.get('PLATFORM_SOAK_CAMERA_LAUNCH',str(root/'02_drivers/orbbec/launch/gemini335l.launch.py'))
 start('camera',['ros2','launch',camera_launch],env=camera_env)
 start('npu',['ros2','run','yolo_ultra_test','person_preview'])
 start('lio',['ros2','launch',str(root/'01_apps/fast_lio/launch/localization.launch.py')])
 start('relay',['ros2','run','to_mavros','vision2mavros'])
 thread=threading.Thread(target=feed);thread.start()
 # Wait for data readiness; never replace missing input with synthetic odometry.
 deadline=time.monotonic()+55
 while time.monotonic()<deadline and not all(len(stats[k])>10 for k in ['color','depth','preview','odom','cloud','vision']):pump(.2)
 check('all_actual_processing_streams_ready',all(len(stats[k])>10 for k in ['color','depth','preview','odom','cloud','vision']))
 recording.request('start',record_root)
 deadline=time.monotonic()+35
 while time.monotonic()<deadline and recording.health(record_root)['phase'] not in ['RECORDING','ERROR']:pump(.2)
 h=recording.health(record_root);check('actual_recorder_ready',h['phase']=='RECORDING');recorder=recording.current(record_root)
 begin_ns=time.time_ns();sample_begin=time.monotonic_ns();duration=float(os.environ.get('PLATFORM_SOAK_SECONDS','180'));deadline=time.monotonic()+duration
 while time.monotonic()<deadline:
  pump(min(5,max(0,deadline-time.monotonic())));resource();h=recording.health(record_root)
  if h['phase']!='RECORDING':raise RuntimeError(h)
  print(json.dumps(dict(remaining_s=round(deadline-time.monotonic(),1),streams={k:len(v) for k,v in stats.items()},bag_bytes=h.get('bag_bytes'),temperature_c=resources[-1]['temperature_c'])),flush=True)
 end_ns=time.time_ns();sample_end=time.monotonic_ns()
 # The recorder remains alive while publishers drain; inspect a stable interior.
 recording.request('stop',record_root);deadline=time.monotonic()+35
 while time.monotonic()<deadline and recording.health(record_root)['phase'] not in ['STOPPED','ERROR']:pump(.2)
 h=recording.health(record_root);check('recorder_normal_finalization',h['phase']=='STOPPED' and not h.get('active_bags'))
 interior=(begin_ns+1_000_000_000,end_ns-1_000_000_000)
 stream_results={}
 for key,min_rate in [('color',20),('depth',20),('preview',5),('odom',7),('cloud',7),('vision',7)]:
  rows=[r for r in stats[key] if sample_begin<=r[0]<=sample_end];times=[r[0] for r in rows];stamps=[r[1] for r in rows];ages=[r[2] for r in rows]
  hz=(len(times)-1)*1e9/(times[-1]-times[0]) if len(times)>1 else 0;max_gap=max([(b-a)/1e9 for a,b in zip(times,times[1:])],default=999)
  stream_results[key]=dict(messages=len(rows),hz=hz,max_gap_s=max_gap,min_age_s=min(ages,default=None),max_age_s=max(ages,default=None))
  check(key+'_sustained_rate',hz>=min_rate);check(key+'_source_order',bool(stamps) and all(b>a for a,b in zip(stamps,stamps[1:])));check(key+'_no_long_gap',max_gap<2)
  check(key+'_ages_bounded',bool(ages) and min(ages)>-.1 and max(ages)<2)
 # Camera/preview source identity is checked with the independent native
 # observer below; Python callback drops must not be mistaken for stamp rewrites.
 check('python_camera_preview_source_stamps_nonempty',bool(stats['preview']) and bool(stats['color']))
 odom_stamps=set(r[1] for r in stats['odom']);check('relay_uses_actual_lio_stamps',bool(stats['vision']) and set(r[1] for r in stats['vision'])<=odom_stamps)
 check('lio_valid_frames_and_finite_unit_quaternion',bool(stats['odom']) and all(r[3]=='camera_init' and r[11]=='body' and all(math.isfinite(x) for x in r[4:11]) and abs(sum(x*x for x in r[7:11])-1)<.01 for r in stats['odom']))
 bag_stamps=defaultdict(set);integrity=[]
 for db_path in Path(h['run']).glob('inputs-*/*.db3'):
  with sqlite3.connect(db_path.as_uri()+'?mode=ro',uri=True) as db:
   integrity.append(db.execute('PRAGMA quick_check').fetchone()[0]=='ok')
   for name,payload in db.execute('SELECT topics.name,messages.data FROM messages JOIN topics ON topics.id=messages.topic_id WHERE topics.name IN (?,?,?,?,?)',('/livox/lidar','/livox/imu','/factory_fastlio/odometry','/factory_fastlio/cloud_world','/mavros/vision_pose/pose')):
    sec,nsec=struct.unpack_from('<iI',payload,4);bag_stamps[name].add(sec*10**9+nsec)
 check('all_recorded_sqlite_parts_valid',bool(integrity) and all(integrity))
 expected={**dict(sent),**{topic:[r[1] for r in stats[key]] for topic,key in [('/factory_fastlio/odometry','odom'),('/factory_fastlio/cloud_world','cloud'),('/mavros/vision_pose/pose','vision')]}}
 coverage={}
 for name,stamps in expected.items():
  wanted={ns for ns in stamps if interior[0]<=ns<=interior[1]};found=bag_stamps[name]&wanted;coverage[name]=dict(expected=len(wanted),recorded=len(found),missing=len(wanted-found));check(name+'_all_stable_source_stamps_recorded',bool(wanted) and wanted<=bag_stamps[name])
 check('no_flight_command_interfaces',all(not node.get_publishers_info_by_topic(t) for t in ['/mavros/setpoint_raw/local','/ego_planner/flight_execution','/Safina_track_stamped']) and not any(n in ['/mavros/set_mode','/mavros/cmd/arming'] for n,_ in node.get_service_names_and_types()))
 check('memory_headroom',bool(resources) and min(r['memory']['MemAvailable'] for r in resources)>512*1024**2)
 completed=True
finally:
 halt.set()
 if thread:thread.join(5)
 if recorder:
  try:
   recording.request('stop',record_root)
   current=recording.current(record_root)
   if current:os.kill(current['pid'],signal.SIGTERM)
  except (ProcessLookupError,RuntimeError):pass
 # Stop video source first and drain, preserving published/received comparisons.
 for name,p in processes:
  if name=='video-source' and p.poll() is None:stop(p)
 until=time.monotonic()+2
 while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
 for name,p in reversed(processes):
  if name=='native-observer':continue
  if p.poll() is None:stop(p)
 time.sleep(.5)
 if observer and observer.poll() is None:(out/'observer.stop').touch();observer.wait(timeout=10)
 for f in logs:f.close()
 trace=(out/'mpp.log').read_text() if (out/'mpp.log').exists() else '';published={key:[int(v) for v in re.findall('published_'+key+r' stamp_ns=(\d+)',trace)] for key in ['raw','color']}
 headers=list(csv.DictReader((out/'gimbal-headers.csv').open())) if (out/'gimbal-headers.csv').exists() else []
 for qos in ['reliable','best_effort']:
  for key in ['raw','color']:
   seen=[int(r['stamp_ns']) for r in headers if r['qos']==qos and r['key']==key];check('mpp_'+qos+'_'+key+'_all_source_stamps',bool(seen) and len(seen)==len(set(seen))==len(published[key]) and set(seen)==set(published[key]))
 native_camera={key:{int(r['stamp_ns']) for r in headers if r['qos']=='reliable' and r['key']==key} for key in ['stereo','person']}
 native_preview_match=bool(native_camera['person']) and native_camera['person']<=native_camera['stereo']
 result=dict(passed=completed and all(checks.values()),completed=completed,checks=checks,resources=resources,returncodes={n:p.returncode for n,p in processes},input_counts={k:len(v) for k,v in sent.items()},mpp_published_counts={k:len(v) for k,v in published.items()},scope='actual Gemini/NPU/LIO/relay/MPP/recorder, repeated .235 recorded LiDAR and synthetic H264, localhost domain83; no physical MAVROS/FC/task controllers')
 for name in ['stream_results','coverage','begin_ns','end_ns','duration']:
  if name in locals():result[name]=locals()[name]
 for key,rows in stats.items():
  with (out/(key+'-headers.jsonl')).open('w') as f:
   for row in rows:f.write(json.dumps(row)+'\n')
 from analyze_soak_preview_headers import analyze
 result['preview_header_coverage']=analyze(out)
 result['preview_header_coverage']['native_only_match']=native_preview_match
 check('npu_preserves_independently_observed_camera_headers',result['preview_header_coverage']['passed'])
 result['passed']=completed and all(checks.values())
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');node.destroy_node();rclpy.try_shutdown();print(json.dumps(dict(passed=result['passed'],checks=checks)),flush=True)
if not result['passed']:raise SystemExit(1)
