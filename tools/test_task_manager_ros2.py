#!/usr/bin/env python3
"""Real ROS2 graph + native idle waypoint, fake FC; domain87/localhost only."""
import os,sys,time,json,math,subprocess,signal,importlib.util
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='87' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
base=Path('/home/orangepi/drone_platform/01_apps/mission_demo')
spec=importlib.util.spec_from_file_location('manager',base/'tools/gcs_task_manager.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State,PositionTarget
from mavros_msgs.srv import SetMode,CommandBool
from std_msgs.msg import String
from yolo_ultra_test.msg import TrackedTarget
from rclpy.qos import QoSProfile,DurabilityPolicy
ros=mod.RosState();node=ros.node
root=Path('/tmp/task-manager-validation');root.mkdir(exist_ok=True)
manager=mod.Manager(base,root/'store',ros)
results=[];calls=[];commands=[]
def check(name,value):
 results.append(dict(name=name,passed=bool(value)));print(('PASS ' if value else 'FAIL ')+name,flush=True)
def rejects(name,action):
 try:action()
 except (ValueError,RuntimeError):check(name,True)
 else:check(name,False)
def wait(test,timeout=5.):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  if test():return True
  time.sleep(.04)
 return False
values=dict(mode='POSCTL',armed=False,sendstate=True,stateage=0.,sendposes=True,poseage=0.,trackage=0.,sendtrack=True,
 markerage=0.,sendmarker=True,pair=True,markerframe='gimbal_camera_optical_frame',sendlanding=True,landingage=0.)
pubs={k:node.create_publisher(t,n,10) for k,t,n in [('state',State,'/mavros/state'),('lio',Odometry,'/factory_fastlio/odometry'),('fc',PoseStamped,'/mavros/local_position/pose'),('marker',PoseStamped,'/aruco_poseresult'),('detector',String,'/aruco_detector/metrics'),('track',TrackedTarget,'/Safina_track_stamped')]}
latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
pubs['schema']=node.create_publisher(String,'/aruco_landing/status_header',latched)
pubs['landing']=node.create_publisher(String,'/aruco_landing/status',latched)
pubs['schema'].publish(String(data='ros_time,phase'))
node.create_subscription(PositionTarget,'/mavros/setpoint_raw/local',lambda m:commands.append(m),100)
def mode(req,res):calls.append(req.custom_mode);res.mode_sent=False;return res
def arm(req,res):calls.append('arm');res.success=False;return res
node.create_service(SetMode,'/mavros/set_mode',mode);node.create_service(CommandBool,'/mavros/cmd/arming',arm)
def stamp(age=0.):return rclpy.time.Time(nanoseconds=max(0,node.get_clock().now().nanoseconds-int(age*1e9))).to_msg()
def send():
 if values['sendstate']:
  m=State();m.header.stamp=stamp(values['stateage']);m.connected=True;m.mode=values['mode'];m.armed=values['armed'];pubs['state'].publish(m)
 if values['sendposes']:
  m=Odometry();m.header.stamp=stamp(values['poseage']);m.header.frame_id='camera_init';m.pose.pose.position.x=1.;m.pose.pose.position.y=2.;m.pose.pose.orientation.w=1.;pubs['lio'].publish(m)
  m=PoseStamped();m.header.stamp=stamp(values['poseage']);m.header.frame_id='map';m.pose.position.x=.5;m.pose.position.y=.5;m.pose.orientation.z=math.sqrt(.5);m.pose.orientation.w=math.sqrt(.5);pubs['fc'].publish(m)
 if values['sendtrack']:
  m=TrackedTarget();m.header.stamp=stamp(values['trackage']);m.header.frame_id='camera_color_optical_frame';m.target_id=7;m.state=2;m.offset_x=.2;m.offset_y=.1;m.distance_m=2.;m.distance_valid=True;m.distance_source=2;m.session_active=True;pubs['track'].publish(m)
 if values['sendmarker']:
  m=PoseStamped();m.header.stamp=stamp(values['markerage']);m.header.frame_id=values['markerframe'];m.pose.position.z=1.;m.pose.orientation.w=1.;pubs['marker'].publish(m)
  ns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
  pubs['detector'].publish(String(data=json.dumps(dict(selected_id=42,reason='selected_pose_valid',source_stamp_known=True,pose_stamp_ns=ns if values['pair'] else ns-10000000))))
 if values['sendlanding']:
  pubs['landing'].publish(String(data=str(node.get_clock().now().nanoseconds*1e-9-values['landingage'])+',IDLE'))
timer=node.create_timer(.03,send)
try:
 check('real ROS2 state discovery',wait(lambda:ros.value('fc') is not None))
 check('stamped tracking retains groundstation schema',wait(lambda:ros.summary()['tracking'].get('id')==7))
 check('ArUco matched source stamps are valid',wait(lambda:ros.summary()['aruco']['valid']))
 check('transient-local landing schema received',wait(lambda:ros.summary()['aruco']['landing'].get('phase')=='IDLE'))
 values['pair']=False;time.sleep(.2);check('mismatched marker metrics rejected',not ros.summary()['aruco']['valid'])
 values.update(pair=True,markerage=3.);time.sleep(.2);check('republished old marker rejected',not ros.summary()['aruco']['valid'])
 values.update(markerage=0.,markerframe='wrong');time.sleep(.2);check('wrong marker frame rejected',not ros.summary()['aruco']['valid'])
 values.update(markerframe='gimbal_camera_optical_frame',landingage=10.);time.sleep(.2);check('old latched landing row rejected',not ros.summary()['aruco']['landing_fresh'])
 values.update(landingage=0.,trackage=3.);time.sleep(.2);check('old stamped tracking omitted',not ros.summary()['tracking'])
 values.update(trackage=0.,stateage=10.);time.sleep(.2);check('old FC source omitted',ros.value('fc') is None)
 values.update(stateage=0.);check('FC recovers with fresh source',wait(lambda:ros.value('fc') is not None))
 session=ros.map_session();check('DDS session has publisher GID',bool(session and session['version']==2 and session['gid']))
 lio,fc=ros.map_alignment();check('alignment matches current publisher',lio['frame']=='camera_init' and fc['frame']=='map')
 original=base/'workspace/src/waypoint_mission/missions/abc_photo.yaml'
 check('native mission validator used',manager.validate(str(original))['ok'])
 import yaml
 cfg=yaml.safe_load(original.read_text())
 payload=dict(version=4,frame='fastlio_world',map_session=session,waypoints=[dict(x=1.,y=3.,z=1.,photo=False)])
 mapped=mod.map_mission(payload,lio,fc,cfg)['mission'];point=mapped['waypoints'][0]
 check('90 degree LIO to FC transform',abs(point['x']+.5)<1e-6 and abs(point['y']-.5)<1e-6 and abs(point['z']-1.)<1e-6)
 check('map mission retains automatic workflow',mapped['manual_airborne'] is False and mapped['return_to_start'] and mapped['auto_land'])
 rejects('nonfinite map input rejected',lambda:mod.map_mission(dict(payload,waypoints=[dict(x=float('nan'),y=0.,z=1.)]),lio,fc,cfg))
 rejects('old map protocol rejected',lambda:mod.map_mission(dict(payload,version=3),lio,fc,cfg))
 duplicate=node.create_publisher(Odometry,'/factory_fastlio/odometry',10);check('multiple LIO publishers invalidate session',wait(lambda:ros.map_session() is None));node.destroy_publisher(duplicate)
 check('session recovers to unique publisher',wait(lambda:ros.map_session() is not None))
 node.destroy_publisher(pubs['lio']);pubs['lio']=node.create_publisher(Odometry,'/factory_fastlio/odometry',10)
 check('publisher restart changes map session',wait(lambda:ros.map_session() is not None and ros.map_session()!=session))
 rejects('old selected map session refused',lambda:manager.start('waypoint','',payload))
 conflict=node.create_publisher(PositionTarget,'/mavros/setpoint_raw/local',10);check('ROS2 control publisher detected',wait(lambda:bool(ros.conflicts())))
 rejects('start refuses competing controller',lambda:manager.start('waypoint',str(original)));node.destroy_publisher(conflict);check('controller removal visible',wait(lambda:not ros.conflicts()))
 values.update(mode='OFFBOARD',armed=True);time.sleep(.2);rejects('start refuses active OFFBOARD',lambda:manager.start('waypoint',str(original)))
 values.update(mode='POSCTL',armed=False);time.sleep(.2)
 for kind in ('waypoint','aruco','tracking'):
  source=mod.launch_python(kind,base,str(original),False)
  file=root/(kind+'.launch.py');file.write_text(source)
  sp=importlib.util.spec_from_file_location('launch_'+kind,file);lm=importlib.util.module_from_spec(sp);sp.loader.exec_module(lm)
  desc=lm.generate_launch_description();check(kind+' native launch loads',bool(desc.entities))
 idle_cfg=yaml.safe_load(original.read_text())
 for point in idle_cfg['mission']['waypoints']:point['photo']=False
 idle_mission=root/'idle-mission.yaml';idle_mission.write_text(yaml.safe_dump(idle_cfg))
 result=manager.start('waypoint',str(idle_mission));record=manager.load()
 check('real native waypoint launch owned',result['ok'] and mod.owned(record))
 check('mission snapshot separate from source',record['mission_snapshot']!=str(idle_mission) and Path(record['mission_snapshot']).read_bytes()==idle_mission.read_bytes())
 time.sleep(.5);check('idle native task sends no commands',not calls and not commands)
 check('same task start is idempotent',manager.start('waypoint',str(idle_mission))['ok'] and manager.load()['pid']==record['pid'])
 rejects('different task requires owned task stop',lambda:manager.start('tracking',''))
 values.update(mode='OFFBOARD',armed=True);time.sleep(.2);manager.stop();time.sleep(1.2)
 check('stop waits for pilot takeover',mod.owned(record) and manager.load()['stop_pending'])
 values.update(mode='POSCTL',armed=True);time.sleep(1.5);check('stop observes stable takeover interval',mod.owned(record))
 check('owned group exits after takeover',wait(lambda:not mod.owned(record),12))
 check('stop completion reports stopped',wait(lambda:not manager.load().get('stop_pending'),3))
 values.update(mode='POSCTL',armed=False);time.sleep(.2)
 current_payload=dict(payload,map_session=ros.map_session())
 mapped_result=manager.start('waypoint','',current_payload);mapped_record=manager.load()
 executed=mod.yaml_data(Path(mapped_record['mission_snapshot']))['mission']
 check('map selection prepares actual native task',mapped_result['ok'] and mod.owned(mapped_record) and executed['name']=='gcs_map')
 point=executed['waypoints'][0]
 check('executed map snapshot has transformed ENU goal',abs(point['x']+.5)<1e-6 and abs(point['y']-.5)<1e-6 and executed['takeoff_z']==1.)
 check('mapped task snapshots alignment evidence',bool(list(Path(mapped_record['run']).glob('alignment.json'))))
 manager.stop();check('mapped idle task stops through owned worker',wait(lambda:not mod.owned(mapped_record),12))
 check('manager never calls control services',not calls and not commands)
 for label,text in [('array','[]'),('NaN','{"v":NaN}'),('overflow','{"v":1e309}')]:
  rejects('malformed JSON '+label+' refused',lambda text=text:mod.json_object(text))
 check('finite JSON preserved',mod.json_object('{"v":1}')=={'v':1})
 with (root/'watch.log').open('w') as watchlog:
  watch=subprocess.Popen([sys.executable,'-B',str(base/'tools/gcs_task_manager.py'),'watch','--workspace',str(base),'--store',str(root/'watch-store')],stdout=watchlog,stderr=watchlog)
  children=Path('/proc')/str(watch.pid)/'task'/str(watch.pid)/'children'
  child=[]
  def find_helper():
   try:
    for pid in children.read_text().split():
     if b'lio_identity' in (Path('/proc')/pid/'cmdline').read_bytes():child.append(int(pid));return True
   except OSError:pass
   return False
  check('watch owns native identity helper',wait(find_helper,6))
  watch.kill();watch.wait()
  check('parent death removes identity helper',not child or wait(lambda:not mod.identity(child[0]),5))

finally:
 try:
  record=manager.load()
  if mod.owned(record):os.killpg(record['pid'],signal.SIGINT)
 except Exception:pass
 node.destroy_timer(timer);ros.close()
 (root/'results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
 print(json.dumps(dict(passed=sum(x['passed'] for x in results),total=len(results))),flush=True)
raise SystemExit(1 if any(not x['passed'] for x in results) else 0)
