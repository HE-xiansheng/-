"""Managed localization tools. No arming, mode, setpoint or parameter-write API."""
import argparse,fcntl,hashlib,json,os,signal,socket,subprocess,sys,time,xmlrpc.client
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent
NODE='/factory_laserMapping';ODOM='/factory_fastlio/odometry';CLOUD='/factory_fastlio/cloud_body'

def ros():
 import rospy
 from mavros_msgs.msg import State
 if os.environ.get('ROS_MASTER_URI')!='http://127.0.0.1:11311':raise RuntimeError('仅允许本机11311机载ROS环境')
 socket.setdefaulttimeout(3)
 try:
  code,message,_=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getSystemState('/factory_master_probe')
  if code!=1:raise RuntimeError(message)
 except Exception as exc:raise RuntimeError('机载ROS master11311不可用，请先启动原有雷达/MAVROS环境: '+str(exc))
 if not rospy.core.is_initialized():rospy.init_node('factory_stack_manager',anonymous=True,disable_signals=True)
 return rospy,State

def graph():
 rospy,_=ros();code,msg,state=rospy.get_master().getSystemState()
 if code!=1:raise RuntimeError(msg)
 return dict(state[0])

def gate():
 rospy,State=ros();s=rospy.wait_for_message('/mavros/state',State,timeout=4)
 if not s.connected or s.armed:raise RuntimeError('需要飞控连接且未解锁')
 pubs=graph();commands=['/mavros/setpoint_raw/attitude','/mavros/setpoint_raw/local','/mavros/setpoint_position/local','/mavros/setpoint_velocity/cmd_vel','/mavros/actuator_control','/mavros/rc/override']
 active={x:pubs[x] for x in commands if pubs.get(x)}
 if active:raise RuntimeError('已有控制指令发布者；本工具拒绝改变定位: '+str(active))
 return s,pubs

def info():
 rospy,State=ros();s=rospy.wait_for_message('/mavros/state',State,timeout=4);pubs=graph();result={'connected':s.connected,'armed':s.armed,'mode':s.mode,'fixed_topic':ODOM,'fixed_publishers':pubs.get(ODOM,[]),'legacy_publishers':pubs.get('/Odometry',[]),'external_odometry_publishers':pubs.get('/mavros/odometry/out',[]),'configuration_id':json.loads((ROOT/'manifest.json').read_text())['configuration_generation']}
 if pubs.get(ODOM):
  from nav_msgs.msg import Odometry
  m=rospy.wait_for_message(ODOM,Odometry,timeout=3);v=m.twist.twist.linear
  result.update(source_age_s=(rospy.Time.now()-m.header.stamp).to_sec(),body_velocity_m_s=[v.x,v.y,v.z],position_m=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z])
 return result

def new_run(kind):
 p=ROOT/'runs'/(time.strftime('%Y%m%d-%H%M%S')+'-'+kind+'-'+str(time.time_ns())[-6:]);p.mkdir(parents=True);return p

def active_file(kind):return ROOT/('active-'+kind+'.json')
def alive(kind):
 p=active_file(kind)
 if not p.exists():return False
 try:
  data=json.loads(p.read_text());pid=data['pid'];cmd=Path('/proc')/str(pid)/'cmdline'
  return cmd.exists() and str(ROOT).encode() in cmd.read_bytes() and b'\x00' in cmd.read_bytes()
 except (OSError,ValueError,KeyError):return False

def spawn(kind,cmd,run):
 if alive(kind):raise RuntimeError(kind+' 已运行，不重复启动')
 with (run/'process.log').open('w') as log:p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 active_file(kind).write_text(json.dumps({'pid':p.pid,'run':str(run),'command':cmd},indent=2)+'\n')
 return p

def verify_build():
 binary=ROOT/'workspace/devel/lib/fast_lio/fastlio_mapping'
 records=ROOT/'workspace/build-records'
 for file,target in [('binary-sha256.txt',binary),('source-sha256.txt',ROOT/'workspace/src/fast_lio/src/laserMapping.cpp')]:
  saved=records/file
  if not saved.exists() or not target.exists():raise RuntimeError('请先执行本工具的机载编译，缺少校验记录 '+file)
  if saved.read_text().split()[0]!=hashlib.sha256(target.read_bytes()).hexdigest():raise RuntimeError('编译产物/源码已变化，请重新机载编译')

def start():
 verify_build()
 s,pubs=gate()
 conflicts={t:pubs[t] for t in [ODOM,'/Odometry','/mavros/odometry/out','/mavros/vision_pose/pose','/mavros/vision_pose/pose_cov'] if pubs.get(t)}
 if conflicts:raise RuntimeError('启动冲突：请先在原终端停止原厂定位/转发。不会自动杀原厂节点。'+str(conflicts))
 binary=ROOT/'workspace/devel/lib/fast_lio/fastlio_mapping'
 if not binary.exists():raise RuntimeError('先执行机载编译')
 run=new_run('localization');logdir=ROOT/'workspace/src/fast_lio/Log';logdir.mkdir(exist_ok=True)
 # Own package path is used for factory runtime logging. Original installation untouched.
 launch=f'''<launch>
<rosparam command="load" file="{ROOT}/config/lidar.yaml"/>
<param name="feature_extract_enable" value="false"/>
<param name="point_filter_num" value="3"/><param name="max_iteration" value="3"/>
<param name="filter_size_surf" value="0.5"/><param name="filter_size_map" value="0.3"/>
<param name="cube_side_length" value="150"/><param name="runtime_pos_log_enable" value="false"/>
<node pkg="fast_lio" type="fastlio_mapping" name="factory_laserMapping" output="screen">
<remap from="/Odometry" to="{ODOM}"/><remap from="/cloud_registered_body" to="{CLOUD}"/>
<remap from="/cloud_registered" to="/factory_fastlio/cloud_world"/>
<remap from="/cloud_effected" to="/factory_fastlio/cloud_effected"/>
<remap from="/Laser_map" to="/factory_fastlio/map"/><remap from="/path" to="/factory_fastlio/path"/>
<remap from="/tf" to="/factory_fastlio/tf"/>
</node></launch>'''
 (run/'localization.launch').write_text(launch)
 p=spawn('localization',['roslaunch',str(run/'localization.launch')],run)
 time.sleep(1)
 if p.poll() is not None:raise RuntimeError((run/'process.log').read_text()[-2500:])
 return {'started':True,'pid':p.pid,'run':str(run),'topic':ODOM,'note':'保持机体静止等待初始化；不启动控制或定位转发'}

def stop_kind(kind):
 if not alive(kind):return {'stopped':False,'reason':kind+' 无本工具运行进程'}
 d=json.loads(active_file(kind).read_text());pid=d['pid'];os.killpg(pid,signal.SIGINT)
 for _ in range(30):
  if not alive(kind):break
  time.sleep(.1)
 if alive(kind):os.killpg(pid,signal.SIGTERM)
 return {'stopped':True,'kind':kind,'pid':pid}

def verify_source():
 verify_build()
 s,pubs=gate()
 if pubs.get(ODOM)!=[NODE]:raise RuntimeError('独立里程计必须只有/factory_laserMapping一个发布者: '+str(pubs.get(ODOM)))
 rospy,_=ros();code,msg,uri=rospy.get_master().lookupNode(NODE)
 if code!=1:raise RuntimeError(msg)
 socket.setdefaulttimeout(3);code,msg,pid=xmlrpc.client.ServerProxy(uri).getPid('/factory_source_check')
 if code!=1:raise RuntimeError(msg)
 expected=(ROOT/'workspace/devel/lib/fast_lio/fastlio_mapping').resolve();actual=(Path('/proc')/str(pid)/'exe').resolve()
 if actual!=expected:raise RuntimeError('节点运行程序不属于本次原厂补丁副本: '+str(actual))
 return {'pid':pid,'binary':str(actual),'sha256':hashlib.sha256(actual.read_bytes()).hexdigest()}

def bench(seconds):
 source=verify_source();pubs=graph()
 for t in ['/mavros/odometry/out','/mavros/vision_pose/pose','/mavros/vision_pose/pose_cov']:
  if pubs.get(t):raise RuntimeError('已有外部定位转发，拒绝重复: '+str(pubs[t]))
 run=new_run('manual-test');(run/'source.json').write_text(json.dumps(source,indent=2)+'\n')
 launch=['python3',str(ROOT/'management/bench-relay.py'),'--live-bench','--seconds',str(seconds),'--report',str(run/'relay.json'),'--odom-topic',ODOM,'--expected-source',NODE]
 # rosbag and relay share this process group, bounded by this worker's deadline.
 worker=ROOT/'management/bench_worker.sh'
 p=spawn('bench',['bash',str(worker),str(run),str(seconds)],run)
 return {'recording_started':True,'pid':p.pid,'seconds':seconds,'run':str(run),'note':'前5秒静止，随后缓慢平移，最后静止；未解锁，仅估计器输入；自动结束'}

def observe(seconds):
 rospy,_=ros();from nav_msgs.msg import Odometry
 source=verify_source();run=new_run('observe');rows=[]
 def cb(m):
  p=m.pose.pose.position;v=m.twist.twist.linear
  rows.append({'stamp':m.header.stamp.to_sec(),'position':[p.x,p.y,p.z],'velocity_body':[v.x,v.y,v.z]})
 sub=rospy.Subscriber(ODOM,Odometry,cb,queue_size=20);end=time.monotonic()+seconds
 while time.monotonic()<end and not rospy.is_shutdown():time.sleep(.05)
 sub.unregister();report={'rows':rows,'source':source,'scope':'Read-only pose/velocity observation, not independent accuracy acceptance'};(run/'observation.json').write_text(json.dumps(report,indent=2)+'\n');return {'samples':len(rows),'run':str(run)}

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['status','start','stop','bench','observe','preview']);p.add_argument('--seconds',type=int,default=30);a=p.parse_args()
 if not 5<=a.seconds<=120:raise SystemExit('测试时长必须5～120秒')
 try:
  lock_handle=None
  if a.action in ('start','stop','bench'):
   lock_handle=(ROOT/'.manager.lock').open('a')
   try:fcntl.flock(lock_handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
   except BlockingIOError:raise RuntimeError('已有定位管理操作，稍后再试')
  if a.action=='status':result=info()
  elif a.action=='start':result=start()
  elif a.action=='stop':gate();result={'bench':stop_kind('bench'),'localization':stop_kind('localization')}
  elif a.action=='bench':result=bench(a.seconds)
  elif a.action=='observe':result=observe(a.seconds)
  else:
   verify_source();result=subprocess.run(['bash',str(ROOT/'management/preview.sh'),str(a.seconds)]).returncode
  print(json.dumps(result,ensure_ascii=False,indent=2))
 except Exception as exc:print('操作未执行/未完成：'+str(exc),file=sys.stderr);sys.exit(1)
