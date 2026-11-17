#!/usr/bin/env python3
"""原生ROS2定位会话管理；只启动所属FAST-LIO与Pose转发，不控制飞行。"""
import argparse,contextlib,fcntl,importlib.util,json,math,os,signal,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
PLATFORM=ROOT.parent.parent
RUNTIME=Path(os.environ.get('LOCALIZATION_RUNTIME_DIR','/tmp/drone-localization-%d'%os.getuid()))
ODOM='/factory_fastlio/odometry'
RELAY_TOPICS=['/mavros/vision_pose/pose','/mavros/vision_pose/pose_cov','/mavros/odometry/out','/mavros/vision_speed/speed_twist']
CONTROL_TOPICS=['/mavros/setpoint_raw/attitude','/mavros/setpoint_raw/local','/mavros/setpoint_position/local','/mavros/setpoint_velocity/cmd_vel','/mavros/actuator_control','/mavros/rc/override']

def load(name,path):
 spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module

def identity(pid):
 try:
  p=Path('/proc')/str(pid);fields=(p/'stat').read_text().split(') ',1)[1].split()
  if fields[0]=='Z':return None
  return {'pid':pid,'ticks':fields[19],'boot':Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
 except (OSError,IndexError):return None

def operate(action,api):
 api.gate();lio,relay=api.inventory()
 if action=='start':
  if not lio or not relay:api.require_idle()
  if not lio:
   if relay:raise RuntimeError('转发仍在运行而定位已退出；请先停止所属旧会话')
   api.start_lio()
  api.wait_source()
  if not relay:api.start_relay()
  return {'ok':True,'action':action,'message':'已复用当前定位与转发' if lio and relay else '定位与转发已启动，保持静止等待100个新测量初始化'}
 if action=='stop':
  api.require_idle()
  if relay:api.stop_relay()
  if lio:api.stop_lio()
  return {'ok':True,'action':action,'message':'定位与转发已停止'}
 raise ValueError('未知操作')

class ExistingStack:
 def __init__(self):
  import rclpy
  from nav_msgs.msg import Odometry
  from mavros_msgs.msg import State
  from rclpy.qos import qos_profile_sensor_data
  self.ros=rclpy;self.node=rclpy.create_node('gcs_localization_manager_'+str(os.getpid()));self.fc=None;self.odom=None
  self.node.create_subscription(State,'/mavros/state',lambda m:setattr(self,'fc',m),qos_profile_sensor_data)
  self.node.create_subscription(Odometry,ODOM,lambda m:setattr(self,'odom',m),qos_profile_sensor_data)
  self.execution=load('localization_execution',PLATFORM/'01_apps/ego_planner/px4_execution/manage.py')
  self.preview=load('localization_preview',PLATFORM/'01_apps/ego_planner/manual_session.py')
  self.launch=ROOT/'launch/localization.launch.py'
  self.binaries={'localization':ROOT/'workspace/install/fast_lio/lib/fast_lio/fastlio_mapping',
                 'relay':PLATFORM/'02_drivers/sensors/workspace/install/to_mavros/lib/to_mavros/vision2mavros'}
  self.discovered=False
 def spin(self,duration,predicate=None):
  end=time.monotonic()+duration
  while time.monotonic()<end:
   self.ros.spin_once(self.node,timeout_sec=.02)
   if predicate and predicate():return True
  return bool(predicate and predicate())
 def endpoints(self,topic):return self.node.get_publishers_info_by_topic(topic)
 def age(self,m):
  if m is None:return math.inf
  ns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
  return (self.node.get_clock().now().nanoseconds-ns)*1e-9 if ns>0 else math.inf
 def gate(self):
  if not self.discovered:self.spin(1.0);self.discovered=True
  self.spin(3,lambda:self.fc is not None and -.1<=self.age(self.fc)<=1.5)
  s=self.fc
  if s is None or not s.connected or s.armed or s.mode=='OFFBOARD' or not s.mode or not -.1<=self.age(s)<=1.5:
   raise RuntimeError('需要新鲜、已连接、未解锁且非OFFBOARD的飞控状态')
  if any(self.endpoints(t) for t in CONTROL_TOPICS):raise RuntimeError('存在控制指令发布者，拒绝改变定位会话')
 def record(self,kind):return RUNTIME/(kind+'.json')
 def owned(self,kind):
  try:
   data=json.loads(self.record(kind).read_text());pid=data['pid']
   if identity(pid)!={k:data[k] for k in ('pid','ticks','boot')}:return None
   args=(Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0')
   token=str(self.launch if kind=='localization' else self.binaries[kind]).encode()
   if token not in args or os.getpgid(pid)!=pid:return None
   return data
  except (OSError,ValueError,KeyError,TypeError):return None
 def executable_owned(self,kind):
  data=self.owned(kind)
  if not data:return False
  for entry in Path('/proc').iterdir():
   if not entry.name.isdigit():continue
   try:
    if os.getpgid(int(entry.name))==data['pid'] and (entry/'exe').resolve()==self.binaries[kind].resolve():return True
   except (OSError,ValueError):pass
  return False
 def verify_publisher(self,topic,kind,name):
  endpoints=self.endpoints(topic)
  if not endpoints:return None
  if len(endpoints)!=1 or endpoints[0].node_name!=name or endpoints[0].node_namespace!='/' or not self.executable_owned(kind):
   raise RuntimeError('定位/转发来源冲突，未接管或停止其他进程：'+topic)
  gid=bytes(endpoints[0].endpoint_gid).hex();data=self.owned(kind)
  if data.get('gid') and data['gid']!=gid:raise RuntimeError('DDS来源已更换，请检查会话：'+topic)
  if not data.get('gid'):
   data['gid']=gid;self.save(kind,data)
  return gid
 def save(self,kind,data):
  file=self.record(kind);temporary=file.with_suffix('.next');temporary.write_text(json.dumps(data));os.replace(temporary,file)
 def inventory(self):
  lio,relay=bool(self.owned('localization')),bool(self.owned('relay'))
  self.verify_publisher(ODOM,'localization','factory_laserMapping')
  self.verify_publisher(RELAY_TOPICS[0],'relay','factory_vision2mavros')
  if any(self.endpoints(t) for t in ['/Odometry']+RELAY_TOPICS[1:]):raise RuntimeError('存在其他里程计/转发来源，未自动替换')
  return lio,relay
 def require_idle(self):
  if self.execution.current() or self.preview.owned() or any(self.endpoints(t) for t in ['/ego_planner/manual_preview/status','/drone_0_planning/trajectory']):
   raise RuntimeError('请先结束避障预览和执行器，再启停定位与转发')
 def spawn(self,kind,args):
  if self.owned(kind):raise RuntimeError('当前所属会话已运行：'+kind)
  if not self.binaries[kind].is_file():raise RuntimeError('运行程序未安装：'+str(self.binaries[kind]))
  run=RUNTIME/(kind+'-'+str(time.time_ns()));run.mkdir()
  env=dict(os.environ,ROS_LOG_DIR=str(run/'ros-logs'))
  with (run/'console.log').open('ab') as log:
   p=subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True,env=env)
  ident=identity(p.pid)
  if not ident:raise RuntimeError('启动后立即退出：'+str(run))
  self.save(kind,dict(ident,kind=kind,run=str(run)))
  if not self.spin(4,lambda:self.executable_owned(kind)):
   raise RuntimeError('启动未确认，保留会话供检查：'+str(run))
 def start_lio(self):
  self.gate()
  readiness=load('localization_sensor_readiness',ROOT/'sensor_readiness.py')
  readiness.wait_for_sensors()
  self.gate();self.spawn('localization',['ros2','launch',str(self.launch)])
 def wait_source(self):
  self.odom=None
  end=time.monotonic()+25
  while time.monotonic()<end:
   self.spin(.1)
   if not self.owned('localization'):raise RuntimeError('FAST-LIO已退出；未启动转发')
   self.verify_publisher(ODOM,'localization','factory_laserMapping')
   m=self.odom
   if m is None:continue
   p,q=m.pose.pose.position,m.pose.pose.orientation;v=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
   if m.header.frame_id=='camera_init' and m.child_frame_id=='body' and all(math.isfinite(x) for x in v) and .81<=sum(x*x for x in v[3:])<=1.21 and -.02<=self.age(m)<=.2:return
  raise RuntimeError('定位无有效新鲜输出；定位进程保留，未启动新转发。请检查雷达/IMU并保持静止')
 def start_relay(self):
  self.gate();self.inventory()
  if not self.owned('localization'):raise RuntimeError('请先启动所属定位会话')
  self.spawn('relay',[str(self.binaries['relay'])])
 def stop_kind(self,kind):
  self.gate();data=self.owned(kind)
  if not data:return
  os.killpg(data['pid'],signal.SIGINT)
  if not self.spin(10,lambda:not self.owned(kind)):raise RuntimeError('会话未正常退出；未强制终止：'+kind)
  self.record(kind).unlink(missing_ok=True)
 def stop_relay(self):self.stop_kind('relay')
 def stop_lio(self):self.stop_kind('localization')
 def status(self):
  self.spin(1.)
  return {'ok':True,'lio_running':bool(self.owned('localization')),'relay_running':bool(self.owned('relay')),'ros_version':2}
 def close(self):self.node.destroy_node()

@contextlib.contextmanager
def locks():
 execution=load('lock_execution',PLATFORM/'01_apps/ego_planner/px4_execution/manage.py')
 preview=load('lock_preview',PLATFORM/'01_apps/ego_planner/manual_session.py')
 with contextlib.ExitStack() as stack:
  for directory in [RUNTIME,execution.RUNTIME,preview.RUNTIME]:
   directory.mkdir(parents=True,exist_ok=True)
   handle=stack.enter_context((directory/'management.lock').open('a'))
   try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
   except BlockingIOError:raise RuntimeError('已有定位、规划或执行器管理操作，请稍后重试')
  yield

def main():
 parser=argparse.ArgumentParser();parser.add_argument('action',choices=['start','stop','status']);args=parser.parse_args()
 import rclpy
 rclpy.init(args=[]);api=None
 try:
  with locks():
   api=ExistingStack();result=api.status() if args.action=='status' else operate(args.action,api)
  print(json.dumps(result,ensure_ascii=False,allow_nan=False),flush=True)
 finally:
  if api:api.close()
  rclpy.shutdown()

if __name__=='__main__':
 try:main()
 except Exception as e:print(json.dumps({'ok':False,'message':str(e)},ensure_ascii=False),flush=True);sys.exit(1)
