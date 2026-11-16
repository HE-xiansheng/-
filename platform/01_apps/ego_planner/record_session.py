#!/usr/bin/env python3
"""ROS2只读录包监督器；不发布任务、不调用飞控服务。运行数据不同于迁移报告。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import sqlite3
import subprocess
import tarfile
import time
import uuid

TOPICS = [
 '/livox/lidar','/livox/imu','/factory_fastlio/cloud_world','/factory_fastlio/odometry',
 '/mavros/vision_pose/pose','/goal','/move_base_simple/goal','/mandatory_stop_to_planner',
 '/ego_planner/flight_execution','/ego_planner/flight_execution/trajectory',
 '/ego_planner/manual_preview/status','/ego_planner/manual_preview/trajectory',
 '/ego_planner/px4_execution/status','/planning/trajectory','/planning/data_display',
 '/grid_map/occupancy','/grid_map/occupancy_inflate',
 '/mavros/state','/mavros/extended_state','/mavros/estimator_status',
 '/mavros/local_position/pose','/mavros/local_position/odom','/mavros/local_position/velocity_local',
 '/mavros/imu/data','/mavros/rc/in','/mavros/battery','/mavros/setpoint_raw/local',
 '/mavros/setpoint_raw/target_local','/mavros/timesync_status','/mavros/statustext/recv',
 '/tf','/tf_static','/rosout']
RETAINED = {'/tf_static','/ego_planner/flight_execution',
 '/ego_planner/manual_preview/status','/ego_planner/manual_preview/trajectory'}


def atomic(path, value):
    tmp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False));tmp.replace(path)


class FlightTrigger:
    """同时检查源时刻和接收时刻；用于显示和可选录制策略。"""
    def __init__(self):self.state=None;self.landed=None
    @staticmethod
    def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    def on_state(self,msg):self.state=(time.monotonic(),self.stamp(msg),bool(msg.connected),bool(msg.armed),msg.mode)
    def on_landed(self,msg):self.landed=(time.monotonic(),self.stamp(msg),msg.landed_state)
    def check(self,ros_now):
        now=time.monotonic();s=self.state;l=self.landed
        fresh=all(m is not None and m[1]>0 and -.1<=ros_now-m[1]<=2 and 0<=now-m[0]<=2 for m in (s,l))
        d={'fresh':fresh,'connected':s[2] if s else False,'armed':s[3] if s else False,
           'mode':s[4] if s else None,'landed_state':l[2] if l else None,
           'state_stamp':s[1] if s else None,'landed_stamp':l[1] if l else None}
        return bool(fresh and s[2] and s[3] and s[4]=='OFFBOARD' and l[2]==2),d


def archive(root,run):
    # 只归档运行源码/配置；不递归历史录包、模型、构建缓存和迁移文档。
    base=root.parents[1] if root.parent.name=='01_apps' else root
    roots=[root]
    if base!=root:
        roots += [base/'env.sh',base/'01_apps/fast_lio',base/'01_apps/mission_demo/workspace/src',base/'02_drivers/sensors/workspace/src']
    manifest={};suffixes={'.py','.sh','.yaml','.yml','.json','.xml','.cpp','.hpp','.h','.msg','.srv','.cmake','.txt'}
    with tarfile.open(run/'sources.tgz','w:gz') as tar:
        for branch in roots:
            for p in ([branch] if branch.is_file() else branch.rglob('*')):
                if not p.is_file() or p.is_symlink() or any(x in p.parts for x in ('flight-records','recording-auto-logs','install','build','log','.git','__pycache__','ros1_reference')):continue
                if p.suffix not in suffixes and p.name!='CMakeLists.txt':continue
                name=p.relative_to(base).as_posix();manifest[name]=hashlib.sha256(p.read_bytes()).hexdigest()
                tar.add(p,arcname=name,recursive=False)
    binaries={}
    for prefix in os.environ.get('AMENT_PREFIX_PATH','').split(':'):
        if not prefix or not str(base) in prefix:continue
        for p in (Path(prefix)/'lib').rglob('*'):
            if p.is_file() and ('.so' in p.name or os.access(p,os.X_OK)):
                binaries[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    atomic(run/'manifest.json',{'schema':3,'ros':'humble','storage':'sqlite3','cdr':True,
       'stamp':time.time(),'platform':platform.platform(),'architecture':platform.machine(),
       'domain':os.environ.get('ROS_DOMAIN_ID','0'),'topics':TOPICS,'sha256':manifest,'runtime_sha256':binaries,
       'limitations':['录制前的非持久化消息无法补录','DDS接收时刻不等于原回调调度时刻',
          '缺失话题单独报告；RECORDING仅表示录制器就绪，不表示全链数据完整',
          '异常掉电保留active标记，不自动修复或覆盖旧数据']})
    with (run/'runtime.txt').open('w') as out:
        for cmd in (['uname','-a'],['dpkg-query','-W','ros-humble-rosbag2','ros-humble-mavros']):
            subprocess.run(cmd,stdout=out,stderr=subprocess.STDOUT,timeout=10,check=False)


def dump_params(node,path):
    """只读服务采样；不可用节点显式列为error，不伪装成完整快照。"""
    import rclpy
    from rcl_interfaces.srv import ListParameters,GetParameters
    result={};deadline=time.monotonic()+10
    for name,ns in sorted(node.get_node_names_and_namespaces()):
        full=ns.rstrip('/')+'/'+name
        if name.startswith(('rosbag2','flight_record')):continue
        if time.monotonic()>=deadline:
            result[full]={'error':'global snapshot deadline'};continue
        clients=[]
        try:
            ls=node.create_client(ListParameters,full+'/list_parameters');clients.append(ls)
            get=node.create_client(GetParameters,full+'/get_parameters');clients.append(get)
            req=ListParameters.Request();req.depth=0
            if not ls.wait_for_service(timeout_sec=.15):raise RuntimeError('parameter service unavailable')
            future=ls.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=min(.5,max(0,deadline-time.monotonic())))
            if not future.done():future.cancel();raise RuntimeError('list timeout')
            names=future.result().result.names;req=GetParameters.Request();req.names=names
            future=get.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=min(.5,max(0,deadline-time.monotonic())))
            if not future.done():future.cancel();raise RuntimeError('get timeout')
            result[full]={k:{'type':v.type,'value':v.string_value if v.type==4 else v.bool_value if v.type==1 else v.integer_value if v.type==2 else v.double_value if v.type==3 else list(v.byte_array_value) if v.type==5 else list(v.bool_array_value) if v.type==6 else list(v.integer_array_value) if v.type==7 else list(v.double_array_value) if v.type==8 else list(v.string_array_value) if v.type==9 else None} for k,v in zip(names,future.result().values)}
        except Exception as exc:result[full]={'error':str(exc)}
        finally:
            for c in clients:node.destroy_client(c)
    atomic(path,result)


def read_control(run):
    try:
        d=json.loads((run/'recording-control.json').read_text())
        if d.get('action') in ('start','stop') and isinstance(d.get('id'),str):return d
    except (OSError,ValueError):pass
    return {}


def subscriptions(node,recorder):
    return {topic for topic in TOPICS if any(i.node_name==recorder for i in node.get_subscriptions_info_by_topic(topic))}


def finish_bag(proc,run,status,log):
    status.update(phase='STOPPING',stamp=time.time());atomic(run/'recording-status.json',status)
    if proc and proc.poll() is None:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=20)
        except subprocess.TimeoutExpired:raise RuntimeError('rosbag2未在20秒内完成收尾；保留active标记')
    clip=status.get('clip_id');folder=run/('inputs-'+clip) if clip else None
    if proc and proc.returncode not in (0,130,-signal.SIGINT):raise RuntimeError('rosbag2异常退出: '+str(proc.returncode))
    inventory=[]
    if folder:
        if not (folder/'metadata.yaml').is_file():raise RuntimeError('缺少rosbag2 metadata.yaml；未确认收尾')
        result=subprocess.run(['ros2','bag','info',str(folder)],stdout=subprocess.PIPE,stderr=log,timeout=15)
        (folder/'info.txt').write_bytes(result.stdout)
        if result.returncode:raise RuntimeError('ros2 bag info失败')
        for p in sorted(folder.glob('*.db3')):
            with sqlite3.connect(p.as_uri()+'?mode=ro',uri=True) as db:
                if db.execute('PRAGMA quick_check').fetchone()[0]!='ok':raise RuntimeError('SQLite结构检查失败')
                count=db.execute('SELECT count(*) FROM messages').fetchone()[0]
            inventory.append({'file':p.relative_to(run).as_posix(),'bytes':p.stat().st_size,'messages':count,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
        if not inventory:raise RuntimeError('录包中没有SQLite分包')
        (run/('inputs-'+clip+'.active')).unlink(missing_ok=True)
    status.update(phase='STOPPED',bags=inventory,bag_pid=None,bag_bytes=sum(x['bytes'] for x in inventory),active_bags=[],stamp=time.time())
    atomic(run/'recording-status.json',status)
    if clip:atomic(run/('segment-'+clip+'.json'),dict(status))


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from mavros_msgs.msg import State,ExtendedState
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--root',type=Path,required=True);a=p.parse_args()
    run=a.run;run.mkdir(parents=True,exist_ok=True)
    stop=[False]
    rclpy.init();node=rclpy.create_node('flight_record_supervisor_'+str(os.getpid()))
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stop.__setitem__(0,True))
    observed=FlightTrigger()
    node.create_subscription(State,'/mavros/state',observed.on_state,qos_profile_sensor_data)
    node.create_subscription(ExtendedState,'/mavros/extended_state',observed.on_landed,qos_profile_sensor_data)
    status={'schema':3,'policy':'manual','phase':'PREPARING','stamp':time.time(),'run':str(run),'duration_s':0.,'bag_bytes':0,'bag_pid':None,'storage':'sqlite3'}
    proc=None;started=None;last=None
    def save():
        status['stamp']=time.time();atomic(run/'recording-status.json',status)
    with (run/'rosbag.log').open('ab') as log:
        try:
            save()
            if shutil.disk_usage(run).free<2*1024**3:raise RuntimeError('磁盘不足2GiB')
            archive(a.root,run);dump_params(node,run/'params-before.json')
            # BE订阅兼容可靠/非可靠发布；持久化话题显式可靠+transient_local。
            qos={t:{'history':'keep_last','depth':100,'reliability':'reliable' if t in RETAINED else 'best_effort','durability':'transient_local' if t in RETAINED else 'volatile'} for t in TOPICS}
            import yaml
            (run/'qos.yaml').write_text(yaml.safe_dump(qos))
            (run/'storage.yaml').write_text('write:\n  pragmas: ["journal_mode = WAL", "synchronous = NORMAL"]\n')
            status['phase']='WAIT_MANUAL';save()
            while not stop[0] and rclpy.ok():
                rclpy.spin_once(node,timeout_sec=.05)
                try:
                    control=read_control(run);new=bool(control.get('id') and control['id']!=last)
                    if new:last=control['id'];status['request_id']=last
                    if new and control['action']=='stop':
                        if proc:
                            status['duration_s']=time.monotonic()-started;finish_bag(proc,run,status,log);proc=None
                        else:status['phase']='STOPPED'
                    if new and control['action']=='start' and proc is None:
                        if list(run.glob('*.active')):raise RuntimeError('存在未完成录包，拒绝覆盖')
                        if shutil.disk_usage(run).free<2*1024**3:raise RuntimeError('磁盘不足2GiB')
                        status.pop('error',None);status.update(phase='STARTING_BAG',clip_id=last,duration_s=0.,bag_bytes=0,bag_pid=None);save()
                        dump_params(node,run/('params-start-'+last+'.json'))
                        marker=run/('inputs-'+last+'.active');marker.touch(exist_ok=False)
                        recorder='flight_bag_'+str(os.getpid())
                        cmd=['ros2','bag','record','-s','sqlite3','-o',str(run/('inputs-'+last)),
                             '-b',str(max(65536,int(os.environ.get('RECORDING_MAX_BAG_SIZE','268435456')))),'--max-cache-size','67108864','-p','1',
                             '--qos-profile-overrides-path',str(run/'qos.yaml'),
                             '--storage-config-file',str(run/'storage.yaml'),'--node-name',recorder,*TOPICS]
                        proc=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                        started=time.monotonic();status.update(bag_pid=proc.pid,recording_started_wall=time.time());save()
                        deadline=time.monotonic()+12;ready=False
                        while proc.poll() is None and time.monotonic()<deadline and not stop[0]:
                            rclpy.spin_once(node,timeout_sec=.1)
                            if read_control(run).get('action')=='stop':break
                            present={t for t in TOPICS if node.get_publishers_info_by_topic(t)}
                            ready=recorder in [n for n,_ in node.get_node_names_and_namespaces()] and present.issubset(subscriptions(node,recorder)) and any((run/('inputs-'+status['clip_id'])).glob('*.db3'))
                            save()
                            if ready:break
                        if not ready and not stop[0] and read_control(run).get('action')!='stop':raise RuntimeError('rosbag2订阅未在12秒内就绪')
                        status['phase']='RECORDING' if ready else 'STARTING_BAG'
                    if proc and proc.poll() is not None:raise RuntimeError('rosbag2意外退出: '+str(proc.returncode))
                    free=shutil.disk_usage(run).free
                    if proc and free<2*1024**3:raise RuntimeError('磁盘不足2GiB；仅停止录制')
                    if proc:
                        status['duration_s']=time.monotonic()-started
                        status['bag_bytes']=sum(x.stat().st_size for x in (run/('inputs-'+status['clip_id'])).glob('*') if x.is_file())
                    _,flight=observed.check(node.get_clock().now().nanoseconds*1e-9)
                    present={t for t in TOPICS if node.get_publishers_info_by_topic(t)}
                    status.update(free_bytes=free,flight_observed=flight,missing_topics=sorted(set(TOPICS)-present),active_bags=[x.name for x in run.glob('*.active')])
                    with (run/'rosbag.log').open('rb') as reader:
                        reader.seek(max(0,(run/'rosbag.log').stat().st_size-20000));text=reader.read().decode(errors='replace')
                    status['warnings']=[x for x in text.splitlines() if any(w in x.lower() for w in ('lost','dropping','error','exception','warn'))][-10:]
                    if status['missing_topics']:status['warnings'].append('缺少%d个话题；详见recording-status.json'%len(status['missing_topics']))
                    save()
                except Exception as exc:
                    reason=str(exc)
                    if proc:
                        status['duration_s']=time.monotonic()-started
                        try:finish_bag(proc,run,status,log)
                        except Exception as final:reason+='; '+str(final)
                        if proc.poll() is not None:proc=None
                    status.update(phase='ERROR',error=reason,bag_pid=proc.pid if proc else None,active_bags=[x.name for x in run.glob('*.active')]);save()
                time.sleep(.15)
        except Exception as exc:status.update(phase='ERROR',error=str(exc));save()
        finally:
            if proc:
                status['duration_s']=time.monotonic()-started
                try:finish_bag(proc,run,status,log)
                except Exception as exc:status.update(phase='ERROR',error=str(exc));save()
            node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
