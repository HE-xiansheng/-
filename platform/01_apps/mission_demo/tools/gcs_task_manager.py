#!/usr/bin/env python3
"""地面站三项任务入口。只管理本工具拥有的ROS2 launch，不直接生成飞行设定点。"""
import argparse, csv, fcntl, hashlib, json, math, os, re, signal, subprocess, sys, tempfile, time, uuid
from pathlib import Path
import threading

KINDS = ('waypoint', 'aruco', 'tracking')
CONTROL_TOPICS = ('/mavros/setpoint_raw/local', '/mavros/setpoint_position/local',
                  '/mavros/setpoint_raw/attitude', '/mavros/setpoint_velocity/cmd_vel')
HINTS = {'waypoint':'任务已准备；按任务配置拨遥控器OFFBOARD触发，任务会自动起飞。',
         'aruco':'降落已准备；手动起飞后按原配置拨OFFBOARD触发精准降落。',
         'tracking':'跟踪已准备；手动起飞、切OFFBOARD，SWA中档跟随，上档保护、下档下降。'}

def emit(value):
    print(json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)

def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(path.parent), prefix='.gcs-')
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(data, f, ensure_ascii=False, allow_nan=False); f.flush(); os.fsync(f.fileno())
        os.replace(tmp, path)
        fd = os.open(str(path.parent), os.O_DIRECTORY)
        try: os.fsync(fd)
        finally: os.close(fd)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)

def identity(pid):
    try:
        stat = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')', 1)[1].split()
        if stat[0] == 'Z': return None
        return {'pid':pid, 'ticks':stat[19], 'boot':Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
    except (OSError, ValueError, IndexError): return None

def owned(record):
    pid=record.get('pid',-1)
    if isinstance(pid,bool) or not isinstance(pid,int) or pid<=1:return False
    actual=identity(pid)
    if actual:
        return all(actual[k]==record.get(k) for k in ('pid','ticks','boot'))
    # 启动器异常退出而子节点尚在时，仍跟踪原独立会话/进程组；不冒称任务已停。
    try:
        if record.get('boot')!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():return False
        for p in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields=p.read_text().rsplit(')',1)[1].split()
                if fields[0]!='Z' and int(fields[2])==pid and int(fields[3])==pid and p.stat().st_uid==os.getuid():return True
            except (OSError,ValueError,IndexError):pass
    except OSError:pass
    return False

def yaml_data(path):
    import yaml
    data = path.read_bytes()
    if len(data) > 1024*1024 or b'\0' in data: raise ValueError('配置文件过大或含空字节：'+str(path))
    value = yaml.safe_load(data)
    if not isinstance(value, dict): raise ValueError('配置必须是YAML映射：'+str(path))
    return value

def mission_path(value, workspace):
    if not value.strip(): raise ValueError('请填写已按实际场地配置的机载任务YAML路径')
    p = Path(value).expanduser().resolve()
    if not p.is_file() or any(c in str(p) for c in '$\n\r<>'): raise ValueError('机载任务文件路径无效')
    mission = yaml_data(p).get('mission', {})
    points = mission.get('waypoints', [])
    if not isinstance(points, list) or not points: raise ValueError('任务没有航点')
    for point in points:
        for axis in ('x','y','z'):
            v = point.get(axis)
            if isinstance(v, bool) or not isinstance(v, (float,int)) or not math.isfinite(v): raise ValueError('航点XYZ必须是有限数值')
    # 只核对文件结构，速度/高度/围栏等由原生解析器继续校验。
    return p

def launch_python(kind, workspace, mission=None, camera=False):
    """生成节点级参数，禁止以ROS1全局参数隐式改变其他任务。"""
    from ament_index_python.packages import get_package_share_directory
    def config(pkg, name): return str(Path(get_package_share_directory(pkg))/'config'/name)
    actions=[]
    def node(pkg,exe,params=None,remaps=None):
        actions.append('Node(package=%r, executable=%r, output="screen", parameters=%r, remappings=%r)' % (pkg,exe,params or [],remaps or []))
    if camera: node('rtsp_test','rtsp_hw_pub_ultra')
    if kind=='waypoint':
        node('waypoint_mission','waypoint_mission',[dict(mission_file=mission,dry_run=False,bench_mode=False)], [('/Odometry','/factory_fastlio/odometry')])
        node('waypoint_mission','waypoint_photo',[dict(image_topic='/rtsp_cam/image_color')])
    elif kind=='aruco':
        node('aruco_test','Safina_aruco_test',[config('aruco_test','aruco.yaml')])
        node('land_test','Safina_land_test',[config('land_test','takeoff_land.yaml'),dict(gimbal_down_on_startup=True)])
    elif kind=='tracking':
        node('yolo_ultra_test','Safina_yoloUltra_test',[config('yolo_ultra_test','yolo_track.yaml')])
        node('track_test','Safina_track_test',[config('track_test','track.yaml')])
    else: raise ValueError('未知任务')
    return 'from launch import LaunchDescription\nfrom launch_ros.actions import Node\ndef generate_launch_description():\n    return LaunchDescription(['+',\n        '.join(actions)+'])\n'

def pose_data(header,pose):
    return {'stamp':header.stamp.sec+header.stamp.nanosec*1e-9,'stamp_ns':header.stamp.sec*1000000000+header.stamp.nanosec,'frame':header.frame_id,
            'p':[getattr(pose.position,k) for k in ('x','y','z')],
            'q':[getattr(pose.orientation,k) for k in ('x','y','z','w')]}

def map_mission(payload,lio,fc,template):
    """界面地图坐标先转为飞控ENU，再由原生解析器核对速度/围栏/高度。"""
    import numpy as np
    def rotation(q):
        q=np.asarray(q,float)
        if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<1e-9:raise ValueError('姿态无效')
        x,y,z,w=q/np.linalg.norm(q)
        return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                         [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                         [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    if payload.get('version')!=4:raise ValueError('地面站版本过旧，请重启地面站后重新编辑航点')
    if payload.get('frame')!='fastlio_world':raise ValueError('地图航点坐标系无效')
    points=payload.get('waypoints')
    if not isinstance(points,list) or not points:raise ValueError('请先在地图上添加航点')
    R=rotation(fc['q'])@rotation(lio['q']).T
    t=np.asarray(fc['p'],float)-R@np.asarray(lio['p'],float)
    if t.shape!=(3,) or not np.isfinite(t).all():raise ValueError('定位坐标无效')
    # 保留现场YAML的速度、超时、围栏和遥控映射，修改任务语义及航点。
    cfg=dict(template['mission']);cfg.update(name='gcs_map',frame='local',manual_airborne=False,
        return_to_start=True,auto_land=True,waypoints=[])
    for i,p in enumerate(points):
        xyz=[p.get(k) for k in ('x','y','z')]
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in xyz):raise ValueError('航点XYZ必须为有限数值')
        if not isinstance(p.get('photo',False),bool):raise ValueError('拍照选项必须为布尔值')
        x,y,z=(R@np.array(xyz)+t).tolist()
        point=dict(name='P%d'%(i+1),x=x,y=y,z=z,yaw='hold',hold=2.,photo=p.get('photo',False))
        if point['photo']:point.update(gimbal_pitch=-90,gimbal_yaw=0)
        cfg['waypoints'].append(point)
    # 起飞高度必须取坐标变换后的第一个航点本地Z；起飞阶段保持起点XY。
    cfg['takeoff_z']=cfg['waypoints'][0]['z']
    if cfg['takeoff_z'] <= float(fc['p'][2]):
        raise ValueError('第一个航点高度必须高于当前飞控本地高度，无法自动起飞')
    return {'mission':cfg}

class RosState:
    """只读ROS2状态及图；DDS发布端身份与源时刻均参与有效性判断。"""
    def __init__(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
        from mavros_msgs.msg import State, ExtendedState
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String, Int32
        from yolo_ultra_test.msg import TrackedTarget
        self.rclpy=rclpy
        rclpy.init()
        self.node=rclpy.create_node('gcs_flight_task_manager_'+uuid.uuid4().hex[:8])
        self.cache={};self.lock=threading.RLock();self.subs=[];self.closed=False
        def sub(topic,typ,key,convert,latched=False):
            def receive(m):
                try:
                    value=convert(m)
                    h=getattr(m,'header',None)
                    stamp=None if h is None else stamp_seconds(h.stamp)
                    if key=='lio_identity':
                        stamp=value['stamp_ns']*1e-9;value['stamp']=stamp
                    with self.lock:self.cache[key]=(value,time.monotonic(),stamp,None)
                except (ValueError,TypeError,IndexError,KeyError,OverflowError):
                    with self.lock: self.cache.pop(key,None)
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if latched else qos_profile_sensor_data
            subscription=self.node.create_subscription(typ,topic,receive,qos)
            self.subs.append(subscription)
        sub('/mavros/state',State,'fc',lambda m:dict(connected=m.connected,armed=m.armed,mode=m.mode))
        sub('/mavros/extended_state',ExtendedState,'landed',lambda m:m.landed_state)
        sub('/mavros/local_position/pose',PoseStamped,'fc_pose',lambda m:pose_data(m.header,m.pose))
        sub('/factory_fastlio/odometry',Odometry,'lio_pose',lambda m:pose_data(m.header,m.pose.pose))
        sub('/waypoint_mission/status',String,'waypoint',lambda m:parse_fields(m.data))
        sub('/aruco_detector/metrics',String,'detector',lambda m:json_object(m.data))
        sub('/aruco_target_id',Int32,'marker_id',lambda m:m.data)
        sub('/aruco_poseresult',PoseStamped,'marker_pose',lambda m:dict(x=m.pose.position.x,y=m.pose.position.y,z=m.pose.position.z,
            stamp=stamp_seconds(m.header.stamp),stamp_ns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec,frame=m.header.frame_id))
        sub('/aruco_landing/status_header',String,'landing_header',lambda m:next(csv.reader([m.data])),True)
        sub('/aruco_landing/status',String,'landing_row',lambda m:next(csv.reader([m.data])),True)
        sub('/Safina_track_stamped',TrackedTarget,'tracking',lambda m:dict(state=m.state,x_offset=m.offset_x,y_offset=m.offset_y,id=m.target_id,
            distance_m=m.distance_m,distance_source=m.distance_source,distance_valid=m.distance_valid,session_active=m.session_active))
        # Native helper supplies the RMW publisher GID unavailable in Humble rclpy callbacks.
        identity_topic='/gcs_task_manager/session_'+uuid.uuid4().hex+'/lio_identity'
        sub(identity_topic,String,'lio_identity',lambda m:json_object(m.data))
        from ament_index_python.packages import get_package_prefix
        executable=Path(get_package_prefix('platform_gcs_support'))/'lib/platform_gcs_support/lio_identity'
        self.helper=subprocess.Popen([str(executable),'--ros-args','-r','__node:=gcs_identity_'+uuid.uuid4().hex[:8],'-p','identity_topic:='+identity_topic],
            stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,start_new_session=True)
        self.executor=SingleThreadedExecutor();self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.spin,daemon=True);self.thread.start()
    def spin(self):
        from rclpy.executors import ExternalShutdownException
        try:self.executor.spin()
        except ExternalShutdownException:
            # Humble的SIGINT处理可能先关闭context，这是正常的监视退出。
            pass
    def close(self):
        if self.closed:return
        self.closed=True
        if self.helper.poll() is None:
            self.helper.send_signal(signal.SIGINT)
            try:self.helper.wait(timeout=3)
            except subprocess.TimeoutExpired:self.helper.kill();self.helper.wait()
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        self.node.destroy_node();self.rclpy.try_shutdown()
    def value(self,key,max_age=3):
        with self.lock: value,at,stamp,_=self.cache.get(key,(None,0,None,None))
        if time.monotonic()-at>=max_age:return None
        if stamp is not None:
            age=self.node.get_clock().now().nanoseconds*1e-9-stamp
            if stamp<=0 or not math.isfinite(age) or age<-.1 or age>max_age:return None
        return value
    def wait_fc(self):
        end=time.monotonic()+4
        while time.monotonic()<end:
            state=self.value('fc')
            if state and state.get('connected'):return state
            time.sleep(.05)
        raise RuntimeError('无法取得实时飞控连接状态，操作未执行')
    def publishers(self):
        return {topic:[e.node_namespace.rstrip('/')+'/'+e.node_name for e in self.node.get_publishers_info_by_topic(topic)]
            for topic,_ in self.node.get_topic_names_and_types()}
    def map_session(self):
        entries=self.node.get_publishers_info_by_topic('/factory_fastlio/odometry')
        if len(entries)!=1:return None
        e=entries[0]
        if e.topic_type!='nav_msgs/msg/Odometry':return None
        return dict(version=2,boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            domain=os.environ.get('ROS_DOMAIN_ID','0'),gid=bytes(e.endpoint_gid).hex(),node=e.node_namespace.rstrip('/')+'/'+e.node_name)
    def map_alignment(self):
        end=time.monotonic()+3
        while time.monotonic()<end:
            session=self.map_session();a=self.value('lio_identity',.5);b=self.value('fc_pose',.5)
            if self.helper.poll() is not None:raise RuntimeError('定位来源标识辅助进程已退出')
            if session and a and b and a['gid']==session['gid'] and abs(a['stamp']-b['stamp'])<.1:
                validate_pose(a);validate_pose(b)
                if a['frame']!='camera_init' or b['frame']!='map':raise ValueError('定位坐标系与部署配置不一致')
                return a,b
            time.sleep(.02)
        raise RuntimeError('未取得同步的FAST-LIO与飞控位姿，无法转换地图航点')
    def conflicts(self):
        return {t:n for t,n in self.publishers().items() if t in CONTROL_TOPICS and n}
    def summary(self):
        header=self.value('landing_header',86400) or [];row=self.value('landing_row') or []
        landing=dict(zip(header,row)) if len(header)==len(row) else {}
        now=self.node.get_clock().now().nanoseconds*1e-9
        try:
            age=now-float(landing['ros_time'])
            if not math.isfinite(age) or age<-.1 or age>1.5:landing={}
        except (KeyError,TypeError,ValueError):landing={}
        marker=self.value('marker_pose',.5) or {};detector=self.value('detector',.5) or {}
        valid=False;reason='marker_unavailable'
        try:
            valid=(marker['stamp_ns']==detector['pose_stamp_ns'] and marker['stamp']>0 and -.1<=now-marker['stamp']<=.4
                and marker['frame']=='gimbal_camera_optical_frame' and detector['reason']=='selected_pose_valid'
                and isinstance(detector['selected_id'],int) and not isinstance(detector['selected_id'],bool) and detector['selected_id']>=0
                and all(math.isfinite(marker[k]) for k in ('x','y','z')) and marker['z']>0)
            reason='selected_pose_valid' if valid else 'marker_source_or_pair_invalid'
        except (KeyError,TypeError,ValueError):pass
        tracking=self.value('tracking',1) or {}
        return dict(fc=self.value('fc'),map_session=self.map_session(),waypoint=self.value('waypoint') or {},
            aruco=dict(id=detector.get('selected_id',-1) if valid else -1,valid=valid,reason=reason,
                offset={k:marker[k] for k in ('x','y','z')} if valid else {},detector=detector,landing=landing,
                landing_fresh=bool(landing),source_stamp_known=detector.get('source_stamp_known',False)),
            tracking={k:(v if not isinstance(v,float) or math.isfinite(v) else None) for k,v in tracking.items()})

def json_object(data):
    value=json.loads(data)
    if not isinstance(value,dict):raise ValueError('状态JSON不是对象')
    json.dumps(value,allow_nan=False)
    return value

def stamp_seconds(stamp):
    if stamp.sec<0 or stamp.nanosec>=1000000000:raise ValueError('非法源时刻')
    return stamp.sec+stamp.nanosec*1e-9

def validate_pose(value):
    if not all(math.isfinite(x) for x in value['p']+value['q']):raise ValueError('定位含非有限数值')
    if abs(sum(x*x for x in value['q'])-1)>0.05:raise ValueError('定位四元数无效')

def parse_fields(text):
    pairs=list(re.finditer(r'(?:^|\s)(\w+)=',text)); result={}
    for i,m in enumerate(pairs): result[m.group(1)]=text[m.end():pairs[i+1].start() if i+1<len(pairs) else len(text)].strip()
    return result

class Manager:
    def __init__(self, workspace, store, ros):
        self.workspace=Path(workspace).resolve(); self.store=Path(store); self.ros=ros
        self.store.mkdir(parents=True,exist_ok=True)
    def load(self):
        try: return json.loads((self.store/'active.json').read_text())
        except FileNotFoundError: return {}
    def save(self,r): write_json(self.store/'active.json',r)
    def lock(self):
        f=(self.store/'operation.lock').open('a'); fcntl.flock(f,fcntl.LOCK_EX); return f
    def snapshot(self):
        record=self.load(); alive=owned(record)
        return dict(self.ros.summary(), task=record.get('kind',''),running=alive,
                    stop_pending=bool(alive and record.get('stop_pending')),run=record.get('run',''),
                    message=(record.get('message','') if alive or '已停止' in record.get('message','') else ('任务进程已退出，请查看机载任务日志。' if record else '')),log_tail=self.tail(record) if record else '')
    def tail(self,record):
        try:
            run=Path(record['run']).resolve()
            if self.store.resolve() not in run.parents: return ''
            with (run/'task.log').open('rb') as f:
                f.seek(0,2); size=f.tell(); f.seek(max(0,size-2000));return f.read().decode(errors='replace')[-1400:]
        except (OSError,KeyError): return ''
    def validate(self,path):
        mission=mission_path(path,self.workspace)
        run=subprocess.run(['ros2','run','waypoint_mission','waypoint_mission','--ros-args','-r','__node:=gcs_waypoint_validate_'+uuid.uuid4().hex,'-p','dry_run:=true','-p','mission_file:='+str(mission)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=12)
        if run.returncode: raise RuntimeError('原生航点校验失败：'+run.stdout[-1800:])
        config=yaml_data(mission).get('mission',{})
        mode=config.get('manual_airborne')
        if isinstance(mode,bool) and ('manual_airborne: '+str(mode).lower()) not in run.stdout:
            raise RuntimeError('机载航点程序未确认任务起飞模式，请先部署匹配版本')
        if config.get('name')=='gcs_map' and 'return_to_start: true, auto_land: true' not in run.stdout:
            raise RuntimeError('机载航点程序未确认返回起点并自动降落，请先部署匹配版本')
        return {'ok':True,'message':'任务文件已通过原生解析校验','mission_file':str(mission)}
    def start(self,kind,path,payload=None):
        if kind not in KINDS:raise ValueError('未知任务')
        with self.lock():
            record=self.load()
            if owned(record):
                if record.get('kind')==kind and not record.get('stop_pending'):
                    return {'ok':True,'message':'该任务已在运行','state':self.snapshot()}
                raise RuntimeError('请先停止当前任务：'+record.get('kind','unknown'))
            fc=self.ros.wait_fc()
            if fc['armed'] and fc['mode']=='OFFBOARD':raise RuntimeError('正在OFFBOARD控制中，请先遥控接管再准备另一任务')
            conflict=self.ros.conflicts()
            if conflict:raise RuntimeError('已有飞控控制器，请先结束该控制任务：'+json.dumps(conflict,ensure_ascii=False))
            mission=None
            if kind=='waypoint' and payload is None:
                mission=str(mission_path(path,self.workspace))
            mapped=None
            if payload is not None:
                if fc['armed']:raise RuntimeError('自动起飞航点任务要求先落地并锁桨')
                if not payload.get('map_session') or payload['map_session']!=self.ros.map_session():
                    raise RuntimeError('地图定位会话已变化，请清空航点后在当前地图重新选点')
                lio,fc_pose=self.ros.map_alignment()
                if payload['map_session']!=self.ros.map_session():raise RuntimeError('定位会话在转换期间变化，请重新选点')
                template=yaml_data(self.workspace/'workspace/src/waypoint_mission/missions/abc_photo.yaml')
                mapped=map_mission(payload,lio,fc_pose,template)
                if payload['map_session']!=self.ros.map_session():raise RuntimeError('坐标转换期间定位会话变化')
            if kind=='aruco': yaml_data(self.workspace/'workspace/src/land_test/config/takeoff_land.yaml')
            if kind=='tracking':
                yaml_data(self.workspace/'workspace/src/track_test/config/track.yaml');yaml_data(self.workspace/'workspace/src/yolo_ultra_test/config/yolo_track.yaml')
            pubs=self.ros.publishers()
            camera=False
            if kind=='aruco':
                if pubs.get('/aruco_imgresult'):raise RuntimeError('已有其他ArUco检测节点，请先协调该节点，避免重复启动')
                camera=not bool(pubs.get('/rtsp_cam/image_raw'))
            if kind=='tracking' and pubs.get('/Safina_track'):raise RuntimeError('已有其他人物识别节点，请先协调该节点')
            if kind=='waypoint':
                import yaml
                cfg=(mapped or yaml_data(Path(mission)))['mission']
                camera=any(bool(p.get('photo')) for p in cfg.get('waypoints',[])) and not bool(pubs.get('/rtsp_cam/image_color'))
            run=self.store/'runs'/('%s-%s'%(time.strftime('%Y%m%d-%H%M%S'),uuid.uuid4().hex[:8]));run.mkdir(parents=True)
            source_mission=mission
            if mapped:
                write_json(run/'map-request.json',payload)
                write_json(run/'alignment.json',{'fastlio':lio,'mavros':fc_pose})
                # JSON是合法YAML；原子持久化后仍交由原生解析器校验。
                write_json(run/'mission.yaml',mapped)
                mission=str(run/'mission.yaml')
            if mission:
                content=Path(mission).read_bytes()
                snapshot=run/'mission.yaml';snapshot.write_bytes(content)
                with snapshot.open('rb') as f:os.fsync(f.fileno())
                # 原生解析器校验本次实际执行的快照，不执行可能被随后修改的源文件。
                self.validate(str(snapshot));mission=str(snapshot)
            launch=run/'task.launch.py';launch.write_text(launch_python(kind,self.workspace,mission,camera))
            with launch.open('rb') as f:os.fsync(f.fileno())
            with (run/'task.log').open('wb') as log:
                p=subprocess.Popen(['ros2','launch',str(launch)],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            hint='航点任务已准备；遥控切OFFBOARD后自动起飞并执行航线，末点完成后返回起飞点并自动降落。' if mapped else HINTS[kind]
            r=dict(identity(p.pid) or {},kind=kind,run=str(run),stop_pending=False,message=hint,mission_file=source_mission,mission_snapshot=mission)
            self.save(r)
            time.sleep(.4)
            if not owned(r):raise RuntimeError('任务进程启动失败：'+self.tail(r))
            return {'ok':True,'message':hint,'state':self.snapshot()}
    def stop(self):
        with self.lock():
            r=self.load()
            if not owned(r):return {'ok':True,'message':'本入口没有运行中的任务','state':self.snapshot()}
            r['stop_pending']=True;r['message']='已请求停止；如在OFFBOARD，请先用遥控器接管。';self.save(r)
            # 独立停止工作进程不依赖GUI或SSH继续在线。不会发送切模式/解锁请求。
            marker=identity(r.get('stop_worker_pid',-1))
            if not marker or marker!=r.get('stop_worker'):
                with (Path(r['run'])/'stop.log').open('ab') as log:
                    p=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'stop-worker','--workspace',str(self.workspace),'--store',str(self.store)],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                r['stop_worker_pid']=p.pid;r['stop_worker']=identity(p.pid);self.save(r)
            return {'ok':True,'message':r['message'],'state':self.snapshot()}
    def stop_worker(self):
        stable=None
        while True:
            r=self.load()
            if not owned(r) or not r.get('stop_pending'):return
            fc=self.ros.value('fc')
            safe=fc and fc.get('connected') and (not fc['armed'] or fc['mode']!='OFFBOARD')
            stable=(stable or time.monotonic()) if safe else None
            # 等待控制节点也收到接管状态，避免其析构仍按旧OFFBOARD状态请求降落。
            if stable is not None and time.monotonic()-stable>=3:
                with self.lock():
                    current=self.load()
                    if current.get('pid')!=r.get('pid') or not current.get('stop_pending'):return
                    if owned(current): os.killpg(current['pid'],signal.SIGINT)
                deadline=time.monotonic()+12
                while owned(r) and time.monotonic()<deadline:time.sleep(.1)
                if owned(r):
                    # ROS launch可在正常退出期间延长清理；不将仍存活的任务标记为已停止。
                    stable=None;time.sleep(1);continue
                with self.lock():
                    current=self.load()
                    if current.get('pid')==r.get('pid'):
                        current['stop_pending']=False;current['message']='任务已停止；飞控模式由飞手保持。';self.save(current)
                return
            time.sleep(.25)

def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['status','watch','start','start-map','stop','validate','stop-worker'])
    p.add_argument('--kind',choices=KINDS);p.add_argument('--mission-file',default='')
    p.add_argument('--workspace',default=str(Path.home()/'drone_platform/01_apps/mission_demo'))
    p.add_argument('--store',default='/tmp/gcs-flight-tasks-'+str(os.getuid()));a=p.parse_args()
    if a.action=='start' and not a.kind:p.error('start requires --kind')
    ros=RosState();manager=Manager(a.workspace,a.store,ros)
    try:
        if a.action=='watch':
            while ros.rclpy.ok():emit({'ok':True,'state':manager.snapshot()});time.sleep(1)
        elif a.action=='status':time.sleep(.5);emit({'ok':True,'state':manager.snapshot()})
        elif a.action=='start':emit(manager.start(a.kind,a.mission_file))
        elif a.action=='start-map':
            raw=sys.stdin.buffer.read(1024*1024+1)
            if len(raw)>1024*1024:raise ValueError('航点请求过大')
            emit(manager.start('waypoint','',json.loads(raw)))
        elif a.action=='stop':emit(manager.stop())
        elif a.action=='validate':emit(manager.validate(a.mission_file))
        else:manager.stop_worker()
    finally: ros.close()

if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:pass
    except Exception as error:
        emit({'ok':False,'message':str(error)});sys.exit(1)
