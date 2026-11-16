#!/usr/bin/env python3
"""地面站三项任务入口。只管理本工具拥有的roslaunch，不直接生成飞行设定点。"""
import argparse, csv, fcntl, hashlib, json, math, os, re, signal, subprocess, sys, tempfile, time, uuid
from pathlib import Path
import xml.etree.ElementTree as ET

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

def launch_xml(kind, workspace, mission=None, camera=False):
    root = ET.Element('launch')
    def include(package, launch, args=None):
        node = ET.SubElement(root, 'include', file='$(find %s)/launch/%s' % (package,launch))
        for k,v in (args or {}).items(): ET.SubElement(node,'arg',name=k,value=str(v))
    if camera: include('rtsp_test','rtsp_test.launch')
    if kind == 'waypoint':
        include('waypoint_mission','waypoint_mission.launch',{'mission_file':mission,'start_camera':'false','dry_run':'false','bench_mode':'false'})
    elif kind == 'aruco':
        include('aruco_test','Safina_aruco.launch'); include('land_test','Safina_land.launch')
        # 准备时仅将云台转向地面，降落仍等待原生遥控OFFBOARD新边沿。
        ET.SubElement(root,'param',name='/aruco_landing/gimbal_down_on_startup',value='true',type='bool')
    elif kind == 'tracking':
        include('yoloUltra_test','Safina_yoloUltra_test.launch'); include('track_test','Safina_track.launch')
    else: raise ValueError('未知任务')
    return ET.tostring(root,encoding='unicode')

def pose_data(header,pose):
    return {'stamp':header.stamp.to_sec(),'frame':header.frame_id,
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
    """只读真实ROS状态；未收到消息时不伪装为未解锁。"""
    def __init__(self):
        import rospy
        from mavros_msgs.msg import State, ExtendedState
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String, Int32, Float32MultiArray
        import rosgraph
        self.rospy=rospy; self.master=rosgraph.Master('/gcs_flight_task_manager')
        rospy.init_node('gcs_flight_task_manager',anonymous=True,disable_signals=True)
        self.cache={}; self.header=[]; self.row=''
        self.subs=[]
        def sub(topic, msgtype, key, convert):
            def receive(m):
                try: self.cache[key]=(convert(m),time.monotonic())
                except (ValueError,TypeError,IndexError,json.JSONDecodeError): pass
            self.subs.append(rospy.Subscriber(topic,msgtype,receive,queue_size=1))
        sub('/mavros/state',State,'fc',lambda m:dict(connected=m.connected,armed=m.armed,mode=m.mode))
        sub('/mavros/extended_state',ExtendedState,'landed',lambda m:m.landed_state)
        sub('/mavros/local_position/pose',PoseStamped,'fc_pose',lambda m:pose_data(m.header,m.pose))
        sub('/factory_fastlio/odometry',Odometry,'lio_pose',lambda m:pose_data(m.header,m.pose.pose))
        sub('/waypoint_mission/status',String,'waypoint',lambda m:parse_fields(m.data))
        sub('/aruco_detector/metrics',String,'detector',lambda m:json.loads(m.data))
        sub('/aruco_target_id',Int32,'marker_id',lambda m:m.data)
        sub('/aruco_poseresult',PoseStamped,'marker_pose',lambda m:dict(x=m.pose.position.x,y=m.pose.position.y,z=m.pose.position.z))
        sub('/aruco_landing/status_header',String,'landing_header',lambda m:next(csv.reader([m.data])))
        sub('/aruco_landing/status',String,'landing_row',lambda m:next(csv.reader([m.data])))
        sub('/Safina_track',Float32MultiArray,'tracking',lambda m:dict(zip(('state','x_offset','y_offset','id','distance_m','distance_source','distance_valid'),list(m.data))))
    def value(self,key,max_age=3):
        value,at=self.cache.get(key,(None,0))
        return value if time.monotonic()-at<max_age else None
    def wait_fc(self):
        end=time.monotonic()+4
        while time.monotonic()<end:
            s=self.value('fc')
            if s and s.get('connected'): return s
            time.sleep(.05)
        raise RuntimeError('无法取得实时飞控连接状态，操作未执行')
    def publishers(self): return dict(self.master.getSystemState()[0])
    def map_session(self):
        # 使用真实定位发布节点PID，支持后台服务及管理器启动的FAST-LIO。
        try:
            import xmlrpc.client, http.client
            class LocalTransport(xmlrpc.client.Transport):
                def make_connection(self,host):return http.client.HTTPConnection(host,timeout=.5)
            nodes=self.publishers().get('/factory_fastlio/odometry',[])
            if len(nodes)!=1:return None
            uri=self.master.lookupNode(nodes[0])
            with xmlrpc.client.ServerProxy(uri,transport=LocalTransport()) as node:
                code,_,pid=node.getPid('/gcs_flight_task_manager')
            return identity(pid) if code==1 else None
        except Exception:return None
    def map_alignment(self):
        end=time.monotonic()+3
        while time.monotonic()<end:
            a=self.value('lio_pose',.5);b=self.value('fc_pose',.5)
            if a and b and abs(a['stamp']-b['stamp'])<.1:return a,b
            time.sleep(.02)
        raise RuntimeError('未取得同步的FAST-LIO与飞控位姿，无法转换地图航点')
    def conflicts(self):
        pubs=self.publishers()
        return {topic:pubs[topic] for topic in CONTROL_TOPICS if pubs.get(topic)}
    def summary(self):
        header=self.value('landing_header',86400) or []
        row=self.value('landing_row') or []
        marker=self.value('marker_pose',1) or {}
        marker_id=self.value('marker_id',1)
        detector=self.value('detector',1) or {}
        valid=marker_id is not None and marker_id>=0 and detector.get('selected_id')==marker_id and bool(marker) and all(math.isfinite(v) for v in marker.values())
        return {'fc':self.value('fc'), 'map_session':self.map_session(), 'waypoint':self.value('waypoint') or {},
                'aruco':{'id':marker_id if valid else -1,'valid':valid,'offset':marker if valid else {},'detector':detector,'landing':dict(zip(header,row))},
                'tracking':{k:(v if not isinstance(v,float) or math.isfinite(v) else None) for k,v in (self.value('tracking',1) or {}).items()}}

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
        run=subprocess.run(['rosrun','waypoint_mission','waypoint_mission','__name:=gcs_waypoint_validate_'+uuid.uuid4().hex,'_dry_run:=true','_mission_file:='+str(mission)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=12)
        if run.returncode: raise RuntimeError('原生航点校验失败：'+run.stdout[-1800:])
        config=yaml_data(mission).get('mission',{})
        mode=config.get('manual_airborne')
        if isinstance(mode,bool) and ('manual_airborne: '+str(mode).lower()) not in run.stdout:
            raise RuntimeError('机载航点程序未确认任务起飞模式，请先部署匹配版本')
        if config.get('name')=='gcs_map' and 'return_to_start: true, auto_land: true' not in run.stdout:
            raise RuntimeError('机载航点程序未确认返回起点并自动降落，请先部署匹配版本')
        return {'ok':True,'message':'任务文件已通过原生解析校验','mission_file':str(mission)}
    def start(self,kind,path,payload=None):
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
                template=yaml_data(self.workspace/'src/waypoint_mission/missions/abc_photo.yaml')
                mapped=map_mission(payload,lio,fc_pose,template)
            if kind=='aruco': yaml_data(self.workspace/'src/land_test/config/takeoff_land.yaml')
            if kind=='tracking':
                yaml_data(self.workspace/'src/track_test/config/track.yaml');yaml_data(self.workspace/'src/yoloUltra_test/config/yolo_track.yaml')
            pubs=self.ros.publishers()
            camera=False
            if kind=='aruco':
                if pubs.get('/aruco_imgresult'):raise RuntimeError('已有其他ArUco检测节点，请先协调该节点，避免重复启动')
                camera=not bool(pubs.get('/rtsp_cam/image_raw'))
            if kind=='tracking' and pubs.get('/Safina_track'):raise RuntimeError('已有其他人物识别节点，请先协调该节点')
            if kind=='waypoint':
                import yaml
                cfg=(mapped or yaml_data(Path(mission)))['mission']
                camera=any(bool(p.get('photo')) for p in cfg.get('waypoints',[])) and not bool(pubs.get('/rtsp_cam/image_raw'))
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
            launch=run/'task.launch';launch.write_text(launch_xml(kind,self.workspace,mission,camera))
            with launch.open('rb') as f:os.fsync(f.fileno())
            with (run/'task.log').open('wb') as log:
                p=subprocess.Popen(['roslaunch',str(launch)],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
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
    p.add_argument('--store',default=str(Path.home()/'.cache/gcs-flight-tasks'));a=p.parse_args()
    if a.action=='start' and not a.kind:p.error('start requires --kind')
    ros=RosState();manager=Manager(a.workspace,a.store,ros)
    if a.action=='watch':
        while not ros.rospy.is_shutdown():emit({'ok':True,'state':manager.snapshot()});time.sleep(1)
    elif a.action=='status':time.sleep(.5);emit({'ok':True,'state':manager.snapshot()})
    elif a.action=='start':emit(manager.start(a.kind,a.mission_file))
    elif a.action=='start-map':
        raw=sys.stdin.buffer.read(1024*1024+1)
        if len(raw)>1024*1024:raise ValueError('航点请求过大')
        emit(manager.start('waypoint','',json.loads(raw)))
    elif a.action=='stop':emit(manager.stop())
    elif a.action=='validate':emit(manager.validate(a.mission_file))
    else:manager.stop_worker()

if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:pass
    except Exception as error:
        emit({'ok':False,'message':str(error)});sys.exit(1)
