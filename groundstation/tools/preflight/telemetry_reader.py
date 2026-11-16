"""One on-board read-only sampler for the workstation; JSON over one SSH stream."""
import argparse,json,math,os,struct,sys,threading,time,zlib,socket,xmlrpc.client
from collections import deque
from pathlib import Path

def source_frequency(samples,now,window_s):
    """仅由接收时间计算源话题频率；样本不足时不冒充 0 Hz。"""
    while samples and now-samples[0]>window_s:samples.popleft()
    if len(samples)<2 or samples[-1]<=samples[0]:return None
    return (len(samples)-1)/(samples[-1]-samples[0])

def sensor_values(raw, ros_type):
    """只解码传感器的小字段，绝不把雷达点数组塞进状态包。"""
    try:
        offset=16+struct.unpack_from('<I',raw,12)[0]
        if offset>len(raw):return {}
        if ros_type=='sensor_msgs/Imu':
            gyro=struct.unpack_from('<3d',raw,offset+104)
            accel=struct.unpack_from('<3d',raw,offset+200)
            if not all(math.isfinite(v) for v in gyro+accel):return {}
            return {'angular_velocity':list(gyro),'acceleration':list(accel)}
        if ros_type=='sensor_msgs/FluidPressure':
            value=struct.unpack_from('<d',raw,offset)[0]
            return {'pressure_pa':value} if math.isfinite(value) else {}
        if ros_type in ('livox_ros_driver2/CustomMsg','livox_ros_driver/CustomMsg'):
            return {'point_count':struct.unpack_from('<I',raw,offset+8)[0]}
        if ros_type=='sensor_msgs/PointCloud2':
            height,width=struct.unpack_from('<II',raw,offset)
            return {'point_count':height*width}
    except (struct.error,ValueError):pass
    return {}

def demo_status(root):
    try:
        d=json.loads((root/'active-control-demo.json').read_text());run=Path(d['run'])
        try:
            argv=(Path('/proc')/str(int(d['pid']))/'cmdline').read_bytes().split(b'\0')
            alive=d['entry'].encode() in argv and b'--run' in argv and argv[argv.index(b'--run')+1].decode()==str(run)
        except (OSError,ValueError,IndexError):alive=False
        try:s=json.loads((run/'status.json').read_text())
        except (OSError,ValueError):s={}
        result=dict(d,alive=alive,status=s,status_age_s=time.monotonic()-s.get('monotonic',0))
        for name,key in [('completion.json','completion'),('lock-result.json','lock')]:
            try:result[key]=json.loads((run/name).read_text())
            except (OSError,ValueError):pass
        return result
    except (OSError,ValueError,KeyError,TypeError,IndexError):return {'alive':False}

def cache_planner_status(cache,key,value,receipt):
    # 轨迹ID可在新规划会话中重用，禁止将前一会话缓存配给新状态。
    if key=='status' and value.get('session_id')!=cache.get('status',{}).get('data',{}).get('session_id'):
        cache.pop('trajectory',None);cache.pop('candidate_trajectory',None);cache.pop('grid',None)
    cache[key]={'data':value,'receipt':receipt}


def localization_status(root,boot_id):
    # 只读进程身份，旧 active-*.json 不作为存活证据；不引入 ROS 发布者。
    result={'lio_running':False,'relay_running':False,'session_id':''}
    for kind,key in [('localization','lio'),('flight-pose-relay','relay')]:
        try:
            d=json.loads((root/('active-'+kind+'.json')).read_text());p=Path('/proc')/str(int(d['pid']))
            args=p.joinpath('cmdline').read_bytes().split(b'\0')
            if key=='lio':owned=str(Path(d['run'])/'localization.launch').encode() in args and str(root/'runs') in d['run']
            else:owned=p.joinpath('exe').resolve()==(Path.home()/'drone_platform/02_drivers/sensors/lib/to_mavros/vision2mavros').resolve() and b'__name:=factory_vision2mavros' in args
            if not owned:continue
            tick=p.joinpath('stat').read_text().rsplit(')',1)[1].split()[19]
            result[key+'_running']=True
            if key=='lio':result['session_id']=boot_id+':'+str(d['pid'])+':'+tick
        except (OSError,ValueError,KeyError,IndexError):pass
    return result


def grid_display_payload(msg,resolution,max_points=18000):
    """只压缩显示抽样的网格中心；无效帧拒绝，空帧也保留。"""
    import base64
    import numpy as np
    if not isinstance(resolution,(int,float)) or not math.isfinite(resolution) or resolution<=0:resolution=None
    width,height=int(msg.width),int(msg.height)
    if width<0 or height<0 or max_points<=0:raise ValueError('栅格尺寸无效')
    source_count=width*height
    if not source_count:
        return {'xyz':base64.b64encode(zlib.compress(b'',1)).decode(),'codec':'zlib-f32','count':0,'source_count':0,'resolution':resolution}
    point_step,row_step=int(msg.point_step),int(msg.row_step)
    if point_step<=0 or row_step<width*point_step or len(msg.data)<height*row_step:raise ValueError('栅格数据长度或步幅不符')
    fields={f.name:f for f in msg.fields};endian='>' if msg.is_bigendian else '<'
    stride=max(1,int(math.ceil(source_count/max_points)));axes=[]
    for name in ('x','y','z'):
        f=fields[name];kind={7:'f4',8:'f8'}.get(f.datatype)
        if not kind or f.count!=1 or f.offset<0 or f.offset+np.dtype(kind).itemsize>point_step:raise ValueError('栅格XYZ字段无效')
        axes.append(np.ndarray((height,width),dtype=endian+kind,buffer=msg.data,offset=f.offset,strides=(row_step,point_step)).reshape(-1)[::stride])
    xyz=np.column_stack(axes).astype('<f4');xyz=xyz[np.isfinite(xyz).all(axis=1)]
    raw=xyz.tobytes();codec='zlib-f32';encoded=zlib.compress(raw,1)
    if len(xyz) and isinstance(resolution,(int,float)) and math.isfinite(resolution) and resolution>0:
        scale=resolution/2.;indices=np.rint(xyz/scale)
        if np.isfinite(indices).all() and indices.min()>=-32768 and indices.max()<=32767 and np.max(np.abs(indices*scale-xyz))<=min(1e-5,resolution*1e-4):
            integer=indices.astype(np.int32);plain=integer.astype('<i2').tobytes();candidate=zlib.compress(plain,1)
            if len(candidate)<len(encoded):encoded=candidate;codec='zlib-i16-half-grid'
            delta=np.vstack((integer[:1],np.diff(integer,axis=0)))
            if delta.min()>=-32768 and delta.max()<=32767:
                candidate=zlib.compress(delta.astype('<i2').tobytes(),1)
                if len(candidate)<len(encoded):encoded=candidate;codec='zlib-i16-delta-half-grid'
    return {'xyz':base64.b64encode(encoded).decode(),'codec':codec,'count':len(xyz),'source_count':source_count,'resolution':resolution}


def status_datagrams(packet,sequence,magic=b'TLM1'):
    raw=json.dumps(packet,ensure_ascii=False,allow_nan=False,separators=(',',':')).encode()
    if len(raw)>512*1024:raise ValueError('状态采样超过上限')
    data=zlib.compress(raw,1);room=1200;count=max(1,(len(data)+room-1)//room)
    if count>256:raise ValueError('状态分片过多')
    for index in range(count):
        yield struct.pack('<4sIHHI',magic,sequence,index,count,len(raw))+data[index*room:(index+1)*room]

def grid_transmissions(packet,sequence,previous):
    """新帧发两份，随后补发同会话上一帧；保留上一帧原始采样时间。"""
    planning=packet.get('ego_planner',{})
    session=planning.get('status',{}).get('data',{}).get('session_id')
    grid_sequence=planning.get('grid',{}).get('data',{}).get('sequence')
    chunks=list(status_datagrams(packet,sequence,magic=b'TLG1'));groups=[chunks,chunks]
    if previous and session and session==previous[0] and grid_sequence is not None and previous[1] is not None and grid_sequence!=previous[1]:
        groups.append(previous[2])
    return groups,(session,grid_sequence,chunks)


def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=0)
    p.add_argument('--cloud-topic',default='/factory_fastlio/cloud_body')
    p.add_argument('--stream-id',default='');p.add_argument('--udp-host');p.add_argument('--udp-port',type=int,default=9005);p.add_argument('--grid-only',action='store_true');p.add_argument('--no-rosout',action='store_true');p.add_argument('--planner',action='store_true');a=p.parse_args()
    parent_pid=os.getppid()
    if a.udp_host and parent_pid==1:raise SystemExit('只读采样的SSH管理进程已退出')
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM) if a.udp_host else None
    if udp and not a.grid_only:
        try:udp.setsockopt(socket.IPPROTO_IP,socket.IP_TOS,0xb8)  # 只读状态小包优先，不修改系统网络配置。
        except OSError:pass
    udp_sequence=time.monotonic_ns()&0xffffffff
    import rospy
    from mavros_msgs.msg import State,ExtendedState,EstimatorStatus,RCIn,StatusText
    from sensor_msgs.msg import BatteryState
    rospy.init_node('flight_desk_readonly_monitor',anonymous=True,disable_signals=True,disable_rosout=a.no_rosout)
    definitions={
      'lidar':('/livox/lidar',rospy.AnyMsg,.4), 'imu':('/livox/imu',rospy.AnyMsg,.4),
      'fc_imu':('/mavros/imu/data',rospy.AnyMsg,.5),
      'pressure':('/mavros/imu/static_pressure',rospy.AnyMsg,3.),
      'lio':('/factory_fastlio/odometry',rospy.AnyMsg,.4),
      'cloud':(a.cloud_topic,rospy.AnyMsg,.4),
      'relay':('/mavros/vision_pose/pose',rospy.AnyMsg,.4),
      'fc_odom':('/mavros/local_position/odom',rospy.AnyMsg,.4),
      'fc':('/mavros/state',State,3.), 'landed':('/mavros/extended_state',ExtendedState,3.),
      'ekf':('/mavros/estimator_status',EstimatorStatus,3.), 'rc':('/mavros/rc/in',RCIn,1.),
      'battery':('/mavros/battery',BatteryState,5.), 'text':('/mavros/statustext/recv',StatusText,120.)}
    if a.grid_only:definitions={}
    cache={};times={k:deque(maxlen=3000) for k in definitions};texts=deque(maxlen=12);lock=threading.Lock()
    def receive(m,k):
        now=time.monotonic(); stamp=0.;frame=''
        if isinstance(m,rospy.AnyMsg):
            try:
                _,sec,ns=struct.unpack_from('<III',m._buff);stamp=sec+ns*1e-9
                n=struct.unpack_from('<I',m._buff,12)[0];frame=m._buff[16:16+n].decode(errors='replace')
            except (struct.error,ValueError):pass
        else:stamp=m.header.stamp.to_sec();frame=m.header.frame_id
        values=sensor_values(m._buff,m._connection_header.get('type','')) if isinstance(m,rospy.AnyMsg) else {}
        if k=='fc':values={x:getattr(m,x) for x in ['connected','armed','mode','system_status']}
        elif k=='landed':values={'landed_state':m.landed_state}
        elif k=='ekf':values={x:getattr(m,x) for x in ['attitude_status_flag','velocity_horiz_status_flag','velocity_vert_status_flag','pos_horiz_rel_status_flag','pos_vert_abs_status_flag','gps_glitch_status_flag','accel_error_status_flag']}
        elif k=='battery':values={x:float(getattr(m,x)) if math.isfinite(getattr(m,x)) else None for x in ['voltage','percentage','current']}
        elif k=='rc':values={'rssi':m.rssi,'channels':list(m.channels)}
        elif k=='text':values={'severity':m.severity,'text':m.text}
        elif k in ['lio','fc_odom','relay']:
            # Decode only the small pose messages; no cloud/raw-lidar payload crosses SSH.
            import roslib.message
            typ=roslib.message.get_message_class(m._connection_header.get('type',''))
            if typ:
                decoded=typ();decoded.deserialize(m._buff)
                pose=decoded.pose if k=='relay' else decoded.pose.pose
                values={'position':[pose.position.x,pose.position.y,pose.position.z],
                        'quaternion':[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]}
                if k!='relay':values['child_frame_id']=decoded.child_frame_id
        with lock:
            cache[k]={'receipt':now,'stamp':stamp,'frame':frame,'values':values}
            times[k].append(now)
            if k=='text':texts.append({'received_monotonic':now,**values})
    subs=[rospy.Subscriber(t,typ,receive,callback_args=k,queue_size=1,buff_size=4*1024*1024) for k,(t,typ,_) in definitions.items()]
    # 仅订阅已有EGO-Planner，栅格保留原坐标，抽样只影响显示。
    planning={};plan_lock=threading.Lock();grid_last=[0.];previous_grid=None
    if a.planner:
        from std_msgs.msg import String
        from traj_utils.msg import PolyTraj
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs import point_cloud2
        import numpy as np
        def plan_status(msg,key):
            try:
                value=json.loads(msg.data)
                if not isinstance(value,dict):return
                with plan_lock:cache_planner_status(planning,key,value,time.monotonic())
            except (ValueError,TypeError):pass
        def poly(msg,key='trajectory'):
            value={'id':msg.traj_id,'start_time':msg.start_time.to_sec(),'order':msg.order,
                   'duration':list(msg.duration),'x':list(msg.coef_x),'y':list(msg.coef_y),'z':list(msg.coef_z)}
            with plan_lock:planning[key]={'data':value,'receipt':time.monotonic()}
        def grid(msg):
            now=time.monotonic()
            if now-grid_last[0]<.5:return
            grid_last[0]=now
            if msg.header.frame_id not in ('world','camera_init'):return
            try:
                value=grid_display_payload(msg,rospy.get_param('/drone_0_ego_planner_node/grid_map/resolution',None))
            except (ValueError,KeyError,TypeError,OverflowError) as exc:
                rospy.logwarn_throttle(5.,'栅格显示帧无效: %s',str(exc));return
            value.update(frame=msg.header.frame_id,stamp=msg.header.stamp.to_sec(),sequence=time.monotonic())
            with plan_lock:planning['grid']={'data':value,'receipt':now}
        subs.append(rospy.Subscriber('/ego_planner/manual_preview/status',String,plan_status,callback_args='status',queue_size=1))
        if a.grid_only:
            subs.append(rospy.Subscriber('/drone_0_ego_planner_node/grid_map/occupancy_inflate',PointCloud2,grid,queue_size=1,buff_size=4*1024*1024))
        else:
            subs.extend([rospy.Subscriber('/ego_planner/px4_execution/status',String,plan_status,callback_args='execution',queue_size=1),
                         rospy.Subscriber('/ego_planner/manual_preview/trajectory',PolyTraj,poly,queue_size=1),
                         rospy.Subscriber('/ego_planner/navigation/candidate_trajectory',PolyTraj,poly,callback_args='candidate_trajectory',queue_size=1)])
    recorder=None
    try:
        import importlib.util
        spec=importlib.util.spec_from_file_location('gcs_recording_status',Path.home()/'drone_platform/01_apps/ego_planner/recording_manage.py')
        recorder=importlib.util.module_from_spec(spec);spec.loader.exec_module(recorder)
    except (OSError,ImportError):pass
    graph_cache={'pubs':{},'received':None};graph_lock=threading.Lock()
    class GraphTransport(xmlrpc.client.Transport):
        def make_connection(self,host):
            connection=super().make_connection(host);connection.timeout=2.;return connection
    def poll_graph():
        while not rospy.is_shutdown():
            try:
                code,_,state=xmlrpc.client.ServerProxy('http://127.0.0.1:11311',transport=GraphTransport()).getSystemState('/gcs_readonly_graph')
                if code==1:
                    with graph_lock:graph_cache.update(pubs=dict(state[0]),received=time.monotonic())
            except (OSError,xmlrpc.client.Error):pass
            time.sleep(1.)
    if not a.grid_only:threading.Thread(target=poll_graph,daemon=True).start()
    root=Path.home()/'drone_platform/01_apps/fast_lio';start=time.monotonic()
    try:
        while not rospy.is_shutdown() and (not a.udp_host or os.getppid()==parent_pid) and (not a.seconds or time.monotonic()-start<a.seconds):
            iteration_started=time.monotonic()
            if a.grid_only:
                with plan_lock:display={k:dict(v,receipt_age_s=time.monotonic()-v['receipt']) for k,v in planning.items()}
                packet={'type':'GRID1','stream_id':a.stream_id,'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                        'server_monotonic':time.monotonic(),'sent_wall':time.time(),'ego_planner':display}
                if udp:
                    udp_sequence=(udp_sequence+1)&0xffffffff
                    groups,previous_grid=grid_transmissions(packet,udp_sequence,previous_grid)
                    # 补发保留原始时间；解码端丢弃重复与乱序帧，不会延长数据有效期。
                    for chunks in groups:
                        for datagram in chunks:udp.sendto(datagram,(a.udp_host,a.udp_port))
                else:print(json.dumps(packet,allow_nan=False),flush=True)
                time.sleep(max(0,.5-(time.monotonic()-iteration_started)));continue
            with graph_lock:
                graph_age=None if graph_cache['received'] is None else time.monotonic()-graph_cache['received']
                graph_fresh=graph_age is not None and 0<=graph_age<=2.5
                pubs=dict(graph_cache['pubs']) if graph_fresh else {}
            rows={}
            with lock:
                # RPC can take long enough for callbacks to replace the cache.
                # Timestamp the actual snapshot after the RPC, under its lock.
                now=time.monotonic();rosnow=rospy.Time.now().to_sec()
                for k,(topic,_,limit) in definitions.items():
                    # 电池常以约 0.5 Hz 更新；3 秒窗口会周期性只剩一帧。
                    hz=source_frequency(times[k],now,12. if k=='battery' else 3.)
                    row={'topic':topic,'publishers':pubs.get(topic,[]),'graph_fresh':graph_fresh,'graph_age_s':graph_age,'samples':len(times[k]),'hz':hz,'limit_s':limit}
                    if k in cache:
                        c=cache[k];row.update(source_age_s=rosnow-c['stamp'],receipt_age_s=now-c['receipt'],frame=c['frame'],values=c['values'])
                    rows[k]=row
                messages=[{**t,'age_s':now-t['received_monotonic']} for t in texts]
            plan={}
            try:
                d=json.loads((root/'active-flight-preview.json').read_text());argv=(Path('/proc')/str(d['pid'])/'cmdline').read_bytes().split(b'\0')
                if b'worker' in argv and b'--run' in argv and argv[argv.index(b'--run')+1].decode()==d['run'] and any(x.endswith(b'/flight_board.py') for x in argv):
                    run=Path(d['run']);status=json.loads((run/'status.json').read_text());age=time.monotonic()-status['monotonic'] if 'monotonic' in status else time.time()-status.get('time',0)
                    plan={'active':d,'status':status,'status_age_s':age}
                    try:plan['goal_lock']=json.loads((run/'goal-lock-result.json').read_text())
                    except (OSError,ValueError):pass
                    f=run/'results/frames.jsonl'
                    if f.exists():
                        with f.open('rb') as stream:
                            stream.seek(max(0,f.stat().st_size-65536));lines=stream.read().splitlines()
                        if lines:
                            frame=json.loads(lines[-1]);plan['last_prediction']={'source_age_s':rosnow-frame['sampling']['source_stamp'],'candidate':frame['candidate'],'quality':frame['sampling'].get('quality'),'goal_world':frame['goal_world']}
            except (OSError,ValueError,KeyError,IndexError):pass
            command_topics={'/mavros/setpoint_position/local','/mavros/setpoint_position/global','/mavros/setpoint_velocity/cmd_vel','/mavros/setpoint_velocity/cmd_vel_unstamped','/mavros/setpoint_raw/local','/mavros/setpoint_raw/global','/mavros/setpoint_raw/attitude','/mavros/setpoint_attitude/attitude','/mavros/setpoint_attitude/thrust','/mavros/actuator_control','/mavros/rc/override','/mavros/setpoint_trajectory/local','/mavros/setpoint_accel/accel','/mavros/setpoint_attitude/cmd_vel','/mavros/setpoint_attitude/cmd_vel_unstamped'}
            # /mavros/setpoint_trajectory/desired and raw/target_* are MAVROS
            # feedback outputs, not evidence of an external command publisher.
            commands={t:n for t,n in pubs.items() if t in command_topics}
            packet={'schema':1,'server_monotonic':now,'sample_period_s':(.2 if udp else 1.),'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'topics':rows,'messages':messages,'plan':plan,'demo_session':demo_status(root),'control_publishers':commands,'scope':'read-only observation, not full arming permission'}
            packet['localization']=localization_status(root,packet['boot_id'])
            packet['recording']=recorder.summary(recorder.health()) if recorder else {'available':False,'phase':'UNAVAILABLE','status_fresh':False}
            with plan_lock:packet['ego_planner']={'now':rosnow,**{k:dict(v,receipt_age_s=time.monotonic()-v['receipt']) for k,v in planning.items()}}
            packet['stream_id']=a.stream_id
            packet['sent_wall']=time.time();packet['sample_cost_s']=time.monotonic()-iteration_started
            if udp:
                udp_sequence=(udp_sequence+1)&0xffffffff
                for datagram in status_datagrams(packet,udp_sequence):udp.sendto(datagram,(a.udp_host,a.udp_port))
            else:print(json.dumps(packet,ensure_ascii=False,allow_nan=False),flush=True)
            time.sleep(max(0,(.2 if udp else 1.)-(time.monotonic()-iteration_started)))
    except (BrokenPipeError,KeyboardInterrupt):pass
    finally:
        for sub in subs:sub.unregister()
        if udp:udp.close()
if __name__=='__main__':main()
