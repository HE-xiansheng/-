"""One on-board read-only sampler for the workstation; JSON over one SSH stream."""
import argparse,json,math,os,struct,sys,threading,time,zlib,socket
from collections import deque
from pathlib import Path

def source_frequency(samples,now,window_s):
    """仅由接收时间计算源话题频率；样本不足时不冒充 0 Hz。"""
    while samples and now-samples[0]>window_s:samples.popleft()
    if len(samples)<2 or samples[-1]<=samples[0]:return None
    return (len(samples)-1)/(samples[-1]-samples[0])

def cache_planner_status(cache,key,value,receipt):
    # 轨迹ID可在新规划会话中重用，禁止将前一会话缓存配给新状态。
    if key=='status' and value.get('session_id')!=cache.get('status',{}).get('data',{}).get('session_id'):
        cache.pop('trajectory',None);cache.pop('candidate_trajectory',None);cache.pop('grid',None)
    cache[key]={'data':value,'receipt':receipt}


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


def message_values(message,key):
    """读取ROS2类型化字段；不解析CDR或ROS1固定二进制偏移。"""
    if key in ('imu','fc_imu'):
        values={'angular_velocity':[message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z],
                'acceleration':[message.linear_acceleration.x,message.linear_acceleration.y,message.linear_acceleration.z]}
    elif key=='pressure':values={'pressure_pa':float(message.fluid_pressure)}
    elif key=='lidar':values={'point_count':int(message.point_num) if hasattr(message,'point_num') else int(message.width)*int(message.height)}
    elif key=='fc':values={x:getattr(message,x) for x in ('connected','armed','mode','system_status')}
    elif key=='landed':values={'landed_state':message.landed_state}
    elif key=='ekf':values={x:getattr(message,x) for x in ('attitude_status_flag','velocity_horiz_status_flag','velocity_vert_status_flag','pos_horiz_rel_status_flag','pos_vert_abs_status_flag','gps_glitch_status_flag','accel_error_status_flag')}
    elif key=='battery':values={x:float(getattr(message,x)) if math.isfinite(getattr(message,x)) else None for x in ('voltage','percentage','current')}
    elif key=='rc':values={'rssi':message.rssi,'channels':list(message.channels)}
    elif key=='text':values={'severity':message.severity,'text':message.text}
    elif key in ('lio','fc_odom','relay'):
        pose=message.pose if key=='relay' else message.pose.pose
        values={'position':[pose.position.x,pose.position.y,pose.position.z],
                'quaternion':[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]}
        if key!='relay':values['child_frame_id']=message.child_frame_id
    else:values={}
    def clean(value):
        if isinstance(value,float) and not math.isfinite(value):return None
        if isinstance(value,list):return [clean(v) for v in value]
        return value
    return {k:clean(v) for k,v in values.items()}


def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=0)
    p.add_argument('--cloud-topic',default='/factory_fastlio/cloud_world')
    p.add_argument('--stream-id',default='');p.add_argument('--udp-host');p.add_argument('--udp-port',type=int,default=9005)
    p.add_argument('--grid-only',action='store_true');p.add_argument('--no-rosout',action='store_true');p.add_argument('--planner',action='store_true');a=p.parse_args()
    from gcs_ros2_runtime import Runtime,stamp_seconds
    from rosidl_runtime_py.utilities import get_message
    runtime=Runtime('flight_desk_readonly_monitor')
    parent_pid=os.getppid()
    if a.udp_host and parent_pid==1:runtime.close();return
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM) if a.udp_host else None
    udp_sequence=time.monotonic_ns()&0xffffffff
    types={
      'lidar':('/livox/lidar','livox_ros_driver2/msg/CustomMsg',.4), 'imu':('/livox/imu','sensor_msgs/msg/Imu',.4),
      'fc_imu':('/mavros/imu/data','sensor_msgs/msg/Imu',.5),'pressure':('/mavros/imu/static_pressure','sensor_msgs/msg/FluidPressure',3.),
      'lio':('/factory_fastlio/odometry','nav_msgs/msg/Odometry',.4),'cloud':(a.cloud_topic,'sensor_msgs/msg/PointCloud2',.4),
      'relay':('/mavros/vision_pose/pose','geometry_msgs/msg/PoseStamped',.4),'fc_odom':('/mavros/local_position/odom','nav_msgs/msg/Odometry',.4),
      'fc':('/mavros/state','mavros_msgs/msg/State',3.),'landed':('/mavros/extended_state','mavros_msgs/msg/ExtendedState',3.),
      'ekf':('/mavros/estimator_status','mavros_msgs/msg/EstimatorStatus',3.),'rc':('/mavros/rc/in','mavros_msgs/msg/RCIn',1.),
      'battery':('/mavros/battery','sensor_msgs/msg/BatteryState',5.),'text':('/mavros/statustext/recv','mavros_msgs/msg/StatusText',120.)}
    definitions={k:(runtime.topic(k,t),typ,limit) for k,(t,typ,limit) in types.items()} if not a.grid_only else {}
    cache={};times={k:deque(maxlen=3000) for k in definitions};source_stamps={k:deque(maxlen=3000) for k in definitions}
    lock=threading.Lock();texts=deque(maxlen=12);errors={};subs={};last_graph=0.;graph={};graph_received=None
    planning={};plan_lock=threading.Lock();grid_last=[0.];previous_grid=None
    def receive(m,k):
        now=time.monotonic();stamp=stamp_seconds(m.header.stamp) if hasattr(m,'header') else None
        frame=m.header.frame_id if hasattr(m,'header') else ''
        try:values=message_values(m,k)
        except (AttributeError,ValueError,TypeError) as exc:
            errors[k]=str(exc);return
        with lock:
            previous=cache.get(k,{})
            if stamp is not None and stamp!=previous.get('stamp'):source_stamps[k].append(now)
            cache[k]={'receipt':now,'stamp':stamp,'frame':frame,'values':values}
            times[k].append(now)
            if k=='text':texts.append({'received_monotonic':now,**values})
    def install_subscriptions():
        for k,(topic,typename,_) in definitions.items():
            if k in subs:continue
            try:
                # 雷达可明确配置标准PointCloud2，不猜测任意二进制消息。
                typename=runtime.config.get('types',{}).get(k,typename)
                typ=get_message(typename)
                subs[k]=runtime.subscribe(typ,topic,lambda m,key=k:receive(m,key),k,depth=5)
                errors.pop(k,None)
            except (ImportError,ModuleNotFoundError,AttributeError,ValueError) as exc:errors[k]=str(exc)
    install_subscriptions()
    if a.planner:
        try:
            from std_msgs.msg import String
            from sensor_msgs.msg import PointCloud2
            def plan_status(msg,key):
                try:
                    value=json.loads(msg.data)
                    if isinstance(value,dict):
                        with plan_lock:cache_planner_status(planning,key,value,time.monotonic())
                except (ValueError,TypeError):pass
            runtime.subscribe(String,'/ego_planner/manual_preview/status',lambda m:plan_status(m,'status'),'planner_status','reliable')
            runtime.subscribe(String,'/ego_planner/px4_execution/status',lambda m:plan_status(m,'execution'),'execution_status','reliable')
            if a.grid_only:
                def grid(msg):
                    now=time.monotonic()
                    if now-grid_last[0]<.5 or msg.header.frame_id not in ('world','camera_init'):return
                    grid_last[0]=now
                    with plan_lock:resolution=planning.get('status',{}).get('data',{}).get('grid_resolution',runtime.config.get('grid_resolution'))
                    try:value=grid_display_payload(msg,resolution)
                    except (ValueError,KeyError,TypeError,OverflowError) as exc:
                        errors['grid']=str(exc);return
                    value.update(frame=msg.header.frame_id,stamp=stamp_seconds(msg.header.stamp),sequence=time.monotonic_ns())
                    with plan_lock:planning['grid']={'data':value,'receipt':now}
                runtime.subscribe(PointCloud2,'/drone_0_ego_planner_node/grid_map/occupancy_inflate',grid,'grid')
            else:
                try:
                    from traj_utils.msg import PolyTraj
                    def poly(msg,key):
                        value={'id':msg.traj_id,'start_time':stamp_seconds(msg.start_time),'order':msg.order,
                               'duration':list(msg.duration),'x':list(msg.coef_x),'y':list(msg.coef_y),'z':list(msg.coef_z)}
                        with plan_lock:planning[key]={'data':value,'receipt':time.monotonic()}
                    runtime.subscribe(PolyTraj,'/ego_planner/manual_preview/trajectory',lambda m:poly(m,'trajectory'),'trajectory','reliable')
                    runtime.subscribe(PolyTraj,'/ego_planner/navigation/candidate_trajectory',lambda m:poly(m,'candidate_trajectory'),'candidate_trajectory','reliable')
                except ImportError as exc:errors['planner']=str(exc)
        except ImportError as exc:errors['planner']=str(exc)
    root=Path(runtime.config.get('app_root',str(Path.home()/'drone_platform')))
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    command_topics=('/mavros/setpoint_position/local','/mavros/setpoint_position/global','/mavros/setpoint_velocity/cmd_vel','/mavros/setpoint_velocity/cmd_vel_unstamped','/mavros/setpoint_raw/local','/mavros/setpoint_raw/global','/mavros/setpoint_raw/attitude','/mavros/setpoint_attitude/attitude','/mavros/setpoint_attitude/thrust','/mavros/actuator_control','/mavros/rc/override','/mavros/setpoint_trajectory/local','/mavros/setpoint_accel/accel')
    start=time.monotonic()
    try:
        while runtime.ok() and (not a.udp_host or os.getppid()==parent_pid) and (not a.seconds or time.monotonic()-start<a.seconds):
            now=time.monotonic();rosnow=runtime.now()
            if now-last_graph>=1. and not a.grid_only:
                try:
                    graph=runtime.publishers([v[0] for v in definitions.values()]+list(command_topics));graph_received=now
                except Exception as exc:errors['graph']=str(exc)
                last_graph=now;install_subscriptions()
            with plan_lock:display={k:dict(v,receipt_age_s=now-v['receipt']) for k,v in planning.items()}
            if a.grid_only:
                packet={'type':'GRID1','stream_id':a.stream_id,'boot_id':boot_id,'server_monotonic':now,'sent_wall':time.time(),'ego_planner':display}
                if udp:
                    udp_sequence=(udp_sequence+1)&0xffffffff
                    groups,previous_grid=grid_transmissions(packet,udp_sequence,previous_grid)
                    for chunks in groups:
                        for datagram in chunks:udp.sendto(datagram,(a.udp_host,a.udp_port))
                else:print(json.dumps(packet,allow_nan=False),flush=True)
            else:
                rows={};graph_age=None if graph_received is None else now-graph_received
                with lock:
                    for k,(topic,typ,limit) in definitions.items():
                        endpoints=graph.get(topic,[])
                        row={'topic':topic,'type':runtime.config.get('types',{}).get(k,typ),'publishers':sorted(set(v['node'] for v in endpoints)),
                             'endpoints':endpoints,'graph_fresh':graph_age is not None and graph_age<=2.5,'graph_age_s':graph_age,
                             'samples':len(times[k]),'hz':source_frequency(times[k],now,12. if k=='battery' else 3.),
                             'source_hz':source_frequency(source_stamps[k],now,3.),'limit_s':limit}
                        if k in cache:
                            c=cache[k];row.update(source_age_s=rosnow-c['stamp'] if c['stamp'] is not None else None,
                                                  receipt_age_s=now-c['receipt'],frame=c['frame'],values=c['values'])
                        if k in errors:row['error']=errors[k]
                        rows[k]=row
                    messages=[dict(t,age_s=now-t['received_monotonic']) for t in texts]
                # ROS2应用管理器状态是独立契约；未部署时如实报告，不导入ROS1管理器。
                localization={'lio_running':False,'relay_running':False,'session_id':''}
                recording={'available':False,'phase':'UNAVAILABLE','status_fresh':False}
                try:
                    import gcs_ros2_recording
                    recording=gcs_ros2_recording.health()
                except (ImportError,ValueError,OSError):pass
                status_file=root/'04_data/gcs_ros2/status.json'
                try:
                    state=json.loads(status_file.read_text())
                    if state.get('boot_id')==boot_id and 0<=now-state['monotonic']<=2.5:
                        localization=state.get('localization',localization);recording=state.get('recording',recording)
                except (OSError,ValueError,KeyError,TypeError):pass
                packet={'schema':1,'server_monotonic':now,'sample_period_s':.2 if udp else 1.,'boot_id':boot_id,
                        'topics':rows,'messages':messages,'control_publishers':{t:[v['node'] for v in graph.get(t,[])] for t in command_topics if graph.get(t)},'scope':'read-only observation',
                        'localization':localization,'recording':recording,'ego_planner':dict(now=rosnow,**display),
                        'stream_id':a.stream_id,'sent_wall':time.time(),'sample_cost_s':time.monotonic()-now,'errors':errors}
                if udp:
                    udp_sequence=(udp_sequence+1)&0xffffffff
                    for datagram in status_datagrams(packet,udp_sequence):udp.sendto(datagram,(a.udp_host,a.udp_port))
                else:print(json.dumps(packet,ensure_ascii=False,allow_nan=False),flush=True)
            time.sleep(max(0.,(.5 if a.grid_only else .2 if udp else 1.)-(time.monotonic()-now)))
    except (BrokenPipeError,KeyboardInterrupt):pass
    finally:
        runtime.close()
        if udp:udp.close()

if __name__=='__main__':main()
