"""ROS2机载只读采样：类型化订阅、DDS图与只读栅格参数；不提供飞控控制接口。"""
import argparse,json,math,os,struct,sys,threading,time,zlib,socket
from collections import deque
from pathlib import Path

def source_frequency(samples,now,window_s):
    """仅由接收时间计算源话题频率；样本不足时不冒充 0 Hz。"""
    while samples and now-samples[0]>window_s:samples.popleft()
    if len(samples)<2 or samples[-1]<=samples[0]:return None
    return (len(samples)-1)/(samples[-1]-samples[0])

def stamp_seconds(stamp):
    return float(stamp.sec)+float(stamp.nanosec)*1e-9


def sensor_values(msg, key):
    """只取类型化消息的小字段，不依赖ROS1序列化偏移或ROS2的CDR布局。"""
    if key in ('imu','fc_imu'):
        gyro=msg.angular_velocity;accel=msg.linear_acceleration
        values=[gyro.x,gyro.y,gyro.z,accel.x,accel.y,accel.z]
        return {'angular_velocity':values[:3],'acceleration':values[3:]} if all(math.isfinite(v) for v in values) else {}
    if key=='pressure':
        return {'pressure_pa':msg.fluid_pressure} if math.isfinite(msg.fluid_pressure) else {}
    if key=='lidar':return {'point_count':int(msg.point_num)}
    if key=='cloud':return {'point_count':int(msg.height)*int(msg.width)}
    return {}


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


COMMAND_TOPICS={
    '/mavros/setpoint_position/local','/mavros/setpoint_position/global',
    '/mavros/setpoint_velocity/cmd_vel','/mavros/setpoint_velocity/cmd_vel_unstamped',
    '/mavros/setpoint_raw/local','/mavros/setpoint_raw/global'}
# MAVROS反馈target_*和desired不是外部控制指令。
COMMAND_TOPICS.update('/mavros/'+name for name in (
    'setpoint_raw/attitude','setpoint_attitude/attitude','setpoint_attitude/thrust',
    'actuator_control','rc/override','setpoint_trajectory/local','setpoint_accel/accel',
    'setpoint_attitude/cmd_vel','setpoint_attitude/cmd_vel_unstamped'))


def node_name(info):
    return info.node_namespace.rstrip('/')+'/'+info.node_name


class TelemetryReader:
    """标准执行器负责回调；主线程快照不等待飞控或参数RPC。"""
    def __init__(self,args):
        import rclpy
        from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
        from mavros_msgs.msg import State,ExtendedState,EstimatorStatus,RCIn,StatusText
        from sensor_msgs.msg import BatteryState,Imu,FluidPressure,PointCloud2
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseStamped
        from livox_ros_driver2.msg import CustomMsg
        import uuid
        self.args=args;self.node=rclpy.create_node('flight_desk_readonly_'+uuid.uuid4().hex[:8],enable_rosout=not args.no_rosout)
        self.boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.definitions={
            'lidar':('/livox/lidar',CustomMsg,.4),'imu':('/livox/imu',Imu,.4),
            'fc_imu':('/mavros/imu/data',Imu,.5),'pressure':('/mavros/imu/static_pressure',FluidPressure,3.),
            'lio':('/factory_fastlio/odometry',Odometry,.4),'cloud':(args.cloud_topic,PointCloud2,.4),
            'relay':('/mavros/vision_pose/pose',PoseStamped,.4),'fc_odom':('/mavros/local_position/odom',Odometry,.4),
            'fc':('/mavros/state',State,3.),'landed':('/mavros/extended_state',ExtendedState,3.),
            'ekf':('/mavros/estimator_status',EstimatorStatus,3.),'rc':('/mavros/rc/in',RCIn,1.),
            'battery':('/mavros/battery',BatteryState,5.),'text':('/mavros/statustext/recv',StatusText,120.)}
        if args.grid_only:self.definitions={}
        self.lock=threading.RLock();self.cache={};self.times={k:deque(maxlen=3000) for k in self.definitions}
        self.texts=deque(maxlen=12);self.planning={};self.graph={'pubs':{},'endpoints':{},'received':None}
        self.resolution=None;self.resolution_receipt=0.;self.parameter_pending=None;self.grid_last=0.;self.subs=[]
        for key,(topic,typ,_) in self.definitions.items():
            qos=QoSProfile(depth=1) if key in ('cloud','lidar') else qos_profile_sensor_data
            self.subs.append(self.node.create_subscription(typ,topic,lambda m,k=key:self.receive(m,k),qos))
        if args.planner:
            from std_msgs.msg import String
            from traj_utils.msg import PolyTraj
            retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subs.append(self.node.create_subscription(String,'/ego_planner/manual_preview/status',lambda m:self.plan_status(m,'status'),retained))
            if args.grid_only:
                from rcl_interfaces.srv import GetParameters
                self.parameters=self.node.create_client(GetParameters,args.planner_node.rstrip('/')+'/get_parameters')
                self.parameter_timer=self.node.create_timer(1.,self.poll_resolution)
                self.subs.append(self.node.create_subscription(PointCloud2,args.grid_topic,self.grid,QoSProfile(depth=1)))
            else:
                self.subs.append(self.node.create_subscription(String,'/ego_planner/px4_execution/status',lambda m:self.plan_status(m,'execution'),retained))
                for topic,key in [('/ego_planner/manual_preview/trajectory','trajectory'),('/ego_planner/navigation/candidate_trajectory','candidate_trajectory')]:
                    self.subs.append(self.node.create_subscription(PolyTraj,topic,lambda m,k=key:self.poly(m,k),retained))
        self.graph_timer=self.node.create_timer(.5,self.poll_graph)

    def receive(self,m,key):
        receipt=time.monotonic();stamp=stamp_seconds(m.header.stamp);frame=m.header.frame_id
        values=sensor_values(m,key)
        if key=='fc':values={k:getattr(m,k) for k in ('connected','armed','mode','system_status')}
        elif key=='landed':values={'landed_state':m.landed_state}
        elif key=='ekf':values={k:getattr(m,k) for k in ('attitude_status_flag','velocity_horiz_status_flag','velocity_vert_status_flag','pos_horiz_rel_status_flag','pos_vert_abs_status_flag','gps_glitch_status_flag','accel_error_status_flag')}
        elif key=='battery':values={k:float(getattr(m,k)) if math.isfinite(getattr(m,k)) else None for k in ('voltage','percentage','current')}
        elif key=='rc':values={'rssi':m.rssi,'channels':list(m.channels)}
        elif key=='text':values={'severity':m.severity,'text':m.text}
        elif key in ('lio','fc_odom','relay'):
            pose=m.pose if key=='relay' else m.pose.pose
            xyz=[pose.position.x,pose.position.y,pose.position.z];q=pose.orientation
            quat=[q.x,q.y,q.z,q.w]
            # 无效数值不能破坏整份JSON；空位姿由显示端明确标为无效。
            values={'position':xyz,'quaternion':quat} if all(math.isfinite(v) for v in xyz+quat) else {}
            if key!='relay':values['child_frame_id']=m.child_frame_id
        with self.lock:
            self.cache[key]={'receipt':receipt,'stamp':stamp,'frame':frame,'values':values}
            self.times[key].append(receipt)
            if key=='text':self.texts.append({'received_monotonic':receipt,**values})

    def poll_graph(self):
        try:
            pubs={};endpoints={}
            topics={topic for topic,_,_ in self.definitions.values()}|COMMAND_TOPICS|{self.args.grid_topic}
            for topic in topics:
                infos=self.node.get_publishers_info_by_topic(topic)
                if infos:
                    # 相同节点名的两个实例也保留两项，不通过set掩盖冲突。
                    pubs[topic]=sorted(node_name(i) for i in infos)
                    endpoints[topic]=[dict(node=node_name(i),gid=bytes(i.endpoint_gid).hex()) for i in infos]
            with self.lock:self.graph={'pubs':pubs,'endpoints':endpoints,'received':time.monotonic()}
        except RuntimeError:pass

    def plan_status(self,msg,key):
        try:
            value=json.loads(msg.data,parse_constant=lambda _:(_ for _ in ()).throw(ValueError('非有限JSON')))
            if not isinstance(value,dict):return
            # 包括数值溢出产生的Infinity；拒绝后保留原接收时刻，不延长旧缓存寿命。
            json.dumps(value,allow_nan=False)
            with self.lock:cache_planner_status(self.planning,key,value,time.monotonic())
        except (ValueError,TypeError):pass

    def poly(self,msg,key):
        value={'id':msg.traj_id,'start_time':stamp_seconds(msg.start_time),'order':msg.order,
               'duration':list(msg.duration),'x':list(msg.coef_x),'y':list(msg.coef_y),'z':list(msg.coef_z)}
        try:json.dumps(value,allow_nan=False)
        except ValueError:return
        with self.lock:self.planning[key]={'data':value,'receipt':time.monotonic()}

    def poll_resolution(self):
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters
        now=time.monotonic()
        if self.parameter_pending is not None:
            future,started=self.parameter_pending
            if not future.done():
                if now-started>2.:
                    self.parameters.remove_pending_request(future)
                    future.cancel();self.parameter_pending=None
                return
            self.parameter_pending=None
            try:
                value=future.result().values[0]
                resolution=value.double_value if value.type==ParameterType.PARAMETER_DOUBLE else None
                if resolution is None or not math.isfinite(resolution) or resolution<=0:raise ValueError('分辨率参数无效')
                with self.lock:self.resolution=resolution;self.resolution_receipt=now
            except (RuntimeError,ValueError,AttributeError,IndexError):
                with self.lock:self.resolution=None
        if self.parameters.service_is_ready():
            self.parameter_pending=(self.parameters.call_async(GetParameters.Request(names=['grid_map.resolution'])),now)

    def grid(self,msg):
        now=time.monotonic();rosnow=self.node.get_clock().now().nanoseconds*1e-9
        if now-self.grid_last<.5:return
        self.grid_last=now
        stamp=stamp_seconds(msg.header.stamp)
        with self.lock:
            status=self.planning.get('status',{}).get('data',{})
            resolution=self.resolution if now-self.resolution_receipt<=3. else None
            publishers=self.graph['pubs'].get(self.args.grid_topic,[])
        if publishers!=[self.args.planner_node]:return
        if not status.get('session_id') or msg.header.frame_id!=status.get('goal_frame') or not -.05<=rosnow-stamp<=1.5:return
        try:
            value=grid_display_payload(msg,resolution)
        except (ValueError,KeyError,TypeError,OverflowError):return
        value.update(frame=msg.header.frame_id,stamp=stamp,sequence=now,session_id=status['session_id'])
        with self.lock:self.planning['grid']={'data':value,'receipt':now}

    def snapshot(self,period):
        now=time.monotonic();rosnow=self.node.get_clock().now().nanoseconds*1e-9
        with self.lock:
            graph_age=None if self.graph['received'] is None else now-self.graph['received']
            fresh=graph_age is not None and 0<=graph_age<=2.5
            pubs=self.graph['pubs'] if fresh else {};endpoints=self.graph['endpoints'] if fresh else {}
            display={k:dict(v,receipt_age_s=now-v['receipt']) for k,v in self.planning.items()}
            if self.args.grid_only:
                return {'type':'GRID1','stream_id':self.args.stream_id,'boot_id':self.boot,'server_monotonic':now,'sent_wall':time.time(),'ego_planner':display}
            rows={}
            for key,(topic,_,limit) in self.definitions.items():
                row={'topic':topic,'publishers':pubs.get(topic,[]),'graph_fresh':fresh,'graph_age_s':graph_age,
                     'samples':len(self.times[key]),'hz':source_frequency(self.times[key],now,12. if key=='battery' else 3.),'limit_s':limit}
                if key in self.cache:
                    c=self.cache[key];row.update(source_age_s=rosnow-c['stamp'],receipt_age_s=now-c['receipt'],frame=c['frame'],values=c['values'])
                rows[key]=row
            messages=[dict(t,age_s=now-t['received_monotonic']) for t in self.texts]
            # 基于当前图的进程存在性，不冒用旧ROS1 active文件；正常数据还需独立源时间校验。
            lio=endpoints.get('/factory_fastlio/odometry',[]);relay=endpoints.get('/mavros/vision_pose/pose',[])
            session=':'.join([self.boot,str(self.node.context.get_domain_id()),lio[0]['gid']]) if len(lio)==1 and lio[0]['node']=='/factory_laserMapping' else ''
            localization={'lio_running':bool(lio),'relay_running':bool(relay),'session_id':session,'identity_source':'ros2_graph','graph_fresh':fresh}
            return {'schema':1,'server_monotonic':now,'sample_period_s':period,'boot_id':self.boot,'topics':rows,'messages':messages,
                    'plan':{},'demo_session':{'alive':False},'control_publishers':{t:pubs[t] for t in COMMAND_TOPICS if t in pubs},
                    'scope':'read-only observation, not full arming permission','localization':localization,
                    'recording':{'available':False,'phase':'UNAVAILABLE','status_fresh':False},
                    'ego_planner':{'now':rosnow,**display},'stream_id':self.args.stream_id,'sent_wall':time.time()}


def main():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor,ExternalShutdownException
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=0)
    p.add_argument('--cloud-topic',default='/factory_fastlio/cloud_world')
    p.add_argument('--grid-topic',default='/grid_map/occupancy_inflate');p.add_argument('--planner-node',default='/ego_planner_node')
    p.add_argument('--stream-id',default='');p.add_argument('--udp-host');p.add_argument('--udp-port',type=int,default=9005)
    p.add_argument('--grid-only',action='store_true');p.add_argument('--no-rosout',action='store_true');p.add_argument('--planner',action='store_true');args=p.parse_args()
    if not math.isfinite(args.seconds) or args.seconds<0 or not 1<=args.udp_port<=65535:p.error('时间或端口无效')
    if args.grid_only and not args.planner:p.error('--grid-only requires --planner')
    parent=os.getppid()
    if args.udp_host and parent==1:raise SystemExit('只读采样的SSH管理进程已退出')
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM) if args.udp_host else None
    if udp and not args.grid_only:
        try:udp.setsockopt(socket.IPPROTO_IP,socket.IP_TOS,0xb8)
        except OSError:pass
    rclpy.init();reader=TelemetryReader(args);executor=SingleThreadedExecutor();executor.add_node(reader.node)
    def spin():
        try:executor.spin()
        except ExternalShutdownException:pass
    worker=threading.Thread(target=spin,daemon=True);worker.start()
    start=time.monotonic();sequence=time.monotonic_ns()&0xffffffff;previous=None
    period=.5 if args.grid_only else .2 if udp else 1.
    try:
        while rclpy.ok() and (not udp or os.getppid()==parent) and (not args.seconds or time.monotonic()-start<args.seconds):
            tick=time.monotonic();packet=reader.snapshot(period);packet['sample_cost_s']=time.monotonic()-tick
            if udp:
                sequence=(sequence+1)&0xffffffff
                if args.grid_only:groups,previous=grid_transmissions(packet,sequence,previous)
                else:groups=[status_datagrams(packet,sequence)]
                for chunks in groups:
                    for datagram in chunks:udp.sendto(datagram,(args.udp_host,args.udp_port))
            else:print(json.dumps(packet,ensure_ascii=False,allow_nan=False),flush=True)
            time.sleep(max(0,period-(time.monotonic()-tick)))
    except (BrokenPipeError,KeyboardInterrupt):pass
    finally:
        executor.shutdown(timeout_sec=2.);worker.join(timeout=3.);reader.node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if udp:udp.close()


if __name__=='__main__':main()
