#!/usr/bin/env python3
"""真实ROS2图/消息/只读参数→遥测快照，在域198本机隔离验证；不调用飞控服务。"""
import argparse,copy,importlib.util,json,math,os,sys,threading,time
from pathlib import Path
from types import SimpleNamespace
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2,PointField,Imu,FluidPressure,BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State,ExtendedState,EstimatorStatus,RCIn,StatusText,PositionTarget
from livox_ros_driver2.msg import CustomMsg,CustomPoint
from traj_utils.msg import PolyTraj


def main():
    p=argparse.ArgumentParser();p.add_argument('--reader',required=True);p.add_argument('--output',required=True);p.add_argument('--serve',type=float,default=0);args=p.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='198' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    spec=importlib.util.spec_from_file_location('telemetry_reader',args.reader);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    sys.path.insert(0,str(Path(args.reader).parent))
    from bridge import format_packet
    from stream_reader import StatusAssembler,decode_grid
    from planner_display import snapshot
    checks=[];out=Path(args.output);out.mkdir(exist_ok=True)
    def check(name,value):
        checks.append({'name':name,'passed':bool(value)});print('PASS' if value else 'FAIL',name,flush=True)
        if not value:raise AssertionError(name)
    rclpy.init();nodes={name:rclpy.create_node(name,enable_rosout=False) for name in ['mavros','factory_laserMapping','factory_vision2mavros','livox_lidar_publisher','ego_planner_node']}
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    settings=SimpleNamespace(cloud_topic='/factory_fastlio/cloud_world',grid_topic='/grid_map/occupancy_inflate',planner_node='/ego_planner_node',no_rosout=True,planner=True,grid_only=False,stream_id='fixture')
    reader=module.TelemetryReader(settings);grid_reader=module.TelemetryReader(SimpleNamespace(**dict(vars(settings),grid_only=True)))
    planner=nodes['ego_planner_node'];planner.declare_parameter('grid_map.resolution',.1)
    executor=SingleThreadedExecutor()
    for n in [*nodes.values(),reader.node,grid_reader.node]:executor.add_node(n)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    pubs={};msgs={}
    def pub(key,typ,topic,name,msg=None,qos=1):
        pubs[key]=nodes[name].create_publisher(typ,topic,qos);msgs[key]=msg or typ()
    for key,(topic,typ,_) in reader.definitions.items():
        name='factory_laserMapping' if key in ('lio','cloud') else 'factory_vision2mavros' if key=='relay' else 'livox_lidar_publisher' if key in ('imu','lidar') else 'mavros'
        pub(key,typ,topic,name,qos=qos_profile_sensor_data if key not in ('lio','cloud','lidar','relay') else 1)
    for key in ['lio','fc_odom']:
        msgs[key].header.frame_id='camera_init' if key=='lio' else 'map';msgs[key].child_frame_id='body';msgs[key].pose.pose.orientation.w=1.;msgs[key].pose.pose.position.z=.8
    msgs['relay'].header.frame_id='map';msgs['relay'].pose.orientation.w=1.
    for key in ['cloud']:
        msgs[key].header.frame_id='camera_init';msgs[key].height=1;msgs[key].width=1;msgs[key].point_step=12;msgs[key].row_step=12;msgs[key].fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')];msgs[key].data=__import__('struct').pack('<fff',.05,.15,.25)
    # 使用有效的小型点序列，点数须与CDR序列长度一致。
    msgs['lidar'].point_num=3;msgs['lidar'].points=[CustomPoint() for _ in range(3)]
    for key in ['imu','fc_imu']:msgs[key].linear_acceleration.z=9.8;msgs[key].angular_velocity.y=.1
    msgs['pressure'].fluid_pressure=101500.
    msgs['fc'].connected=True;msgs['fc'].mode='POSCTL';msgs['fc'].system_status=3
    msgs['landed'].landed_state=1
    for k in ['attitude_status_flag','velocity_horiz_status_flag','velocity_vert_status_flag','pos_horiz_rel_status_flag','pos_vert_abs_status_flag']:setattr(msgs['ekf'],k,True)
    msgs['rc'].channels=[1500]*8;msgs['rc'].rssi=255
    msgs['battery'].voltage=24.;msgs['battery'].percentage=.8;msgs['battery'].current=float('nan')
    msgs['text'].severity=6;msgs['text'].text='synthetic fixture only'
    pub('grid',PointCloud2,settings.grid_topic,'ego_planner_node',copy.deepcopy(msgs['cloud']))
    pub('status',String,'/ego_planner/manual_preview/status','ego_planner_node',qos=retained)
    pub('execution',String,'/ego_planner/px4_execution/status','ego_planner_node',qos=retained)
    pub('trajectory',PolyTraj,'/ego_planner/manual_preview/trajectory','ego_planner_node',qos=retained)
    tr=msgs['trajectory'];tr.traj_id=7;tr.order=5;tr.duration=[1.];tr.coef_x=[0.,0.,0.,0.,1.,0.];tr.coef_y=[0.]*6;tr.coef_z=[0.,0.,0.,0.,0.,1.]
    status={'mode':'manual_preview','phase':'ACTIVE','session_id':'fixture-session','trajectory_id':7,'planning_mode':'upstream_fsm','goal_frame':'camera_init','height_reference':'fastlio_session_origin','candidate':{'available':False}}
    def publish(duration=.65,exclude=()):
        until=time.monotonic()+duration
        while time.monotonic()<until:
            stamp=planner.get_clock().now().to_msg();now=stamp.sec+stamp.nanosec*1e-9
            status['stamp']=now;msgs['status'].data=json.dumps(status);msgs['execution'].data=json.dumps({'stamp':now,'phase':'WAIT_OFFBOARD','output_enabled':False})
            tr.start_time=stamp
            for key,p in pubs.items():
                if key in exclude:continue
                if hasattr(msgs[key],'header'):msgs[key].header.stamp=stamp
                p.publish(msgs[key])
            time.sleep(.04)
    try:
        publish(3.2)
        if args.serve:
            print('FIXTURE_READY',flush=True);publish(args.serve);return
        packet=reader.snapshot(.2);(out/'healthy.json').write_text(json.dumps(packet,ensure_ascii=False,allow_nan=False,indent=2)+'\n')
        formatted=format_packet(packet,'192.168.1.224')
        check('all preflight rows healthy from typed ROS2 messages',all(r['state']=='正常' for r in formatted['rows']))
        check('source frequency from callbacks instead of 5Hz monitor',15<packet['topics']['imu']['hz']<30)
        check('small sensor fields and custom Livox count',packet['topics']['lidar']['values']=={'point_count':3} and packet['topics']['pressure']['values']=={'pressure_pa':101500.})
        check('nonfinite battery field becomes null',packet['topics']['battery']['values']['current'] is None)
        check('frame and body pose retained',packet['topics']['lio']['frame']=='camera_init' and packet['topics']['lio']['values']['child_frame_id']=='body')
        check('ROS2 graph publisher identity and source nodes',packet['topics']['lio']['publishers']==['/factory_laserMapping'] and packet['localization']['lio_running'] and packet['localization']['relay_running'] and packet['localization']['session_id'])
        check('native recording readonly status',packet['recording']['available'] is True and packet['recording']['phase']=='NOT_RECORDING' and packet['recording']['running'] is False)
        check('native camera_init trajectory visible without candidate workflow',formatted['planning']['path_valid'] and not formatted['planning']['candidate_path_valid'])
        g=grid_reader.snapshot(.5);row=g['ego_planner'].get('grid',{})
        check('asynchronous read-only native grid parameter',row.get('data',{}).get('resolution')==.1)
        grid=copy.deepcopy(packet['ego_planner']);grid['grid']=row
        check('grid frame session source time accepted',snapshot(grid)['grid_valid'] and len(decode_grid(row['data']))==12)
        datagrams=list(module.status_datagrams(packet,17));assembler=StatusAssembler();decoded=None
        for data in reversed(datagrams):decoded=assembler.accept(data) or decoded
        check('unchanged TLM1 JSON compression and fragment format',decoded==packet)
        check('no control publishers from read-only monitors',not packet['control_publishers'])
        dup=rclpy.create_node('factory_laserMapping',enable_rosout=False);dp=dup.create_publisher(Odometry,'/factory_fastlio/odometry',1)
        publish(1.1);conflict=reader.snapshot(.2)
        check('duplicate same-name publisher remains a conflict',conflict['topics']['lio']['publishers']==['/factory_laserMapping']*2 and not conflict['localization']['session_id'] and next(r for r in format_packet(conflict,'host')['rows'] if r['key']=='lio')['state']=='来源冲突')
        dup.destroy_node();publish(.7)
        feedback=nodes['mavros'].create_publisher(PositionTarget,'/mavros/setpoint_raw/target_local',1);publish(.7)
        check('MAVROS target feedback not treated as command',not reader.snapshot(.2)['control_publishers'])
        command=nodes['mavros'].create_publisher(PositionTarget,'/mavros/setpoint_raw/local',1);publish(.7)
        check('external command publisher detected without sending command messages','/mavros/setpoint_raw/local' in reader.snapshot(.2)['control_publishers'])
        nodes['mavros'].destroy_publisher(command)
        publish(.8,exclude=('lio','cloud','relay','imu','lidar','fc_odom'))
        old=format_packet(reader.snapshot(.2),'host')
        check('source stream stop marks stale while monitor stays online',all(r['state']=='缺失/过期' for r in old['rows'] if r['key'] in ('lio','cloud','relay','imu','lidar','fc_odom')))
        msgs['lio'].pose.pose.position.x=float('nan');publish(.65)
        check('invalid pose does not poison JSON and is explicitly rejected',next(r for r in format_packet(reader.snapshot(.2),'host')['rows'] if r['key']=='lio')['state']=='位姿无效')
        msgs['lio'].pose.pose.position.x=0.;publish(.7)
        with reader.lock:before=reader.planning['status']['receipt']
        pubs['status'].publish(String(data='{"stamp": NaN}'));time.sleep(.1)
        with reader.lock:after=reader.planning['status']['receipt']
        check('nonfinite status rejected without refreshing old cache',before==after)
        with grid_reader.lock:before=grid_reader.planning['grid']['receipt']
        msgs['grid'].header.frame_id='wrong';publish(.8)
        with grid_reader.lock:after=grid_reader.planning['grid']['receipt']
        check('wrong-frame grid cannot refresh previous cache',before==after)
        msgs['grid'].header.frame_id='camera_init';status['session_id']='new-session';publish(.8)
        check('session restart clears old trajectory or attaches current only',reader.snapshot(.2)['ego_planner']['status']['data']['session_id']=='new-session' and grid_reader.snapshot(.5)['ego_planner']['grid']['data']['session_id']=='new-session')
        (out/'formatted.json').write_text(json.dumps(formatted,ensure_ascii=False,indent=2)+'\n')
    finally:
        executor.shutdown(timeout_sec=2.);thread.join(timeout=3.)
        for n in [reader.node,grid_reader.node,*nodes.values()]:n.destroy_node()
        rclpy.shutdown()
        (out/'result.json').write_text(json.dumps({'passed':all(c['passed'] for c in checks),'checks':checks},ensure_ascii=False,indent=2)+'\n')

if __name__=='__main__':main()
