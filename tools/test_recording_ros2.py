#!/usr/bin/env python3
"""在独立DDS域运行真实rosbag2；所有飞控观测均为合成，绝无飞控调用。"""
import argparse,importlib,os,json,signal,sqlite3,sys,threading,time,struct,shutil
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();root=a.root
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and os.environ.get('ROS_DOMAIN_ID')=='89'
os.environ['RECORDING_MAX_BAG_SIZE']='131072'
sys.path.insert(0,str(root));import recording_manage as m;import recording_auto as auto
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from mavros_msgs.msg import State,ExtendedState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import serialize_message
rclpy.init();node=rclpy.create_node('recording_fixture');checks=[];halt=threading.Event()
q=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
sp=node.create_publisher(State,'/mavros/state',q);lp=node.create_publisher(ExtendedState,'/mavros/extended_state',q)
op=node.create_publisher(Odometry,'/factory_fastlio/odometry',q);cp=node.create_publisher(PointCloud2,'/factory_fastlio/cloud_world',q)
tp=node.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
values={'armed':False,'mode':'POSCTL','landed':1,'pause':False};seq=[0]
def feed():
    while not halt.is_set():
        rclpy.spin_once(node,timeout_sec=.001)
        if not values['pause']:
            seq[0]+=1;stamp=node.get_clock().now().to_msg()
            s=State();s.header.stamp=stamp;s.connected=True;s.armed=values['armed'];s.mode=values['mode'];sp.publish(s)
            l=ExtendedState();l.header.stamp=stamp;l.landed_state=values['landed'];lp.publish(l)
            o=Odometry();o.header.stamp=stamp;o.header.frame_id='camera_init';o.child_frame_id='body';o.pose.pose.position.x=seq[0]*.01;o.pose.pose.orientation.w=1.;op.publish(o)
            if seq[0]%5==0:
                c=PointCloud2();c.header=o.header;c.height=1;c.width=1200;c.point_step=16;c.row_step=19200
                c.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z','intensity'))]
                c.data=b''.join(struct.pack('<ffff',float(i%50)*.1,float(i//50)*.1,1.,.8) for i in range(1200));cp.publish(c)
        halt.wait(.02)
t=TransformStamped();t.header.stamp=node.get_clock().now().to_msg();t.header.frame_id='camera_init';t.child_frame_id='fixture';t.transform.rotation.w=1.;tp.publish(TFMessage(transforms=[t]))
thread=threading.Thread(target=feed);thread.start()
def check(v,label):
    assert v,label;checks.append(label)
def wait(wanted,timeout=30):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        h=m.health(root)
        if h.get('phase')=='ERROR':raise AssertionError(h)
        if h.get('phase') in wanted:return h
        time.sleep(.1)
    raise AssertionError(m.health(root))
def rows(run):
    result={}
    for path in Path(run).rglob('*.db3'):
        with sqlite3.connect(str(path)) as db:
            for name,count in db.execute('SELECT topics.name,count(*) FROM messages JOIN topics ON topics.id=messages.topic_id GROUP BY topics.id'):result[name]=result.get(name,0)+count
    return result
result={}
try:
    time.sleep(1)
    check(not m.health(root)['auto_enabled'],'auto defaults off')
    req=m.request('start',root);first=wait({'RECORDING'});check(req['requested']=='start','manual request acknowledged')
    time.sleep(3)
    check(m.health(root)['phase']=='RECORDING','manual records disarmed POSCTL')
    check(m.health(root)['missing_topics'],'absent topics reported without failing recorder')
    same=m.request('start',root);check(same['run']==first['run'],'start idempotent')
    values.update(armed=True,mode='OFFBOARD',landed=2);time.sleep(.5)
    m.request('stop',root);saved=wait({'STOPPED'});check(not list(Path(saved['run']).glob('*.active')),'normal stop clears active marker')
    check(saved['bag_pid'] is None and saved['bag_bytes']>0,'stop finalizes independently of OFFBOARD')
    check(len(list(Path(saved['run']).glob('inputs-*/*.db3')))>1,'actual rosbag2 split finalization')
    counts=rows(saved['run']);check(counts.get('/factory_fastlio/odometry',0)>100,'real sqlite odometry messages')
    check(counts.get('/factory_fastlio/cloud_world',0)>10,'best effort cloud recorded')
    check(counts.get('/tf_static',0)>=1,'late transient TF retained')
    check((Path(saved['run'])/'manifest.json').is_file(),'runtime source hash archive')
    check((Path(saved['run'])/'params-before.json').is_file(),'readonly parameter snapshot')
    result['manual_run']=saved['run'];result['manual_counts']=counts
    # 真正数据结构生成两分包，供电脑原生CDR/SQLite查看器验证。
    import rosbag2_py
    fixture=root/'replay-fixture';fixture.mkdir()
    for part in range(2):
        writer=rosbag2_py.SequentialWriter();uri=fixture/('part%d'%part)
        writer.open(rosbag2_py.StorageOptions(uri=str(uri),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
        for name,typ in [('/factory_fastlio/cloud_world','sensor_msgs/msg/PointCloud2'),('/factory_fastlio/odometry','nav_msgs/msg/Odometry'),('/mavros/local_position/pose','geometry_msgs/msg/PoseStamped')]:
            writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=typ,serialization_format='cdr'))
        from geometry_msgs.msg import PoseStamped
        for i in range(120):
            n=part*120+i;ns=1700000000000000000+n*20000000
            o=Odometry();o.header.stamp.sec=ns//1000000000;o.header.stamp.nanosec=ns%1000000000;o.header.frame_id='camera_init';o.child_frame_id='body';o.pose.pose.position.x=n*.01;o.pose.pose.orientation.w=1.
            writer.write('/factory_fastlio/odometry',serialize_message(o),ns)
            bad=PoseStamped();bad.header=o.header;bad.pose.position.x=999.;bad.pose.orientation.w=1.;writer.write('/mavros/local_position/pose',serialize_message(bad),ns)
            if i%5==0:
                c=PointCloud2();c.header=o.header;c.height=1;c.width=1200;c.point_step=16;c.row_step=19200;c.fields=[PointField(name=k,offset=j*4,datatype=7,count=1) for j,k in enumerate(('x','y','z','intensity'))];c.data=struct.pack('<ffff',1.,2.,3.,.7)*1200
                writer.write('/factory_fastlio/cloud_world',serialize_message(c),ns)
        del writer
    values.update(armed=False,mode='POSCTL',landed=1);time.sleep(.5)
    auto.configure(True,root);wait({'WAIT_MANUAL'});time.sleep(1)
    check(m.health(root)['auto_fresh'],'native auto observer fresh')
    check(m.health(root)['phase']=='WAIT_MANUAL','auto prepares without recording')
    values.update(armed=True,mode='OFFBOARD',landed=2);active=wait({'RECORDING'});time.sleep(.5)
    values.update(mode='POSCTL');time.sleep(.5);check(m.health(root)['phase']=='RECORDING','mode exit keeps recording')
    values.update(landed=1);time.sleep(.5);check(m.health(root)['phase']=='RECORDING','landing alone keeps recording')
    values.update(armed=False,landed=2);time.sleep(.5);check(m.health(root)['phase']=='RECORDING','disarm alone keeps recording')
    values.update(pause=True);time.sleep(2.4);values.update(landed=1);time.sleep(.5);check(m.health(root)['phase']=='RECORDING','stale source cannot stop')
    values.update(pause=False);wait({'STOPPED'});check(True,'fresh landing and disarm finalize')
    values.update(armed=True,mode='OFFBOARD',landed=2);wait({'RECORDING'});auto.configure(False,root);time.sleep(.5)
    check(m.health(root)['phase']=='RECORDING','disable does not discard active recording')
    m.request('stop',root);wait({'STOPPED'});time.sleep(.5);check(m.health(root)['phase']=='STOPPED','manual stop does not auto restart')
    # 中断监督器会收尾自己的子进程；标记遗留后拒绝覆盖。
    m.request('start',root);wait({'RECORDING'});info=m.current(root);os.kill(info['pid'],signal.SIGTERM)
    end=time.monotonic()+25
    while m.current(root) and time.monotonic()<end:time.sleep(.1)
    check(m.current(root) is None,'supervisor termination bounded')
    h=m.health(root);check(h['phase']=='STOPPED' and not list(Path(h['run']).glob('*.active')),'supervisor termination finalizes child')
    marker=Path(h['run'])/'interrupted.active';marker.touch()
    try:m.request('start',root);raise AssertionError('accepted interrupted recording')
    except RuntimeError:check(True,'unfinalized data blocks replacement')
    marker.unlink()
    pubs=[topic for topic in ('/mavros/setpoint_raw/local','/ego_planner/flight_execution') if node.get_publishers_info_by_topic(topic)]
    check(not pubs,'no flight/control publishers')
    result.update(passed=True,checks=checks)
finally:
    for info in (auto.current(root),m.current(root)):
        if info:
            try:os.kill(info['pid'],signal.SIGTERM)
            except ProcessLookupError:pass
    halt.set();thread.join(3);node.destroy_node();rclpy.try_shutdown()
    result['completed_checks']=checks;(root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
