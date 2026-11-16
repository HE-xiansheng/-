#!/usr/bin/env python3
"""机载只读取证：参数/源码指纹、实际进程、话题频率与时间年龄、历史取消CSV。
只订阅和读文件，绝不发布设定点、调用模式/解锁服务、启动任务或写飞控参数。
在机载source ROS及任务workspace后运行；stdout为JSON，可经SSH重定向到地面文件。
"""
import argparse,csv,hashlib,json,math,os,statistics,time,threading
from collections import deque
from pathlib import Path

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,default=Path.home()/'drone_platform/01_apps/mission_demo')
    parser.add_argument('--seconds',type=float,default=8)
    args=parser.parse_args()
    if not 2<=args.seconds<=60:parser.error('--seconds must be in [2,60]')
    import rospy,rosgraph
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import PoseStamped,TwistStamped
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import State,RCIn,ExtendedState
    from std_msgs.msg import String,Int32
    rospy.init_node('gcs_aruco_readonly_audit',anonymous=True,disable_signals=True)
    result={'captured_at':time.strftime('%Y-%m-%dT%H:%M:%S%z'),'workspace':str(args.workspace),'readonly':True}
    result['parameters']={key:rospy.get_param(key,None) for key in ['/aruco_marker','/aruco_landing']}
    channel=(result['parameters'].get('/aruco_landing') or {}).get('rc_channel',5)
    cached=rospy.get_param('/mavros/param',{})
    keys=['RC_MAP_FLTMODE']+['COM_FLTMODE%d'%i for i in range(1,7)]+['RC%s_%s'%(channel,k) for k in ('MIN','TRIM','MAX','DZ','REV')]
    result['flight_parameters_cached']={key:cached.get(key) for key in keys}
    result['flight_parameter_note']='仅为ROS参数缓存；未刷新飞控参数，需与现场只读参数导出核对。'
    samples={};lock=threading.Lock();subs=[]
    def sample(topic,msg):
        now=time.monotonic();header=getattr(msg,'header',None)
        entry={'received':now}
        if header is not None:
            entry.update(source_stamp=header.stamp.to_sec(),age=(rospy.Time.now()-header.stamp).to_sec() if header.stamp.to_sec()>0 else None,seq=header.seq,frame=header.frame_id)
        if isinstance(msg,RCIn):entry.update(channels=list(msg.channels),rssi=msg.rssi)
        elif isinstance(msg,State):entry.update(connected=msg.connected,armed=msg.armed,mode=msg.mode)
        elif isinstance(msg,ExtendedState):entry['landed_state']=msg.landed_state
        elif isinstance(msg,Image):entry.update(width=msg.width,height=msg.height)
        elif isinstance(msg,String):entry['text']=msg.data[:12000]
        elif isinstance(msg,Int32):entry['id']=msg.data
        elif isinstance(msg,PoseStamped):entry['position']={k:getattr(msg.pose.position,k) for k in ('x','y','z')}
        with lock:samples.setdefault(topic,deque(maxlen=2000)).append(entry)
    topics={'/rtsp_cam/image_raw':Image,'/aruco_imgresult':Image,'/aruco_poseresult':PoseStamped,
        '/aruco_target_id':Int32,'/aruco_detector/metrics':String,'/aruco_landing/status_header':String,
        '/aruco_landing/status':String,'/mavros/rc/in':RCIn,'/mavros/state':State,
        '/mavros/extended_state':ExtendedState,'/mavros/local_position/pose':PoseStamped,
        '/mavros/local_position/velocity_local':TwistStamped,'/factory_fastlio/odometry':Odometry}
    for topic,typ in topics.items():subs.append(rospy.Subscriber(topic,typ,lambda msg,t=topic:sample(t,msg),queue_size=1,buff_size=16*1024*1024))
    master=rosgraph.Master(rospy.get_name());pubs,subscribers,services=master.getSystemState()
    result['publishers']=dict(pubs)
    # 直接读取同用户进程；不依赖只看节点名称判断程序版本。
    processes=[]
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
            if proc.stat().st_uid==os.getuid() and any(x in cmd for x in ['Safina_land','Safina_aruco','rtsp_hw_pub','gcs_task_manager','waypoint_mission']):
                processes.append({'pid':int(proc.name),'command':cmd,'exe':os.readlink(str(proc/'exe'))})
        except (OSError,ValueError):pass
    result['processes']=processes
    paths=['src/land_test/config/takeoff_land.yaml','src/land_test/src/Safina_land_test.cpp',
        'src/land_test/include/land_test/landing_task.h','src/land_test/include/land_test/rc_start_gate.h',
        'src/land_test/include/land_test/observation_geometry.h','src/aruco_test/src/Safina_aruco_test.cpp',
        'src/aruco_test/include/aruco_test/Safina_aruco_test.h','src/aruco_test/include/aruco_test/marker_model.h',
        'src/aruco_test/launch/Safina_aruco.launch','src/land_test/launch/Safina_land.launch',
        'src/rtsp_test/launch/rtsp_test.launch','devel/lib/land_test/Safina_land_test',
        'devel/lib/aruco_test/Safina_aruco_test','tools/gcs_task_manager.py']
    result['files']={}
    for name in paths:
        p=args.workspace/name
        if not p.is_file():continue
        data=p.read_bytes();entry={'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)}
        if p.suffix in ('.yaml','.launch'):entry.update(text=data.decode(errors='replace'),nul_bytes=data.count(b'\0'))
        result['files'][name]=entry
    active=Path.home()/'.cache/gcs-flight-tasks/active.json'
    try:result['task_record']=json.loads(active.read_text())
    except (OSError,ValueError):result['task_record']=None
    end=time.monotonic()+args.seconds
    while time.monotonic()<end and not rospy.is_shutdown():time.sleep(.05)
    result['topics']={}
    with lock:
        for topic in topics:
            values=list(samples.get(topic,[]));entry={'count':len(values)}
            if values:
                span=values[-1]['received']-values[0]['received']
                entry.update(hz=(len(values)-1)/span if span else None,last=values[-1],receive_age=time.monotonic()-values[-1]['received'])
                ages=[x['age'] for x in values if x.get('age') is not None]
                if ages:entry['source_age']={'min':min(ages),'median':statistics.median(ages),'max':max(ages)}
                if topic=='/mavros/rc/in':entry['raw_samples']=values
            result['topics'][topic]=entry
    # 最近5份CSV，保留取消前后各3行及末行，避免把当前RC误认作历史触发值。
    dirs=[Path.home()/'.ros/aruco_landing']
    record=result['task_record'] or {}
    if record.get('run'):dirs.append(Path(record['run'])/'landing-diagnostics')
    paramdir=(result['parameters'].get('/aruco_landing') or {}).get('diagnostic_directory')
    if paramdir:dirs.append(Path(paramdir))
    csvs=sorted({p for d in dirs for p in d.glob('landing_*.csv')},key=lambda p:p.stat().st_mtime,reverse=True)[:5]
    result['landing_logs']=[]
    for p in csvs:
        try:
            with p.open() as f:rows=list(csv.DictReader(line for line in f if not line.startswith('#')))
            events=set([len(rows)-1])
            for i,row in enumerate(rows):
                if row.get('phase')=='CANCELLED' and (i==0 or rows[i-1].get('phase')!='CANCELLED'):events.update(range(max(0,i-3),min(len(rows),i+4)))
            result['landing_logs'].append({'path':str(p),'rows':len(rows),'events':[rows[i] for i in sorted(events) if i>=0]})
        except (OSError,ValueError,csv.Error) as e:result['landing_logs'].append({'path':str(p),'error':str(e)})
    def finite(value):
        if isinstance(value,float) and not math.isfinite(value):return None
        if isinstance(value,dict):return {k:finite(v) for k,v in value.items()}
        if isinstance(value,(list,tuple)):return [finite(v) for v in value]
        return value
    print(json.dumps(finite(result),ensure_ascii=False,indent=2,allow_nan=False))
if __name__=='__main__':main()
