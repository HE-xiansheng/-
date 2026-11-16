#!/usr/bin/env python3
"""Diff-Planner -> PX4 internal PVA controller. No arming/mode/parameter writes."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import argparse,copy,fcntl,hashlib,json,math,signal,threading,time
from collections import deque
from pathlib import Path
import numpy as np
from execution import Alignment,Trajectory,Session,rotation,interpolate_pose

COMMANDS={'/mavros/setpoint_position/local','/mavros/setpoint_position/global','/mavros/setpoint_position/global_to_local',
 '/mavros/setpoint_raw/local','/mavros/setpoint_raw/global','/mavros/setpoint_raw/attitude',
 '/mavros/setpoint_velocity/cmd_vel','/mavros/setpoint_velocity/cmd_vel_unstamped','/mavros/setpoint_accel/accel',
 '/mavros/setpoint_attitude/attitude','/mavros/setpoint_attitude/thrust','/mavros/setpoint_attitude/cmd_vel',
 '/mavros/setpoint_attitude/cmd_vel_unstamped','/mavros/actuator_control','/mavros/rc/override','/mavros/setpoint_trajectory/local'}

def atomic(path,data):
    tmp=path.with_suffix('.next');tmp.write_text(json.dumps(data,ensure_ascii=False,allow_nan=False,indent=2));os.replace(tmp,path)

def pose(msg):
    v=msg.pose.pose if hasattr(msg.pose,'pose') else msg.pose
    p=np.array([v.position.x,v.position.y,v.position.z]);q=np.array([v.orientation.x,v.orientation.y,v.orientation.z,v.orientation.w])
    if not np.isfinite(p).all():raise ValueError('nonfinite position')
    R=rotation(q)
    return p,q,math.atan2(R[1,0],R[0,0])

def paired_pose(history,stamp):
    before=max((m for m in history if m.header.stamp.to_sec()<=stamp),key=lambda m:m.header.stamp.to_sec(),default=None)
    after=min((m for m in history if m.header.stamp.to_sec()>=stamp),key=lambda m:m.header.stamp.to_sec(),default=None)
    if before is not None and after is not None:
        t0=before.header.stamp.to_sec();t1=after.header.stamp.to_sec()
        if t0==t1:return pose(before)[:2]
        if stamp-t0<=.05 and t1-stamp<=.05:
            p0,q0,_=pose(before);p1,q1,_=pose(after)
            return interpolate_pose(t0,p0,q0,t1,p1,q1,stamp)
    nearest=min(history,key=lambda m:abs(m.header.stamp.to_sec()-stamp))
    if abs(nearest.header.stamp.to_sec()-stamp)>.05:raise ValueError('pose_pair_time')
    return pose(nearest)[:2]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--publish',action='store_true');parser.add_argument('--check',action='store_true')
    parser.add_argument('--isolated-test',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--run',type=Path,required=True);args=parser.parse_args()
    if args.publish and args.check:parser.error('Choose check or publish')
    expected='http://127.0.0.1:11377' if args.isolated_test else 'http://127.0.0.1:11311'
    if os.environ.get('ROS_MASTER_URI')!=expected:raise RuntimeError('Wrong ROS master')
    import rospy,rosgraph
    from mavros_msgs.msg import State,ExtendedState,EstimatorStatus,RCIn,PositionTarget
    from mavros_msgs.srv import ParamGet
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped,TwistStamped
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String,Bool
    from traj_utils.msg import PolyTraj
    args.run.mkdir(parents=True,exist_ok=True)
    if args.publish:
        singleton=(args.run.parent/'execution-publisher.lock').open('a');fcntl.flock(singleton,fcntl.LOCK_EX|fcntl.LOCK_NB)
    rospy.init_node('diff_planner_px4_execution',anonymous=not args.publish,disable_signals=True)
    cache={};lock=threading.RLock();history=deque(maxlen=150);polycache=deque(maxlen=10);planner_session=[None]
    topics={'state':('/mavros/state',State,2.),'landed':('/mavros/extended_state',ExtendedState,2.),
      'ekf':('/mavros/estimator_status',EstimatorStatus,2.),'fcpose':('/mavros/local_position/pose',PoseStamped,.4),
      'fcvel':('/mavros/local_position/velocity_local',TwistStamped,.4),'lio':('/factory_fastlio/odometry',Odometry,.4),
      'relay':('/mavros/vision_pose/pose',PoseStamped,.5),'rc':('/mavros/rc/in',RCIn,.5),
      'cloud':('/factory_fastlio/cloud_world',PointCloud2,.5),'status':('/diff_planner/manual_preview/status',String,.6)}
    def receive(msg,key):
        with lock:
            if key=='status':
                try:
                    incoming=json.loads(msg.data).get('session_id')
                    if incoming is not None and incoming!=planner_session[0]:
                        polycache.clear();planner_session[0]=incoming
                except (ValueError,TypeError):pass
            cache[key]=(msg,time.monotonic())
            if key=='fcpose':history.append(msg)
    def poly(msg):
        with lock:polycache.append((msg,time.monotonic()))
    subs=[rospy.Subscriber(topic,typ,receive,callback_args=k,queue_size=1) for k,(topic,typ,_) in topics.items()]
    subs.append(rospy.Subscriber('/diff_planner/flight_execution/trajectory',PolyTraj,poly,queue_size=1))
    pub=None;modepub=rospy.Publisher('/diff_planner/flight_execution',Bool,queue_size=1,latch=True) if args.publish else None
    statuspub=rospy.Publisher('/diff_planner/px4_execution/'+('status' if args.publish else 'check_status'),String,queue_size=1,latch=True)
    parameters={'policy':'native_planner_parameters_no_additional_gate'}
    stopped=[False]
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,lambda *_:stopped.__setitem__(0,True))
    session=Session();alignment=None;anchor=None;active=None;activekey=None;lastgoal=None;frozen_yaw=None;execution_mode=None
    atomic(args.run/'source.json',{'parameters_read_only':parameters,'publish_requested':args.publish,
        'policy':'upstream_fsm_native_clock_persistent_pilot_cycles','sha256':{name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest() for name in ('adapter.py','execution.py')}})
    lastloop=time.monotonic();lastlog=0.;lastgraph=0.;lastphase=None;lastpacket={};competing={};check_exit=0;check_deadline=lastloop+6.
    try:
      with (args.run/'samples.jsonl').open('a',buffering=1) as logfile:
       while not rospy.is_shutdown():
        now=time.monotonic();rosnow=rospy.Time.now().to_sec();gap=now-lastloop;lastloop=now
        with lock:data=dict(cache);poses=list(history);polys=list(polycache)
        hard=[];warnings=[];errors=[];status={};fc=data.get('state',(State(),))[0];landed=data.get('landed',(ExtendedState(),))[0].landed_state
        if 'status' in data:
            try:status=json.loads(data['status'][0].data)
            except (ValueError,TypeError):warnings.append('planner_status_format_invalid')
        fp=fq=lp=lq=pairp=None;current=None;speed=None
        try:
            fp,fq,yaw=pose(data['fcpose'][0]);lp,lq,_=pose(data['lio'][0])
            # 保留坐标转换运算；删除坐标系名称白名单和时间配对拒绝。
            pairp,pairq=fp,fq;current=(*fp,yaw)
            if 'fcvel' in data:
                v=data['fcvel'][0].twist.linear;speed=float(np.linalg.norm([v.x,v.y,v.z]))
        except (KeyError,ValueError,TypeError):hard.append('pose_or_frame_invalid')
        healthy=not hard;stop=stopped[0] or (args.run/'stop-request').exists()
        confirmed=True
        requested=bool(args.publish and fc.mode=='OFFBOARD' and fc.armed and landed==2 and not stop and confirmed)
        if requested!=execution_mode:
            if modepub:modepub.publish(Bool(data=requested))
            execution_mode=requested;active=None;activekey=None
            with lock:polycache.clear()
            polys=[]
            if requested:alignment=None
        if not session.entered and healthy and fc.armed and landed==2 and fc.mode in ('POSCTL','OFFBOARD'):
            alignment=Alignment(lp,lq,pairp,pairq);anchor=fp.copy();frozen_yaw=current[3]
        prepared=alignment is not None and current is not None
        key=(status.get('session_id'),status.get('goal_id'))
        lastgoal=key
        status_ok=bool(status)
        native=bool(status.get('planning_mode')=='upstream_fsm')
        # Native FSM keeps the last valid trajectory during a replan, or sends its own emergency stop trajectory.
        eligible=bool(prepared and fc.mode=='OFFBOARD')
        ref=None;frame_offset=None
        if eligible:
            matches=[m for m,_ in polys]
            if matches:
                msg=matches[-1];identity=(key,msg.traj_id,msg.start_time.to_sec())
                try:
                    if identity!=activekey:
                        active=Trajectory({'id':msg.traj_id,'start':msg.start_time.to_sec(),'order':msg.order,'duration':list(msg.duration),'x':list(msg.coef_x),'y':list(msg.coef_y),'z':list(msg.coef_z)},alignment.at_pose(lp,pairp),anchor);activekey=identity
                    frame_offset=active.translation_offset(lp,pairp);p,v,a=active.sample(rosnow,position_offset=frame_offset)
                    ref={'p':p.tolist(),'v':v.tolist(),'a':a.tolist(),'yaw':frozen_yaw}
                except (ValueError,TypeError,KeyError,IndexError,OverflowError):errors.append('invalid_trajectory');active=None;activekey=None
            else:errors.append('await_upstream_trajectory')
        if status.get('phase') not in ('ACTIVE','WAIT_TARGET','EMERGENCY_STOP'):
            errors.append('planner:'+str(status.get('phase','missing')))
        target=session.step(now,rosnow,fc.mode,fc.armed,landed==2,healthy,prepared,ref is not None,current,ref,stop=stop)
        if session.entered and session.entry_ros==rosnow:frozen_yaw=current[3]
        if session.phase=='TRACK' and native and status.get('phase') in ('EMERGENCY_STOP','WAIT_TARGET'):
            session.phase='UPSTREAM_HOLD';session.reason='upstream:'+status['phase']
        emitted=False
        if args.publish and target is not None:
            if pub is None:pub=rospy.Publisher('/mavros/setpoint_raw/local',PositionTarget,queue_size=1)
            out=PositionTarget();out.header.stamp=rospy.Time.now();out.header.frame_id='map';out.coordinate_frame=PositionTarget.FRAME_LOCAL_NED;out.type_mask=PositionTarget.IGNORE_YAW_RATE
            if isinstance(target,dict):p,v,a,y=target['p'],target['v'],target['a'],target['yaw']
            else:p,v,a,y=target[:3],[0.,0.,0.],[0.,0.,0.],target[3]
            out.position.x,out.position.y,out.position.z=p;out.velocity.x,out.velocity.y,out.velocity.z=v;out.acceleration_or_force.x,out.acceleration_or_force.y,out.acceleration_or_force.z=a;out.yaw=y
            # Re-read mode immediately before publishing, so an RC takeover during polynomial evaluation wins.
            with lock:
                latest=cache.get('state',(State(),))[0]
                if session.entered and latest.mode!='OFFBOARD':
                    target=None;session.manual_takeover(latest.mode,stop=stop);fc=latest
                else:pub.publish(out);emitted=True
        if now-lastlog>=.1 or session.phase!=lastphase:
            lastpacket={'stamp':rosnow,'phase':session.phase,'reason':session.reason,'mode':fc.mode,'armed':fc.armed,'airborne':landed==2,
              'hard_errors':sorted(set(hard)),'planning_errors':sorted(set(errors)),'warnings':sorted(set(warnings)),'competing_publishers':competing,
              'goal_key':key,'output_enabled':emitted,'publish_requested':args.publish,'time_scale':1. if active else None,
              'cycle_id':session.cycle_id,'tracking_output_enabled':bool(emitted and session.entered and session.phase=='TRACK'),
              'output_kind':('path_tracking' if session.entered and session.phase=='TRACK' else 'upstream_hold' if session.phase=='UPSTREAM_HOLD' else 'prestream_current_pose' if fc.mode=='POSCTL' else 'position_hold') if emitted else 'none',
              'limits':{'source':'planner_parameters','horizontal_speed':1.,'vertical_speed':.3,'acceleration':.5,'acceleration_tolerance':1.,'acceleration_check':'upstream_3d_norm'},
              'measured_fc_p':fp.tolist() if fp is not None else None,'measured_fc_q_xyzw':fq.tolist() if fq is not None else None,
              'measured_lio_p':lp.tolist() if lp is not None else None,'measured_lio_q_xyzw':lq.tolist() if lq is not None else None,
              'alignment_R':alignment.R.tolist() if alignment else None,'alignment_t':alignment.t.tolist() if alignment else None,
              'trajectory_frame_t':active.frame_t.tolist() if active else None,'live_translation_offset':frame_offset.tolist() if frame_offset is not None else None,
              'paired_fc_p':pairp.tolist() if pairp is not None else None,'alignment_policy':'latest_pose_translation_frozen_rotation','trajectory_clock':'upstream_native',
              'anchor_fc':anchor.tolist() if anchor is not None else None,'trajectory_id':active.ident if active else None,'trajectory_start':active.start if active else None,
              'input_ages':{k:{'receive':now-r,'source':rosnow-m.header.stamp.to_sec() if hasattr(m,'header') else None} for k,(m,r) in data.items()},
              'control_loop_gap':gap,'measured_speed':speed,'replan_recoveries':session.replan_recoveries,'planner_status':status,
              'parameters_read_only':parameters,'reference_fc_enu':target,'note':'RC mode switch owns handover; no arming/mode/parameter writes; native planner controls replans and emergency stops.'}
            atomic(args.run/'status.json',lastpacket);logfile.write(json.dumps(lastpacket,ensure_ascii=False)+'\n');statuspub.publish(String(data=json.dumps(lastpacket,ensure_ascii=False)))
            if session.phase!=lastphase:print(json.dumps(lastpacket,ensure_ascii=False),flush=True)
            lastphase=session.phase;lastlog=now
        if not args.publish and time.monotonic()>=check_deadline:
            check_exit=1 if hard else 0;break
        if session.phase=='DONE':break
        time.sleep(max(0.,.02-(time.monotonic()-now)))
    finally:
        if pub:pub.unregister()
        if modepub:modepub.publish(Bool(data=False));rospy.sleep(.05)
        lastpacket.update(phase='DONE',output_enabled=False,reason=session.reason or 'process_ended');atomic(args.run/'status.json',lastpacket);statuspub.publish(String(data=json.dumps(lastpacket,ensure_ascii=False)))
        for sub in subs:sub.unregister()
    return check_exit

if __name__=='__main__':raise SystemExit(main())
