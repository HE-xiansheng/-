#!/usr/bin/env python3
"""精准降落其他触发路径：localhost域90，模拟飞控，不访问真实MAVROS/云台。"""
import csv,json,math,os,signal,subprocess,time
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='90' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseStamped,TwistStamped
from mavros_msgs.msg import State,ExtendedState,PositionTarget
from mavros_msgs.srv import SetMode,CommandBool
from std_srvs.srv import SetBool
from std_msgs.msg import String

rclpy.init();node=rclpy.create_node('landing_modes_mock_px4');out=Path(os.environ['LAND_MODES_OUTPUT']);out.mkdir(exist_ok=True)
checks=[];case=None;schema=[];rows=[];commands=[];calls=[];arms=[]
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String,'/aruco_landing/status_header',lambda m:schema.append(m.data),q)
def status(m):
    if schema:rows.append((time.monotonic(),dict(zip(next(csv.reader([schema[-1]])),next(csv.reader([m.data]))))))
node.create_subscription(String,'/aruco_landing/status',status,q)
node.create_subscription(PositionTarget,'/mavros/setpoint_raw/local',lambda m:commands.append((time.monotonic(),m)),100)
pubs={k:node.create_publisher(t,p,1) for k,t,p in [('state',State,'/mavros/state'),('landed',ExtendedState,'/mavros/extended_state'),('pose',PoseStamped,'/mavros/local_position/pose'),('vel',TwistStamped,'/mavros/local_position/velocity_local')]}
value={};services=[]
def mode(req,res):
    calls.append((time.monotonic(),req.custom_mode));res.mode_sent=sum(c==req.custom_mode for _,c in calls)>1
    if res.mode_sent:value['mode']=req.custom_mode
    return res
def arm(req,res):
    arms.append((time.monotonic(),req.value));res.success=len(arms)>1
    if res.success:value['armed']=req.value
    return res
client=node.create_client(SetBool,'/aruco_landing/enable')
def check(label,condition):
    checks.append({'case':case,'check':label,'passed':bool(condition)});print(json.dumps(checks[-1]),flush=True)
    if not condition:raise AssertionError(label)
def pump(seconds=0.,until=None):
    deadline=time.monotonic()+seconds;previous=time.monotonic();sent=0.
    while time.monotonic()<deadline:
        now=time.monotonic();dt=now-previous;previous=now
        cmd=commands[-1][1] if commands and now-commands[-1][0]<.15 else None
        if value.get('integrate') and value['armed'] and value['mode']=='OFFBOARD' and cmd:
            value['z']=max(0.,value['z']+cmd.velocity.z*dt)
            if value['z']>.04:value['landed']=2
        if now-sent>=.025:
            sent=now;stamp=node.get_clock().now().to_msg()
            s=State();s.header.stamp=stamp;s.connected=True;s.armed=value['armed'];s.mode=value['mode'];pubs['state'].publish(s)
            e=ExtendedState();e.header.stamp=stamp;e.landed_state=value['landed'];pubs['landed'].publish(e)
            p=PoseStamped();p.header.stamp=stamp;p.header.frame_id='map';p.pose.position.x=2.;p.pose.position.y=3.;p.pose.position.z=value['z'];p.pose.orientation.w=1.;pubs['pose'].publish(p)
            v=TwistStamped();v.header=p.header
            if value.get('integrate') and cmd:v.twist.linear.z=cmd.velocity.z
            pubs['vel'].publish(v)
        rclpy.spin_once(node,timeout_sec=.005)
        if until and until():return True
    return bool(until and until())
def latest():return rows[-1][1] if rows else {}
def enable(flag):
    request=SetBool.Request();request.data=flag;future=client.call_async(request)
    check('enable service responds',pump(3.,lambda:future.done()))
    return future.result()
def start(name,trigger,auto,servers=True,extra=None):
    global case,proc,log
    case=name;rows.clear();schema.clear();commands.clear();calls.clear();arms.clear()
    value.clear();value.update(mode='POSCTL',armed=not auto,landed=1 if auto else 2,z=0. if auto else 1.5,integrate=False)
    if servers:services.extend([node.create_service(SetMode,'/mavros/set_mode',mode),node.create_service(CommandBool,'/mavros/cmd/arming',arm)])
    cmd=['ros2','run','land_test','Safina_land_test','--ros-args','--params-file',os.environ['LAND_CONFIG']]
    params={'start_trigger':trigger,'auto_takeoff':auto,'command_gimbal_down':False,'gimbal_down_on_startup':False,'diagnostic_directory':str(out/name),'diagnostic_hz':20.,'takeOff_altitude':.25,'hover_seconds':.2,'initial_target_timeout':.7}
    params.update(extra or {})
    for k,v in params.items():cmd+=['-p',k+':='+str(v).lower() if isinstance(v,bool) else k+':='+str(v)]
    log=(out/(name+'.log')).open('w');proc=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    check('native node starts with fresh diagnostic schema',pump(5.,lambda:bool(rows) and latest().get('mode')=='POSCTL'))
    pump(.3)
def stop():
    global proc
    if proc:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=6)
        except subprocess.TimeoutExpired:raise RuntimeError('controller failed to stop')
        check('native controller normal exit',proc.returncode==0)
        proc=None;log.close()
    for s in services:node.destroy_service(s)
    services.clear();pump(.3)
proc=None
try:
    start('service_airborne','service',False)
    check('service workflow produces no standby commands',not commands and not calls and not arms)
    value.update(armed=False,landed=1);pump(.3)
    response=enable(True);check('airborne-only task refuses disarmed ground activation',not response.success)
    value.update(armed=True,landed=2);pump(.3)
    response=enable(True);check('airborne service accepts fresh inputs',response.success)
    check('service takeover reaches ACQUIRE after rejected mode request',pump(5.,lambda:latest().get('phase')=='ACQUIRE'))
    check('airborne service never arms',not arms)
    check('mode rejection retries then accepts OFFBOARD',len(calls)==2 and all(m=='OFFBOARD' for _,m in calls))
    response=enable(False);check('explicit disable accepted',response.success)
    count=len(calls);armcount=len(arms);pump(.5)
    check('disabled task stops commands and never reclaims modes',latest().get('enabled')=='0' and not [m for t,m in commands if t>time.monotonic()-.25] and len(calls)==count and len(arms)==armcount)
    stop()
    start('service_auto_takeoff','service',True)
    check('auto takeoff remains idle before enable',not commands and not calls and not arms)
    value['integrate']=True;check('auto takeoff enable accepts ground state',enable(True).success)
    check('service auto takeoff reaches TAKEOFF',pump(7.,lambda:latest().get('phase')=='TAKEOFF'))
    check('arming rejection retries only after OFFBOARD',len(arms)==2 and all(v for _,v in arms) and value['mode']=='OFFBOARD' and calls[-1][0]<arms[0][0])
    check('simulated ascent reaches acquisition then no-target AUTO.LAND',pump(9.,lambda:value['mode']=='AUTO.LAND'))
    check('positive ENU ascent is limited',any(m.velocity.z>.01 for _,m in commands) and all(m.coordinate_frame==m.FRAME_LOCAL_NED and math.isfinite(m.velocity.z) and abs(m.velocity.z)<=.281 for _,m in commands))
    check('no-target landing handoff rejected then retried',sum(m=='AUTO.LAND' for _,m in calls)==2)
    value.update(armed=False,landed=1);check('auto land completes on ground and disarm',pump(1.,lambda:latest().get('phase')=='COMPLETE'))
    stop()
    start('px4_mode_auto_takeoff','px4_mode',True)
    pump(1.7);check('ground POSCTL only streams zero standby with no mode or arm requests',bool(commands) and not calls and not arms and all(abs(m.velocity.z)<1e-8 for _,m in commands))
    check('PX4 mode workflow refuses enable service',not enable(True).success)
    value['mode']='OFFBOARD';check('manual OFFBOARD edge triggers ground auto takeoff and arm retry',pump(5.,lambda:latest().get('phase')=='TAKEOFF') and len(arms)==2 and not calls)
    value['mode']='POSCTL';n=len(arms);pump(.6)
    check('pilot POSCTL cancels without reclaim or rearm',latest().get('enabled')=='0' and len(arms)==n and not calls)
    stop()
    start('missing_mode_server','service',False,servers=False,extra={'takeover_timeout':2.3})
    check('enable works before mode server appears',enable(True).success)
    check('missing mode server reaches bounded cancellation',pump(4.,lambda:latest().get('phase')=='CANCELLED'))
    check('no arming or hidden mode writes on unavailable server',not calls and not arms)
    response=enable(False);check('cancellation remains responsive',response.success)
    stop()
    result={'passed':True,'checks':checks}
finally:
    if proc:stop()
    result={'passed':bool(checks) and all(c['passed'] for c in checks),'checks':checks}
    (out/'result.json').write_text(json.dumps(result,indent=2));node.destroy_node();rclpy.try_shutdown()
