#!/usr/bin/env python3
"""Native image detector -> landing controller -> real MAVROS/disarmed isolated PX4; synthetic inputs."""
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

assert os.environ.get('ROS_DOMAIN_ID') == '83' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String

out=Path(sys.argv[1]);out.mkdir(exist_ok=True)
base=Path('/home/orangepi/drone_platform/01_apps/mission_demo/workspace/install')
rclpy.init();node=rclpy.create_node('aruco_landing_image_fixture')
checks=[];rows=[];schemas=[];markers=[];accepted=[];commands=[];calls=[];arms=[];stamps=set();procs=[];logs=[];completed=False
actual_states=[]
node.create_subscription(State,'/mavros/state',actual_states.append,qos_profile_sensor_data)
retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String,'/aruco_landing/status_header',lambda m:schemas.append(m.data),retained)
def status(m):
    if schemas:rows.append((time.monotonic(),dict(zip(next(csv.reader([schemas[-1]])),next(csv.reader([m.data]))))))
node.create_subscription(String,'/aruco_landing/status',status,retained)
node.create_subscription(PoseStamped,'/aruco_poseresult',lambda m:markers.append((time.monotonic(),m)),30)
node.create_subscription(PoseStamped,'/aruco_landing/accepted_target',lambda m:accepted.append((time.monotonic(),m)),30)
node.create_subscription(PositionTarget,'/mavros/setpoint_raw/local',lambda m:commands.append((time.monotonic(),m)),100)
def mode(req,res):calls.append(req.custom_mode);res.mode_sent=False;return res
def arm(req,res):arms.append(req.value);res.success=False;return res
node.create_service(SetMode,'/fixture/set_mode',mode);node.create_service(CommandBool,'/fixture/arming',arm)
pubs={k:node.create_publisher(t,topic,2) for k,t,topic in [
    ('image',Image,'/rtsp_cam/image_raw'),('state',State,'/fixture/state'),
    ('pose',PoseStamped,'/fixture/pose'),('vel',TwistStamped,'/fixture/velocity'),
    ('landed',ExtendedState,'/fixture/landed')]}
v={'yaw':0.0,'mode':'POSCTL','age':0.0,'frame':'gimbal_camera_optical_frame','image':None}
K=np.array([[2042.0851788597752,0,959.0675902004489],[0,2042.8993693539358,467.50991502443503],[0,0,1.]])
D=np.array([-.4849208954761301,.3575460777522052,.0024881576093297698,-.0002765905391136276,0.])
def image(marker_id=0,blank=False,small=False):
    gray=np.full((1080,1920),255,np.uint8)
    if not blank:
        code=cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL),marker_id,400)
        size=.035
        corners=np.float32([[-size/2,size/2,0],[size/2,size/2,0],[size/2,-size/2,0],[-size/2,-size/2,0]])
        pixels,_=cv2.projectPoints(corners,np.array([math.pi,0.,0.]),np.array([.12,-.16,1.5]),K,D)
        matrix=cv2.getPerspectiveTransform(np.float32([[0,0],[399,0],[399,399],[0,399]]),pixels.reshape(4,2).astype(np.float32))
        gray=cv2.warpPerspective(code,matrix,(1920,1080),borderValue=255)
    if small:gray=cv2.resize(gray,(960,540))
    m=Image();m.height,m.width=gray.shape;m.encoding='mono8';m.step=m.width;m.data=gray.tobytes()
    return m
normal=image();blank=image(blank=True);wrong_id=image(marker_id=1);wrong_size=image(small=True);v['image']=normal

def phase(name,seconds=1.6,**changes):
    v.update(changes);end=time.monotonic()+seconds;next_state=next_image=0
    while time.monotonic()<end:
        assert not any(m.armed for m in actual_states), 'actual PX4 unexpectedly armed'
        now=time.monotonic()
        if now>=next_state:
            stamp=node.get_clock().now().to_msg()
            s=State();s.header.stamp=stamp;s.connected=True;s.armed=True;s.mode=v['mode'];pubs['state'].publish(s)
            p=PoseStamped();p.header.stamp=stamp;p.header.frame_id='map';p.pose.position.x=2.;p.pose.position.y=3.;p.pose.position.z=1.5
            p.pose.orientation.z=math.sin(v['yaw']/2);p.pose.orientation.w=math.cos(v['yaw']/2);pubs['pose'].publish(p)
            velocity=TwistStamped();velocity.header=p.header;pubs['vel'].publish(velocity)
            e=ExtendedState();e.header.stamp=stamp;e.landed_state=ExtendedState.LANDED_STATE_IN_AIR;pubs['landed'].publish(e)
            next_state=now+.025
        if now>=next_image and v['image'] is not None:
            m=v['image'];m.header.frame_id=v['frame']
            stamp=node.get_clock().now().nanoseconds-int(v['age']*1e9)
            m.header.stamp.sec=stamp//1000000000;m.header.stamp.nanosec=stamp%1000000000
            stamps.add(stamp);pubs['image'].publish(m);next_image=now+.12
        rclpy.spin_once(node,timeout_sec=.005)
        for proc in procs:
            assert proc.poll() is None,'native process exited before test completion'
    cut=time.monotonic()-.5
    diag=[r for t,r in rows if t>cut]
    cmd=[m for t,m in commands if t>cut]
    targets=[m for t,m in accepted if t>cut]
    print('PHASE',name,'rows',len(diag),'phase',diag[-1].get('phase') if diag else None,'reason',diag[-1].get('marker_reason') if diag else None,flush=True)
    return diag,cmd,targets

def check(label,value):
    checks.append({'check':label,'passed':bool(value)});print(json.dumps(checks[-1]),flush=True)
    assert value,label

def start(package,executable,config,extra):
    log=(out/(package+'.log')).open('wb');logs.append(log)
    args=[str(base/package/'lib'/package/executable),'--ros-args','--params-file',str(base/package/'share'/package/'config'/config)]
    for item in extra:args+=['-p',item]
    if package=='land_test':
        for source,dest in {'/mavros/state': '/fixture/state', '/mavros/local_position/pose': '/fixture/pose', '/mavros/local_position/velocity_local': '/fixture/velocity', '/mavros/extended_state': '/fixture/landed', '/mavros/set_mode': '/fixture/set_mode', '/mavros/cmd/arming': '/fixture/arming'}.items(): args+=['-r',source+':='+dest]
    p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p)

try:
    log=(out/'mavros.log').open('wb');logs.append(log)
    procs.append(subprocess.Popen(['ros2','launch',str(out/'mavros_platform.launch.py'),'fcu_url:=udp://127.0.0.1:14640@127.0.0.1:14641'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    deadline=time.monotonic()+25
    while time.monotonic()<deadline and not (actual_states and actual_states[-1].connected):
        assert procs[0].poll() is None
        rclpy.spin_once(node,timeout_sec=.05)
    check('actual PX4 connected disarmed before controller starts',bool(actual_states) and actual_states[-1].connected and not actual_states[-1].armed)
    start('aruco_test','Safina_aruco_test','aruco.yaml',['preview_color:=false'])
    start('land_test','Safina_land_test','takeoff_land.yaml',['command_gimbal_down:=false','gimbal_down_on_startup:=false','diagnostic_directory:='+str(out/'controller-data')])
    d,c,t=phase('ready image chain',3.5)
    check('actual image detector and controller diagnostics are ready',bool(d) and any(r.get('target_valid')=='1' for r in d))
    recent=[m for at,m in markers if at>time.monotonic()-.5 and m.pose.position.z>0]
    check('PnP recovers synthetic optical position',bool(recent) and all(abs(m.pose.position.x-.12)<.03 and abs(m.pose.position.y+.16)<.03 and abs(m.pose.position.z-1.5)<.15 for m in recent))
    check('detector preserves actual image source header',bool(recent) and all(m.header.frame_id==v['frame'] and m.header.stamp.sec*1000000000+m.header.stamp.nanosec in stamps for m in recent))
    check('optical target maps to ENU at zero aircraft yaw',bool(t) and all(abs(m.pose.position.x-2.16)<.04 and abs(m.pose.position.y-2.88)<.04 for m in t))
    check('marker is paired with pose history within configured tolerance',all(0<=float(r['marker_pair_dt'])<=.151 for r in d if r.get('target_valid')=='1'))
    check('healthy POSCTL standby produces only zero commands',bool(c) and all(abs(m.velocity.x)+abs(m.velocity.y)+abs(m.velocity.z)+abs(m.yaw_rate)<1e-8 for m in c))
    check('no flight service request before mode trigger',not calls and not arms)
    d,c,t=phase('wrong optical frame',frame='wrong')
    check('image source wrong frame propagates and controller rejects it',bool(d) and all(r['marker_reason']=='unexpected_marker_frame' and r['target_valid']=='0' for r in d))
    d,c,t=phase('stale image',frame='gimbal_camera_optical_frame',age=3.)
    check('old image source cannot become a fresh control target',bool(d) and all(r['marker_reason']=='source_stamp_stale' and r['target_valid']=='0' for r in d))
    d,c,t=phase('future image',age=-3.)
    check('future image source cannot become a control target',bool(d) and all(r['marker_reason']=='source_stamp_stale' and r['target_valid']=='0' for r in d))
    d,c,t=phase('blank image',image=blank,age=0.)
    check('blank image detector sentinel clears target',bool(d) and all(r['marker_reason']=='detector_no_selected_target' and r['target_valid']=='0' for r in d))
    d,c,t=phase('wrong image size',image=wrong_size)
    check('calibration resolution mismatch cannot drive the controller',bool(d) and all(r['target_valid']=='0' for r in d))
    d,c,t=phase('unconfigured marker ID',image=wrong_id)
    check('unconfigured ArUco ID cannot drive the controller',bool(d) and all(r['target_valid']=='0' for r in d))
    d,c,t=phase('90 degree aircraft yaw',image=normal,yaw=math.pi/2,seconds=2.)
    check('image target rotates into ENU using aircraft pose yaw',bool(t) and all(abs(m.pose.position.x-2.12)<.04 and abs(m.pose.position.y-3.16)<.04 for m in t))
    phase('filter reset',image=blank,yaw=0.)
    phase('restore source',image=normal,seconds=2.)
    d,c,t=phase('simulated pilot OFFBOARD edge',mode='OFFBOARD',seconds=2.5)
    check('fresh image chain enables task on simulated pilot mode edge',bool(d) and all(r['enabled']=='1' for r in d))
    check('controller moves toward the detected ENU target',bool(c) and any(m.velocity.x>.005 and m.velocity.y<-.005 for m in c))
    d,c,t=phase('image stream dropout',image=None,seconds=1.5)
    check('image dropout holds rather than continuing blind descent at 1.5m',bool(d) and d[-1]['phase'] in ('LOST_HOLD','ACQUIRE') and bool(c) and all(abs(m.velocity.z)<.03 for m in c))
    d,c,t=phase('image stream recovered',image=normal,seconds=2.)
    check('fresh images reacquire the control target',bool(d) and all(r['target_valid']=='1' for r in d) and d[-1]['phase'] in ('TRANSLATE','STABILIZE','DESCEND'))
    d,c,t=phase('pilot returns POSCTL',mode='POSCTL',seconds=.8)
    check('pilot mode exit cancels without reclaiming control',bool(d) and all(r['enabled']=='0' for r in d) and not calls and not arms)
    check('all generated commands obey finite ENU limits',bool(commands) and all(m.coordinate_frame==PositionTarget.FRAME_LOCAL_NED and all(math.isfinite(x) for x in [m.velocity.x,m.velocity.y,m.velocity.z,m.yaw_rate]) and math.hypot(m.velocity.x,m.velocity.y)<=.151 and abs(m.velocity.z)<=.281 and abs(m.yaw_rate)<=.101 for _,m in commands))
    check('actual PX4 stays disarmed and nonoffboard',bool(actual_states) and all(not m.armed and m.mode!='OFFBOARD' for m in actual_states))
    completed=True
finally:
    for p in procs:
        if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
    for p in procs:
        try:p.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3)
    for log in logs:log.close()
    checks.append({'check':'both native nodes exit normally','passed':all(p.returncode==0 for p in procs)})
    (out/'result.json').write_text(json.dumps({'passed':completed and all(x['passed'] for x in checks),'checks':{x['check']:x['passed'] for x in checks},'commands':[dict(stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,frame=m.header.frame_id,coordinate_frame=m.coordinate_frame,type_mask=m.type_mask,position=[m.position.x,m.position.y,m.position.z],velocity=[m.velocity.x,m.velocity.y,m.velocity.z],acceleration=[m.acceleration_or_force.x,m.acceleration_or_force.y,m.acceleration_or_force.z],yaw=m.yaw,yaw_rate=m.yaw_rate) for _,m in commands],'actual_states':[dict(armed=m.armed,mode=m.mode,connected=m.connected) for m in actual_states],'returncodes':[p.returncode for p in procs]},indent=2))
    (out/'diagnostics.json').write_text(json.dumps([r for _,r in rows],indent=2))
    node.destroy_node();rclpy.shutdown()
