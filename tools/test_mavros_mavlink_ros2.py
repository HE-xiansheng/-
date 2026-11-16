#!/usr/bin/env python3
"""Actual MAVROS2 with loopback MAVLink peer. Never opens a serial device."""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

from pymavlink.dialects.v20 import common as mav
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, RCIn, PositionTarget, ExtendedState
from mavros_msgs.srv import SetMode

base = Path(sys.argv[1])
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
assert os.environ.get('ROS_DOMAIN_ID') == '85'
checks = []
received = []
states = []; poses = []; rcs = []; landed = []
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 14641)); sock.setblocking(False)
class Writer:
    def write(self, data):
        return sock.sendto(data, ('127.0.0.1', 14640))
peer = mav.MAVLink(Writer(), srcSystem=1, srcComponent=1)
parser = mav.MAVLink(None)
rclpy.init()
node = rclpy.create_node('mavros_wire_fixture')
node.create_subscription(State, '/mavros/state', states.append, qos_profile_sensor_data)
node.create_subscription(PoseStamped, '/mavros/local_position/pose', poses.append, qos_profile_sensor_data)
node.create_subscription(RCIn, '/mavros/rc/in', rcs.append, qos_profile_sensor_data)
node.create_subscription(ExtendedState, '/mavros/extended_state', landed.append, qos_profile_sensor_data)
vision = node.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
setpoints = node.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
client = node.create_client(SetMode, '/mavros/set_mode')
next_input = 0
started = time.monotonic()
log = (base/'mavros.log').open('wb')
process = subprocess.Popen(['ros2','launch',str(base/'mavros_platform.launch.py'),
    'fcu_url:=udp://127.0.0.1:14640@127.0.0.1:14641'],
    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

def pump(seconds=0.1, extra=None):
    global next_input
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if process.poll() is not None:
            raise RuntimeError('MAVROS exited {}'.format(process.returncode))
        now = time.monotonic()
        if now >= next_input:
            boot = int((now-started)*1000)
            peer.heartbeat_send(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_PX4,
                mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 1 << 16, mav.MAV_STATE_STANDBY)
            peer.attitude_send(boot, 0.0, 0.0, .2, 0.0, 0.0, 0.0)
            peer.local_position_ned_send(boot, 10.0, 20.0, -3.0, 1.0, 2.0, -0.5)
            peer.extended_sys_state_send(mav.MAV_VTOL_STATE_UNDEFINED, mav.MAV_LANDED_STATE_ON_GROUND)
            peer.rc_channels_send(boot, 18, *[1000+i*10 for i in range(18)], 90)
            next_input = now + .1
        while True:
            try:
                data, address = sock.recvfrom(65536)
            except BlockingIOError:
                break
            assert address[0] == '127.0.0.1'
            messages = parser.parse_buffer(data) or []
            received.extend(messages)
            for msg in messages:
                if msg.get_type() == 'COMMAND_LONG':
                    peer.command_ack_send(msg.command, mav.MAV_RESULT_UNSUPPORTED)
        if extra:
            extra()
        rclpy.spin_once(node, timeout_sec=.015)

def wait(predicate, timeout=15, extra=None):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate(): return True
        pump(.05, extra)
    return bool(predicate())

def check(value, label):
    checks.append({'check':label,'passed':bool(value)})
    print(('PASS ' if value else 'FAIL ') + label, flush=True)
    assert value, label

def near(a,b): return abs(a-b)<1e-5

try:
    check(wait(lambda: states and states[-1].connected), 'wire heartbeat establishes real MAVROS connection')
    check(wait(lambda: states and not states[-1].armed and states[-1].mode == 'MANUAL'), 'disarmed PX4 heartbeat maps to ROS state')
    check(wait(lambda: bool(poses)), 'wire local position produces ROS pose')
    p = poses[-1].pose.position
    check(near(p.x,20) and near(p.y,10) and near(p.z,3), 'incoming NED position converts once to ENU')
    check(wait(lambda: rcs and len(rcs[-1].channels)==18), 'rc_io plugin publishes wire RC input')
    check(list(rcs[-1].channels)==[1000+i*10 for i in range(18)], 'all 18 RC channel values survive transport')
    check(wait(lambda: landed and landed[-1].landed_state==ExtendedState.LANDED_STATE_ON_GROUND), 'wire landed state reaches ROS extended state')
    check(wait(lambda: node.count_subscribers('/mavros/vision_pose/pose')>0), 'vision_pose plugin subscribes')
    pose = PoseStamped(); pose.header.frame_id='camera_init'
    pose.pose.position.x=1.0;pose.pose.position.y=2.0;pose.pose.position.z=3.0
    pose.pose.orientation.z=math.sin(.4/2);pose.pose.orientation.w=math.cos(.4/2)
    stamps=[]
    def send_vision():
        pose.header.stamp=node.get_clock().now().to_msg()
        stamps.append(pose.header.stamp.sec*1000000+pose.header.stamp.nanosec//1000)
        vision.publish(pose)
    check(wait(lambda:any(m.get_type()=='VISION_POSITION_ESTIMATE' for m in received),extra=send_vision), 'ROS external pose reaches actual MAVLink vision packet')
    v=[m for m in received if m.get_type()=='VISION_POSITION_ESTIMATE'][-1]
    check(near(v.x,2) and near(v.y,1) and near(v.z,-3), 'external vision ENU position converts once to NED')
    check(near(v.roll,0) and near(v.pitch,0) and near(v.yaw,math.pi/2-.4), 'external vision applies ENU/NED and FLU/FRD attitude convention')
    check(any(abs(v.usec-s)<=1 for s in stamps), 'external vision uses original ROS source stamp')
    check(wait(lambda:node.count_subscribers('/mavros/setpoint_raw/local')>0), 'setpoint_raw plugin subscribes')
    cmd=PositionTarget();cmd.coordinate_frame=PositionTarget.FRAME_LOCAL_NED;cmd.type_mask=0
    cmd.position.x=4.;cmd.position.y=5.;cmd.position.z=6.
    cmd.velocity.x=.2;cmd.velocity.y=.3;cmd.velocity.z=.4
    cmd.acceleration_or_force.x=.5;cmd.acceleration_or_force.y=.6;cmd.acceleration_or_force.z=.7
    cmd.yaw=.8;cmd.yaw_rate=.9
    def send_setpoint():
        cmd.header.stamp=node.get_clock().now().to_msg();setpoints.publish(cmd)
    check(wait(lambda:any(m.get_type()=='SET_POSITION_TARGET_LOCAL_NED' for m in received),extra=send_setpoint), 'ROS setpoint reaches actual MAVLink packet')
    s=[m for m in received if m.get_type()=='SET_POSITION_TARGET_LOCAL_NED'][-1]
    check(s.coordinate_frame==1 and s.type_mask==0 and s.target_system==1 and s.target_component==1, 'frame mask and target identity survive wire encoding')
    check(all(near(getattr(s,n),e) for n,e in [('x',5),('y',4),('z',-6),('vx',.3),('vy',.2),('vz',-.4),('afx',.6),('afy',.5),('afz',-.7)]), 'position velocity acceleration all convert ENU to NED once')
    check(near(s.yaw,math.pi/2-.8) and near(s.yaw_rate,-.9), 'setpoint yaw and yaw rate convert once')
    check(wait(client.service_is_ready), 'actual mode service is present')
    request=SetMode.Request();request.custom_mode='POSCTL'
    future=client.call_async(request)
    check(wait(future.done) and future.result().mode_sent, 'mode service reports MAVLink dispatch to simulated peer')
    check(wait(lambda:any(m.get_type()=='SET_MODE' and m.custom_mode==3<<16 for m in received)), 'POSCTL mode request encodes PX4 main mode on loopback only')
    check('/mavros/cmd/arming' in dict(node.get_service_names_and_types()), 'command plugin exposes arming API without calling it')
    check(not any(m.get_type()=='COMMAND_LONG' and m.command in [mav.MAV_CMD_COMPONENT_ARM_DISARM,mav.MAV_CMD_NAV_TAKEOFF] for m in received), 'no arm or takeoff command transmitted')
finally:
    try:
        os.killpg(process.pid,signal.SIGINT)
    except ProcessLookupError:
        pass
    try:process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
    log.close()
    (base/'wire-messages.json').write_text(json.dumps([m.to_dict() for m in received],indent=2))
    (base/'result.json').write_text(json.dumps({'passed':len(checks)==22 and all(x['passed'] for x in checks),'checks':checks,'process_returncode':process.returncode,'states':[{'connected':s.connected,'armed':s.armed,'mode':s.mode} for s in states]},indent=2))
    sock.close();node.destroy_node();rclpy.shutdown()
