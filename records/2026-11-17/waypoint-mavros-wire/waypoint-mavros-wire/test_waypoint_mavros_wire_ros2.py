#!/usr/bin/env python3
"""Synthetic waypoint inputs, real MAVROS/PX4 wire; never arm or change PX4 mode."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, RCIn, ExtendedState, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import yaml


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '78'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    base = Path(sys.argv[1])
    rclpy.init()
    node = rclpy.create_node('waypoint_wire_fixture')
    states, commands, statuses, calls, checks = [], [], [], [], {}
    processes, logs = [], []
    value = dict(pwm=1100, mode='POSCTL', pose_age=0.)
    node.create_subscription(State, '/mavros/state', states.append, qos_profile_sensor_data)
    def record_command(m):
        commands.append(dict(received_monotonic=time.monotonic(),
            stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
            frame=m.header.frame_id, coordinate_frame=m.coordinate_frame, type_mask=m.type_mask,
            position=[m.position.x,m.position.y,m.position.z],
            velocity=[m.velocity.x,m.velocity.y,m.velocity.z],
            acceleration=[m.acceleration_or_force.x,m.acceleration_or_force.y,m.acceleration_or_force.z],
            yaw=m.yaw,yaw_rate=m.yaw_rate))
    node.create_subscription(PositionTarget, '/mavros/setpoint_raw/local', record_command, 300)
    node.create_subscription(String, '/waypoint_mission/status', lambda m: statuses.append(m.data), 100)
    def mode(req, res):
        calls.append(dict(kind='mode_stub_only', mode=req.custom_mode))
        value['mode'] = req.custom_mode
        res.mode_sent = True
        return res
    def arm(req, res):
        calls.append(dict(kind='unexpected_arm_stub_only', value=req.value))
        res.success = False
        return res
    node.create_service(SetMode, '/fixture/set_mode', mode)
    node.create_service(CommandBool, '/fixture/arming', arm)
    pubs={key:node.create_publisher(cls, '/fixture/'+key,10) for key,cls in
        [('state',State),('rc',RCIn),('extended',ExtendedState),('pose',PoseStamped),('odom',Odometry)]}
    def pump(seconds, predicate=None):
        end=time.monotonic()+seconds
        last=0.
        while time.monotonic()<end:
            assert all(p.poll() is None for p in processes), 'owned child exited early'
            assert not any(s.armed for s in states), 'real PX4 unexpectedly armed'
            if time.monotonic()-last>=.025:
                last=time.monotonic(); stamp=node.get_clock().now().to_msg()
                m=State();m.header.stamp=stamp;m.connected=True;m.armed=True;m.mode=value['mode'];pubs['state'].publish(m)
                m=RCIn();m.header.stamp=stamp;m.rssi=100;m.channels=[1500]*8;m.channels[4]=value['pwm'];pubs['rc'].publish(m)
                m=ExtendedState();m.header.stamp=stamp;m.landed_state=2;pubs['extended'].publish(m)
                m=PoseStamped();m.header.stamp=rclpy.time.Time(nanoseconds=node.get_clock().now().nanoseconds-int(value['pose_age']*1e9)).to_msg();m.header.frame_id='map';m.pose.position.z=1.;m.pose.orientation.w=1.;pubs['pose'].publish(m)
                m=Odometry();m.header.stamp=stamp;m.header.frame_id='camera_init';m.child_frame_id='body';m.pose.pose.orientation.w=1.;pubs['odom'].publish(m)
            rclpy.spin_once(node,timeout_sec=.003)
            if predicate and predicate():return True
        return bool(predicate and predicate())
    def check(name, ok):
        checks[name]=bool(ok)
        print(('PASS ' if ok else 'FAIL ')+name,flush=True)
        assert ok,name
    def start(name,args):
        log=(base/(name+'.log')).open('w');logs.append(log)
        p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p)
        return p
    def stop(p):
        if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=12)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        processes.remove(p)
        check('normal_exit_'+str(p.pid),p.returncode==0)
    try:
        start('mavros',['ros2','launch',str(base/'mavros_platform.launch.py'),
            'fcu_url:=udp://127.0.0.1:14640@127.0.0.1:14641'])
        check('real_px4_connected_disarmed',pump(25,lambda:states and states[-1].connected and not states[-1].armed))
        binary=Path(get_package_prefix('waypoint_mission'))/'lib/waypoint_mission/waypoint_mission'
        remaps={'/mavros/state':'/fixture/state','/mavros/rc/in':'/fixture/rc',
            '/mavros/extended_state':'/fixture/extended','/mavros/local_position/pose':'/fixture/pose',
            '/Odometry':'/fixture/odom','/mavros/set_mode':'/fixture/set_mode','/mavros/cmd/arming':'/fixture/arming'}
        for scenario in ['rc_cancel','stale_pose']:
            value.update(pwm=1100,mode='POSCTL',pose_age=0.);statuses.clear()
            cfg={'mission':dict(name=scenario,frame='x_right_y_forward',manual_airborne=True,
                state_timeout=.8,return_to_start=False,auto_land=False,
                rc=dict(channel=5,offboard_slot=6,min=1075,trim=1494,max=1913,off_hold_seconds=.3,on_hold_seconds=.2),
                waypoints=[dict(name='right',x=.8,y=0.,z=1.,yaw='hold',hold=.1)])}
            path=base/(scenario+'.yaml');path.write_text(yaml.safe_dump(cfg))
            args=[str(binary),'--ros-args','-p','mission_file:='+str(path),'-p','photo_dir:='+str(base/'photos'),'-p','log_dir:='+str(base/'csv')]
            for a,b in remaps.items():args+=['-r',a+':='+b]
            task=start(scenario,args)
            check(scenario+'_gate_ready',pump(7,lambda:any('gate_ready=1' in s for s in statuses)))
            begin=len(commands);pump(.4)
            check(scenario+'_idle_has_no_output',len(commands)==begin)
            value['pwm']=1900
            check(scenario+'_rightward_velocity',pump(10,lambda:any(c['velocity'][1]<-.1 for c in commands[begin:])))
            pump(2.)
            active=commands[begin:]
            check(scenario+'_bounded_rightward_ENU',any(c['velocity'][1]<-.1 for c in active) and all(abs(c['velocity'][0])<.001 and -.601<=c['velocity'][1]<=.001 for c in active))
            if scenario=='rc_cancel':value['pwm']=1100
            else:value['pose_age']=3.
            check(scenario+'_aborts',pump(4,lambda:any('phase=ABORT ' in s for s in statuses)))
            pump(.6);settled=len(commands);pump(1.2)
            check(scenario+'_stops_setpoints',len(commands)==settled)
            stop(task)
        check('no_arm_request_even_to_stub',all(c['kind']=='mode_stub_only' for c in calls))
        check('offboard_requests_only_to_stub',len(calls)==2 and all(c['mode']=='OFFBOARD' for c in calls))
        check('all_actual_px4_states_disarmed',bool(states) and all(not s.armed for s in states))
        check('real_px4_mode_never_offboard',all(s.mode!='OFFBOARD' for s in states))
        check('command_geometry_finite',bool(commands) and all(math.isfinite(v) for c in commands for v in c['position']+c['velocity']+c['acceleration']+[c['yaw'],c['yaw_rate']]))
    finally:
        for p in list(reversed(processes)):stop(p)
        for log in logs:log.close()
        (base/'result.json').write_text(json.dumps(dict(checks=checks,commands=commands,calls=calls,
            actual_states=[dict(armed=s.armed,mode=s.mode,connected=s.connected) for s in states]),indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    return 0 if checks and all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
