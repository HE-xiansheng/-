#!/usr/bin/env python3
"""Actual EGO planner/adapter/MAVROS, synthetic sensors, disarmed isolated PX4."""
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState, PositionTarget, Mavlink
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from traj_utils.msg import PolyTraj


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='87' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    base=Path(sys.argv[1]); root=Path('/home/orangepi/drone_platform/01_apps/ego_planner')
    rclpy.init();node=rclpy.create_node('ego_wire_fixture')
    checks={};commands=[];actual_states=[];statuses=[];plans=[];trajectories=[];processes=[];logs=[];sink=[]
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(State,'/mavros/state',actual_states.append,qos_profile_sensor_data)
    node.create_subscription(String,'/ego_planner/px4_execution/status',lambda m:statuses.append(json.loads(m.data)),retained)
    node.create_subscription(String,'/ego_planner/manual_preview/status',lambda m:plans.append(json.loads(m.data)),retained)
    node.create_subscription(PolyTraj,'/ego_planner/flight_execution/trajectory',trajectories.append,100)
    def record(m):
        commands.append(dict(received_monotonic=time.monotonic(),stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
            frame=m.header.frame_id,coordinate_frame=m.coordinate_frame,type_mask=m.type_mask,
            position=[m.position.x,m.position.y,m.position.z],velocity=[m.velocity.x,m.velocity.y,m.velocity.z],
            acceleration=[m.acceleration_or_force.x,m.acceleration_or_force.y,m.acceleration_or_force.z],yaw=m.yaw,yaw_rate=m.yaw_rate))
    node.create_subscription(PositionTarget,'/mavros/setpoint_raw/local',record,500)
    if (base/'trace_adapter_publish.py').exists():
        def record_sink(m):
            if m.msgid==84:
                payload=b''.join(struct.pack('<Q',int(v)) for v in m.payload64)
                sink.append(dict(time_boot_ms=struct.unpack_from('<I',payload)[0],
                    received_monotonic=time.monotonic(),seq=m.seq,sysid=m.sysid,compid=m.compid))
        node.create_subscription(Mavlink,'/uas1/mavlink_sink',record_sink,
            QoSProfile(depth=500,reliability=ReliabilityPolicy.BEST_EFFORT))
    pubs={key:node.create_publisher(cls,topic,10) for key,cls,topic in [
        ('state',State,'/fixture/state'),('landed',ExtendedState,'/fixture/landed'),('pose',PoseStamped,'/fixture/pose'),
        ('lio',Odometry,'/factory_fastlio/odometry'),('cloud',PointCloud2,'/factory_fastlio/cloud_world'),
        ('goal',PoseStamped,'/ego_planner/navigation/preview_goal')]}
    cloud=PointCloud2();cloud.header.frame_id='camera_init';cloud.height=1
    cloud.fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
    points=[(1.05,y*.1,z*.1) for y in range(-5,6) for z in range(-5,6)]
    cloud.width=len(points);cloud.point_step=12;cloud.row_step=len(points)*12;cloud.is_dense=True
    cloud.data=b''.join(struct.pack('<fff',*p) for p in points)
    value=dict(mode='POSCTL',cloud=True,lio=True)
    last=0.
    def pump(seconds,predicate=None):
        nonlocal last
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            assert all(p.poll() is None for p in processes),'owned child exited early'
            assert all(not s.armed for s in actual_states),'actual PX4 armed'
            now=time.monotonic()
            if now-last>.025:
                last=now;stamp=node.get_clock().now().to_msg()
                m=State();m.header.stamp=stamp;m.connected=True;m.armed=True;m.mode=value['mode'];pubs['state'].publish(m)
                m=ExtendedState();m.header.stamp=stamp;m.landed_state=2;pubs['landed'].publish(m)
                m=PoseStamped();m.header.stamp=stamp;m.header.frame_id='map';m.pose.position.x=10.;m.pose.position.y=20.;m.pose.position.z=3.;m.pose.orientation.z=m.pose.orientation.w=math.sqrt(.5);pubs['pose'].publish(m)
                if value['lio']:
                    m=Odometry();m.header.stamp=stamp;m.header.frame_id='camera_init';m.child_frame_id='body';m.pose.pose.position.x=-2.;m.pose.pose.orientation.w=1.;pubs['lio'].publish(m)
                if value['cloud']:cloud.header.stamp=stamp;pubs['cloud'].publish(cloud)
            rclpy.spin_once(node,timeout_sec=.002)
            if predicate and predicate():return True
        return bool(predicate and predicate())
    def check(label,ok):
        checks[label]=bool(ok);print(('PASS ' if ok else 'FAIL ')+label,flush=True);assert ok,label
    def start(name,args):
        log=(base/(name+'.log')).open('w');logs.append(log)
        p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
    def stop(name,p):
        if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=12)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        processes.remove(p);checks[name+'_normal_exit']=p.returncode==0
    def phase(name):return pump(10,lambda:statuses and statuses[-1]['phase']==name)
    def quiet():
        pump(.25);n=len(commands);pump(.6);return len(commands)==n
    try:
        baseline=subprocess.run([sys.executable,str(base/'adapter-before.py'),'--check','--run',str(base/'baseline'),
            '--ros-args','-r','/mavros/state:=/fixture/state'],capture_output=True,text=True)
        (base/'baseline.log').write_text(baseline.stdout+baseline.stderr)
        check('old_adapter_reproduces_ros_remap_argparse_failure',baseline.returncode==2 and 'unrecognized arguments' in baseline.stderr)
        start('mavros',['ros2','launch',str(base/'mavros_platform.launch.py'),'fcu_url:=udp://127.0.0.1:14640@127.0.0.1:14641'])
        check('real_px4_connected_disarmed',pump(25,lambda:actual_states and actual_states[-1].connected and not actual_states[-1].armed))
        if (base/'native-build/observer').exists():
            start('native-observer',[str(base/'native-build/observer')])
            check('native_qos_observer_ready',pump(5,lambda:'"kind":"ready"' in (base/'native-observer.log').read_text()))
        planner=start('planner',[str(root/'workspace/install/plan_manage/lib/plan_manage/planner_node'),'--ros-args',
            '--params-file',str(root/'config/planner.yaml'),'--params-file',str(root/'config/manual_preview.yaml'),
            '-r','odom_world:=/factory_fastlio/odometry','-r','grid_map/odom:=/factory_fastlio/odometry','-r','grid_map/cloud:=/factory_fastlio/cloud_world'])
        check('actual_planner_ready',pump(10,lambda:plans and plans[-1]['phase']=='WAIT_GOAL'))
        entry=base/('trace_adapter_publish.py' if (base/'trace_adapter_publish.py').exists() else 'adapter.py')
        adapter=start('adapter',[sys.executable,str(entry),'--publish','--isolated-test','--run',str(base/'execution'),
            '--ros-args','-r','/mavros/state:=/fixture/state','-r','/mavros/extended_state:=/fixture/landed',
            '-r','/mavros/local_position/pose:=/fixture/pose'])
        check('standard_ros_remaps_allow_ready',phase('READY'))
        check('prestream_observed',pump(4,lambda:len(commands)>25))
        check('prestream_measured_fc_position_yaw',all(np.allclose(c['position'],[10,20,3]) and abs(c['yaw']-math.pi/2)<1e-5 for c in commands))
        m=PoseStamped();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='camera_init';m.pose.position.x=-.5;m.pose.orientation.w=1.;pubs['goal'].publish(m)
        check('map_goal_accepted',pump(6,lambda:plans and plans[-1].get('has_goal')))
        value['mode']='OFFBOARD'
        check('actual_native_trajectory_tracks',phase('TRACK') and bool(trajectories));pump(2.)
        check('nonzero_velocity_and_acceleration',any(np.linalg.norm(c['velocity'])>.01 for c in commands) and any(np.linalg.norm(c['acceleration'])>.01 for c in commands))
        check('single_enu_frame_and_complete_pva_mask',all(c['frame']=='map' and c['coordinate_frame']==1 and c['type_mask']==PositionTarget.IGNORE_YAW_RATE for c in commands))
        value['lio']=False
        check('stale_lio_stops_output',pump(4,lambda:statuses and 'lio:stale' in statuses[-1]['hard_errors']) and quiet())
        value['lio']=True;check('lio_recovers_native_track',phase('TRACK'))
        value['cloud']=False
        check('stale_cloud_stops_output',pump(4,lambda:statuses and 'cloud:stale' in statuses[-1]['hard_errors']) and quiet())
        value['cloud']=True;check('cloud_recovers_native_track',phase('TRACK'))
        planner.send_signal(signal.SIGSTOP)
        check('frozen_planner_stops_output',pump(4,lambda:statuses and 'status:stale' in statuses[-1]['hard_errors']) and quiet())
        planner.send_signal(signal.SIGCONT);check('planner_recovers_track',phase('TRACK'));pump(.5)
        check('adapter_has_no_flight_service_clients',not node.get_client_names_and_types_by_node('ego_planner_px4_execution','/'))
        (base/'execution/stop-request').touch()
        check('operator_stop_holds',phase('STOP_HOLD'));n=len(commands);pump(.5)
        check('stop_hold_measured_pose_zero_va',len(commands)>n and all(np.allclose(c['position'],[10,20,3]) and np.allclose(c['velocity'],0) and np.allclose(c['acceleration'],0) for c in commands[n:]))
        value['mode']='POSCTL'
        # The adapter exits itself on handover; remove it from the live-child guard first.
        processes.remove(adapter)
        check('manual_handover_completes_exit',pump(4,lambda:adapter.poll() is not None) and adapter.returncode==0)
        check('no_output_after_exit',quiet())
        check('all_actual_px4_disarmed_nonoffboard',bool(actual_states) and all(not s.armed and s.mode!='OFFBOARD' for s in actual_states))
    finally:
        for p in list(reversed(processes)):
            try:os.killpg(p.pid,signal.SIGCONT)
            except ProcessLookupError:pass
            stop('process_'+str(p.pid),p)
        for log in logs:log.close()
        (base/'result.json').write_text(json.dumps(dict(checks=checks,commands=commands,statuses=statuses,
            mavros_sink_commands=sink,trajectory_count=len(trajectories),actual_states=[dict(armed=s.armed,mode=s.mode,connected=s.connected) for s in actual_states]),indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    return 0 if checks and all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
