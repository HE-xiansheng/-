#!/usr/bin/env python3
"""Recorded MID360/IMU -> actual FAST-LIO/relay -> MAVROS -> isolated PX4."""
import gzip
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from validate_fastlio_replay import decode


def main():
    base=Path(sys.argv[1])
    assert os.environ.get('ROS_DOMAIN_ID')=='82'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    platform=Path('/home/orangepi/drone_platform')
    env=dict(os.environ,LOCALIZATION_RUNTIME_DIR=str(base/'localization-runtime'),
        EGO_EXECUTION_RUNTIME_DIR=str(base/'execution'),EGO_PREVIEW_RUNTIME_DIR=str(base/'preview'))
    with gzip.open(base/'livox.jsonl.gz','rt') as source:
        messages=[decode(json.loads(line)) for line in source]
    imu=[m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for t,_,m in messages if t=='/livox/imu']
    minimum,maximum=min(imu),max(imu);ordered=[]
    for topic,_,m in messages:
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        event=stamp+(max((p.offset_time for p in m.points),default=0)*1e-9 if topic=='/livox/lidar' else 0)
        if topic=='/livox/imu' or minimum<=stamp<=event<=maximum:
            ordered.append((event-minimum,topic,m,int(round((stamp-minimum)*1e9))))
    ordered.sort(key=lambda row:row[0]);period=maximum-minimum+.12
    rclpy.init();node=rclpy.create_node('fastlio_px4_ground_fixture')
    states=[];odom=[];vision=[];checks={};calls=[];feed=True;counts={t:0 for t in ['/livox/lidar','/livox/imu']}
    pubs={'/livox/lidar':node.create_publisher(CustomMsg,'/livox/lidar',10),
          '/livox/imu':node.create_publisher(Imu,'/livox/imu',200)}
    node.create_subscription(State,'/mavros/state',states.append,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/factory_fastlio/odometry',odom.append,100)
    node.create_subscription(PoseStamped,'/mavros/vision_pose/pose',vision.append,100)
    epoch=time.time_ns()+100000000;start=time.monotonic()+.1;index=0;cycle=0
    log=(base/'mavros.log').open('w')
    mavros=subprocess.Popen(['ros2','launch',str(base/'mavros_platform.launch.py'),
        'fcu_url:=udp://127.0.0.1:14640@127.0.0.1:14641'],stdout=log,
        stderr=subprocess.STDOUT,start_new_session=True)

    def pump(seconds,predicate=None):
        nonlocal index,cycle
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if mavros.poll() is not None:raise RuntimeError('MAVROS exited')
            if states and states[-1].armed:raise RuntimeError('Unexpected armed state')
            now=time.monotonic()
            while feed and now>=start+cycle*period+ordered[index][0]:
                _,topic,m,relative=ordered[index];ns=epoch+int(cycle*period*1e9)+relative
                m.header.stamp.sec=ns//10**9;m.header.stamp.nanosec=ns%10**9
                if topic=='/livox/lidar':m.timebase=ns
                pubs[topic].publish(m);counts[topic]+=1;index+=1
                if index==len(ordered):index=0;cycle+=1
            rclpy.spin_once(node,timeout_sec=.002)
            if predicate and predicate():return True
        return bool(predicate and predicate())

    def check(label,value):
        checks[label]=bool(value);print(('PASS ' if value else 'FAIL ')+label,flush=True)
        assert value,label

    def manager(action):
        p=subprocess.Popen(['python3',str(platform/'01_apps/fast_lio/manage_stack.py'),action],
            env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        if not pump(40,lambda:p.poll() is not None):
            p.kill();p.wait();raise RuntimeError('Localization manager timeout')
        out,err=p.communicate();calls.append({'action':action,'code':p.returncode,
            'out':out.decode(),'err':err.decode()});print(out.decode(),flush=True)
        return p.returncode

    try:
        check('actual_px4_connected_disarmed',pump(20,lambda:states and states[-1].connected and not states[-1].armed))
        check('manager_starts_actual_stack',manager('start')==0)
        check('relay_initializes_from_actual_lio',pump(22,lambda:len(vision)>5))
        pump(max(0,75-(time.monotonic()-start)))
        check('actual_odometry_stream',len(odom)>300)
        check('actual_vision_stream',len(vision)>200)
        def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
        source_stamps={stamp(m) for m in odom};output_stamps=[stamp(m) for m in vision]
        check('relay_keeps_lio_source_stamps',all(s in source_stamps for s in output_stamps))
        check('relay_unique_increasing',all(a<b for a,b in zip(output_stamps,output_stamps[1:])))
        check('expected_frames',all(m.header.frame_id=='map' for m in vision) and
              all(m.header.frame_id=='camera_init' and m.child_frame_id=='body' for m in odom))
        feed=False;pump(1);stopped=len(vision);pump(5)
        check('sensor_loss_stops_vision_retransmission',len(vision)==stopped)
        check('manager_stops_owned_stack',manager('stop')==0)
        check('all_observed_states_disarmed',bool(states) and all(not s.armed for s in states))
    finally:
        # Only owned test records may be cleaned; no production services are touched.
        for kind in ['relay','localization']:
            record=base/'localization-runtime'/(kind+'.json')
            if record.exists():
                data=json.loads(record.read_text());proc=Path('/proc')/str(data['pid'])
                try:
                    ticks=(proc/'stat').read_text().split(') ',1)[1].split()[19]
                    if str(data['ticks'])==ticks and data['boot']==Path('/proc/sys/kernel/random/boot_id').read_text().strip():
                        os.killpg(data['pid'],signal.SIGINT)
                except (OSError,ProcessLookupError):pass
        try:os.killpg(mavros.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:mavros.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(mavros.pid,signal.SIGTERM);mavros.wait(timeout=5)
        log.close()
        (base/'fixture-result.json').write_text(json.dumps({'checks':checks,'calls':calls,
            'sensor_inputs':counts,'odom_count':len(odom),'vision_count':len(vision),
            'vision':[{'stamp':m.header.stamp.sec*1000000+m.header.stamp.nanosec//1000,
                       'position':[m.pose.position.x,m.pose.position.y,m.pose.position.z]} for m in vision],
            'states':[{'connected':s.connected,'armed':s.armed,'mode':s.mode} for s in states]},indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    return 0 if checks and all(checks.values()) else 1


if __name__=='__main__':
    raise SystemExit(main())
