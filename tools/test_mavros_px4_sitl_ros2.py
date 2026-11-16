#!/usr/bin/env python3
"""Real MAVROS2 to isolated official PX4 over loopback SSH transport; no arming."""
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState


def main():
    base=Path(sys.argv[1])
    assert os.environ.get('ROS_DOMAIN_ID')=='84'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    rclpy.init(); node=rclpy.create_node('mavros_sitl_ground_fixture')
    states=[]; poses=[]; landed=[]; sent=[]
    node.create_subscription(State,'/mavros/state',states.append,qos_profile_sensor_data)
    node.create_subscription(PoseStamped,'/mavros/local_position/pose',poses.append,qos_profile_sensor_data)
    node.create_subscription(ExtendedState,'/mavros/extended_state',landed.append,qos_profile_sensor_data)
    publisher=node.create_publisher(PoseStamped,'/mavros/vision_pose/pose',10)
    process=None; checks={}
    try:
        with (base/'mavros.log').open('w') as log:
            process=subprocess.Popen(['ros2','launch',str(base/'mavros_platform.launch.py'),
                'fcu_url:=udp://127.0.0.1:14640@127.0.0.1:14641'],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            end=time.monotonic()+60; next_publish=0
            while time.monotonic()<end:
                if process.poll() is not None:raise RuntimeError('MAVROS exited')
                rclpy.spin_once(node,timeout_sec=.01)
                if states and states[-1].armed:raise RuntimeError('Unexpected armed state')
                now=time.monotonic()
                if poses and now>=next_publish and states and states[-1].connected:
                    pose=copy.deepcopy(poses[-1]);pose.header.stamp=node.get_clock().now().to_msg()
                    publisher.publish(pose);next_publish=now+1/30
                    sent.append({'stamp':pose.header.stamp.sec*1000000+pose.header.stamp.nanosec//1000,
                                 'position':[pose.pose.position.x,pose.pose.position.y,pose.pose.position.z]})
            checks['connected']=any(s.connected for s in states)
            checks['all_disarmed']=bool(states) and all(not s.armed for s in states)
            checks['local_poses']=len(poses)>100
            checks['vision_published']=len(sent)>300
            checks['landed_state']=any(s.landed_state==ExtendedState.LANDED_STATE_ON_GROUND for s in landed)
            checks['fresh_pose']=bool(poses) and (node.get_clock().now().nanoseconds-
                (poses[-1].header.stamp.sec*10**9+poses[-1].header.stamp.nanosec))<2*10**9
            checks['connected_at_end']=bool(states) and states[-1].connected
    finally:
        if process is not None:
            try:os.killpg(process.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        (base/'fixture-result.json').write_text(json.dumps({'checks':checks,
            'states':[{'connected':s.connected,'armed':s.armed,'mode':s.mode} for s in states],
            'pose_count':len(poses),'vision_sent':sent,
            'process_returncode':process.returncode if process else None},indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(checks,indent=2))
    return 0 if checks and all(checks.values()) else 1


if __name__=='__main__':
    raise SystemExit(main())
