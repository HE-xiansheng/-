#!/usr/bin/env python3
"""候选定位订阅现有传感器，只发布到独立命名空间；不接飞控转发。"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from mavros_msgs.msg import State


def main():
    root=Path('/tmp/fastlio-source-clock-candidate')
    assert os.environ.get('ROS_DOMAIN_ID','0')=='0'
    out=root/'live'
    out.mkdir(exist_ok=True)
    rclpy.init();node=rclpy.create_node('candidate_clock_readonly')
    rows=[];cloud=[];states=[]
    def receive(m):
        ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        p,q=m.pose.pose.position,m.pose.pose.orientation
        rows.append(dict(at=time.monotonic(),stamp=ns,age=(time.time_ns()-ns)*1e-9,
                         frame=m.header.frame_id,child=m.child_frame_id,
                         pose=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]))
    node.create_subscription(Odometry,'/gcs_clock_candidate/odometry',receive,100)
    node.create_subscription(PointCloud2,'/gcs_clock_candidate/cloud_body',lambda m:cloud.append(m.header.stamp.sec*10**9+m.header.stamp.nanosec),10)
    node.create_subscription(State,'/mavros/state',lambda m:states.append(dict(connected=m.connected,armed=m.armed)),10)
    remaps={'/Odometry':'odometry','/cloud_registered_body':'cloud_body','/cloud_registered':'cloud_world',
            '/cloud_effected':'cloud_effected','/Laser_map':'map','/path':'path','/tf':'tf'}
    args=[str(root/'install/lib/fast_lio/fastlio_mapping'),'--ros-args','-r','__ns:=/gcs_clock_candidate',
          '--params-file','/home/orangepi/drone_platform/01_apps/fast_lio/config/lidar.yaml']
    for old,new in remaps.items():args+=['-r',old+':=/gcs_clock_candidate/'+new]
    with (out/'node.log').open('w') as log:
        process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        begin=time.monotonic()
        try:
            while time.monotonic()-begin<75:
                rclpy.spin_once(node,timeout_sec=.02)
                if process.poll() is not None:raise RuntimeError('candidate LIO exited')
                if states and states[-1]['armed']:raise RuntimeError('controller armed; stopping only candidate observer')
        finally:
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:process.kill();process.wait()
    text=(out/'node.log').read_text()
    stable=[row for row in rows if row['at']>=begin+5]
    checks={'candidate clock locks without false shift':'Source clock alignment verified' in text and 'Self sync IMU and LiDAR' not in text,
            'at least 600 unique real source measurements':len(stable)>=600 and len(cloud)>=600,
            'strictly increasing source stamps':bool(rows) and all(b['stamp']>a['stamp'] for a,b in zip(rows,rows[1:])),
            'settled source ages meet existing relay 0.2s limit':bool(stable) and all(-.02<=row['age']<=.2 for row in stable),
            'finite unit poses and expected frames':bool(rows) and all(row['frame']=='camera_init' and row['child']=='body' and all(math.isfinite(v) for v in row['pose']) and abs(sum(v*v for v in row['pose'][3:])-1)<.01 for row in rows),
            'real controller connected and never armed':bool(states) and all(s['connected'] and not s['armed'] for s in states),
            'candidate exits cleanly':process.returncode==0}
    summary=dict(count=len(stable),cloud_count=len(cloud),source_hz=(len(stable)-1)*1e9/(stable[-1]['stamp']-stable[0]['stamp']) if len(stable)>1 else None,
                 max_source_age_s=max(row['age'] for row in stable) if stable else None,
                 max_receipt_gap_s=max(b['at']-a['at'] for a,b in zip(stable,stable[1:])) if len(stable)>1 else None)
    result=dict(passed=all(checks.values()),checks=checks,streams=summary,scope='actual existing MID360 input; candidate outputs only under /gcs_clock_candidate; no MAVROS relay, no task or control calls')
    (out/'odometry.json').write_text(json.dumps(rows))
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    node.destroy_node();rclpy.try_shutdown()
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
