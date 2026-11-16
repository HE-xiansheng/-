#!/usr/bin/env python3
"""Computer-side orchestration: isolated PX4 <stdio SSH> loopback MAVROS2."""
import os
from pathlib import Path
import subprocess
import time

root=Path(__file__).resolve().parents[1]
base=root/'records/2026-11-16/fastlio-px4-sitl'
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o',
     'UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts',
     '-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
ab_r,ab_w=os.pipe();ba_r,ba_w=os.pipe()
processes=[];logs=[]
try:
    for name in ['remote-transport','px4-transport','fixture']:
        logs.append((base/(name+'.log')).open('wb'))
    processes.append(subprocess.Popen(ssh+[
        'python3 /tmp/fastlio-px4-sitl/tools/sitl_stdio_transport.py relay '
        '--runtime /tmp/fastlio-px4-sitl/relay --seconds 115'],
        stdin=ba_r,stdout=ab_w,stderr=logs[0]))
    processes.append(subprocess.Popen(['unshare','--user','--map-root-user','--net',
        '/home/tyzk/drone-migration-work/px4-sitl-venv/bin/python',
        str(root/'tools/sitl_stdio_transport.py'),'px4','--build',
        '/home/tyzk/drone-migration-work/build-px4-sitl','--runtime',
        str(base/'px4-runtime'),'--seconds','105','--diagnostics-at','65','85'],
        stdin=ab_r,stdout=ba_w,stderr=logs[1]))
    for fd in [ab_r,ab_w,ba_r,ba_w]:os.close(fd)
    processes.append(subprocess.Popen(ssh+[
        'bash -c "source /home/orangepi/drone_platform/env.sh; '
        'export ROS_DOMAIN_ID=82 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/fastlio-px4-sitl/ros-log; '
        'python3 /tmp/fastlio-px4-sitl/tools/test_fastlio_px4_sitl_ros2.py '
        '/tmp/fastlio-px4-sitl/fixture"'],stdout=logs[2],stderr=subprocess.STDOUT))
    processes[2].wait(timeout=110)
    processes[1].wait(timeout=25)
    processes[0].wait(timeout=15)
    print('returncodes', [p.returncode for p in processes],flush=True)
    if any(p.returncode for p in processes):raise RuntimeError('SITL stage failed')
finally:
    for p in reversed(processes):
        if p.poll() is None:
            p.terminate()
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
    for log in logs:log.close()
