#!/usr/bin/env python3
"""Run native person -> MAVROS -> disarmed isolated PX4 wire validation."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile

repo=Path(__file__).resolve().parents[1]
base=Path(sys.argv[1]).resolve()
base.mkdir(parents=True,exist_ok=False)
remote='/tmp/person-mavros-wire'
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o',
    'UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts',
    '-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
with tarfile.open(base/'inputs.tar','w') as archive:
    for src,dst in [('tools/sitl_stdio_transport.py','sitl_stdio_transport.py'),
        ('tools/test_person_mavros_wire_ros2.py','test_person_mavros_wire_ros2.py'),
        ('platform/02_drivers/sensors/launch/mavros_platform.launch.py','mavros_platform.launch.py')]:
        archive.add(repo/src,arcname=dst)
    if os.environ.get('PERSON_WIRE_CANDIDATE')=='1':
        archive.add(repo/'platform/01_apps/mission_demo/workspace/src/track_test',arcname='candidate-src')
    archive.add('/home/tyzk/drone-migration-work/fixtures/bus.jpg',arcname='bus.jpg')
with (base/'inputs.tar').open('rb') as source:
    subprocess.run(ssh+['mkdir '+remote+' && tar -xf - -C '+remote],stdin=source,check=True)
if os.environ.get('PERSON_WIRE_CANDIDATE')=='1':
    with (base/'candidate-build.log').open('wb') as output:
        subprocess.run(ssh+['bash -c "source /home/orangepi/drone_platform/env.sh; cmake -S '+remote+'/candidate-src -B '+remote+'/candidate-build -DCMAKE_INSTALL_PREFIX='+remote+'/candidate-install && cmake --build '+remote+'/candidate-build -j2 && cmake --install '+remote+'/candidate-build"'],stdout=output,stderr=subprocess.STDOUT,check=True)
ab_r,ab_w=os.pipe();ba_r,ba_w=os.pipe()
processes=[];logs=[];success=False
try:
    for name in ['remote-transport','px4-transport','fixture']:
        logs.append((base/(name+'.log')).open('wb'))
    processes.append(subprocess.Popen(ssh+['python3 '+remote+'/sitl_stdio_transport.py relay --runtime '+remote+'/relay --seconds 85'],stdin=ba_r,stdout=ab_w,stderr=logs[0]))
    processes.append(subprocess.Popen(['unshare','--user','--map-root-user','--net',
        '/home/tyzk/drone-migration-work/px4-sitl-venv/bin/python',str(repo/'tools/sitl_stdio_transport.py'),
        'px4','--build','/home/tyzk/drone-migration-work/build-px4-sitl',
        '--runtime',str(base/'px4-runtime'),'--seconds','75'],stdin=ab_r,stdout=ba_w,stderr=logs[1]))
    for fd in [ab_r,ab_w,ba_r,ba_w]:os.close(fd)
    command='bash -c "source /home/orangepi/drone_platform/env.sh; export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR='+remote+'/ros-log; python3 '+remote+'/test_person_mavros_wire_ros2.py '+remote+'"'
    processes.append(subprocess.Popen(ssh+[command],stdout=logs[2],stderr=subprocess.STDOUT))
    processes[2].wait(timeout=85)
    processes[1].wait(timeout=85)
    processes[0].wait(timeout=20)
    success=all(p.returncode==0 for p in processes)
finally:
    for p in reversed(processes):
        if p.poll() is None:
            p.terminate()
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
    for log in logs:log.close()
    with (base/'remote-evidence.tar.gz').open('wb') as output:
        subprocess.run(ssh+['tar -czf - -C /tmp person-mavros-wire'],stdout=output,check=True)
    subprocess.run(['tar','-xzf',str(base/'remote-evidence.tar.gz'),'-C',str(base)],check=True)
    # Only remove remote evidence after all owned transports and fixture exited normally.
    if success:subprocess.run(ssh+['rm -rf -- '+remote],check=True)
    (base/'runner-result.json').write_text(json.dumps(dict(passed=success,returncodes=[p.returncode for p in processes],remote_removed=success),indent=2)+'\n')
print('returncodes',[p.returncode for p in processes],flush=True)
if not success:raise SystemExit(1)
