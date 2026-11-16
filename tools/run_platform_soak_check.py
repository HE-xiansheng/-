#!/usr/bin/env python3
from pathlib import Path
import json,os,subprocess,sys,tarfile
repo=Path(__file__).resolve().parents[1];out=Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=False);remote='/tmp/platform-soak-check'
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
with tarfile.open(out/'inputs.tar','w') as a:
 for n in ['record_session.py','recording_manage.py','recording_auto.py']:a.add(repo/'platform/01_apps/ego_planner'/n,arcname='recording-source/'+n)
 for n in ['test_platform_soak_ros2.py','validate_fastlio_replay.py','analyze_soak_preview_headers.py']:a.add(repo/'tools'/n,arcname=n)
 a.add(repo/'tools/gimbal_delivery_observer',arcname='observer-src')
 a.add(Path('/home/tyzk/drone-migration-work/reference/livox235-sample.jsonl.gz'),arcname='livox.jsonl.gz')
 if os.environ.get('PLATFORM_SOAK_CAMERA_LAUNCH_CANDIDATE')=='1':
  for name in ['launch/gemini335l.launch.py','config/gemini335l.yaml','config/fastdds_large_images.xml']:
   a.add(repo/'platform/02_drivers/orbbec'/name,arcname='camera-candidate/'+name)
with (out/'inputs.tar').open('rb') as f:subprocess.run(ssh+['mkdir '+remote+' && tar -xf - -C '+remote],stdin=f,check=True)
seconds=str(int(os.environ.get('PLATFORM_SOAK_SECONDS','180')))
cmd='source /home/orangepi/drone_platform/env.sh && cmake -S '+remote+'/observer-src -B '+remote+'/observer-build && cmake --build '+remote+'/observer-build -j2 && ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR='+remote+'/ros-log PLATFORM_SOAK_SECONDS='+seconds+' python3 '+remote+'/test_platform_soak_ros2.py '+remote
if os.environ.get('PLATFORM_SOAK_FASTLIO_CANDIDATE')=='1':
 cmd=cmd.replace('ROS_DOMAIN_ID=83', 'AMENT_PREFIX_PATH=/tmp/fastlio-sync-candidate/install:$AMENT_PREFIX_PATH LD_LIBRARY_PATH=/tmp/fastlio-sync-candidate/install/lib:$LD_LIBRARY_PATH ROS_DOMAIN_ID=83')
if os.environ.get('PLATFORM_SOAK_CAMERA_PROFILE')=='1':
 cmd=cmd.replace('ROS_DOMAIN_ID=83','PLATFORM_SOAK_CAMERA_PROFILE=1 ROS_DOMAIN_ID=83')
if os.environ.get('PLATFORM_SOAK_CAMERA_LAUNCH_CANDIDATE')=='1':
 cmd=cmd.replace('ROS_DOMAIN_ID=83','PLATFORM_SOAK_CAMERA_LAUNCH='+remote+'/camera-candidate/launch/gemini335l.launch.py ROS_DOMAIN_ID=83')
with (out/'runner.log').open('wb') as f:p=subprocess.run(ssh+[cmd],stdout=f,stderr=subprocess.STDOUT,timeout=int(seconds)+240)
with (out/'evidence.tar.gz').open('wb') as f:subprocess.run(ssh+['tar -I "gzip -1" -cf - -C /tmp platform-soak-check'],stdout=f,check=True)
subprocess.run(['tar','-xzf',str(out/'evidence.tar.gz'),'-C',str(out)],check=True)
result=out/'platform-soak-check/result.json';passed=result.exists() and json.loads(result.read_text()).get('passed') is True
# All test-owned children have been stopped by the fixture before collection.
subprocess.run(ssh+['rm -rf -- '+remote],check=True)
(out/'runner-result.json').write_text(json.dumps(dict(passed=passed,returncode=p.returncode,evidence_retrieved=True,remote_removed=True),indent=2)+'\n')
print('passed',passed,'exit',p.returncode);raise SystemExit(0 if passed and p.returncode==0 else 1)
