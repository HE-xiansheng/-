#!/usr/bin/env python3
"""Run isolated recording crash evidence collection; retain records only on computer."""
import io,json,subprocess,sys,tarfile
from pathlib import Path
repo=Path(__file__).resolve().parents[1];out=Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=False)
remote='/tmp/recording-crash-check'
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
with tarfile.open(out/'inputs.tar','w') as a:
 for n in ['record_session.py','recording_manage.py','recording_auto.py']:a.add(repo/'platform/01_apps/ego_planner'/n,arcname='source/'+n)
 a.add(repo/'tools/test_recording_crash_ros2.py',arcname='test.py')
with (out/'inputs.tar').open('rb') as f:subprocess.run(ssh+['mkdir '+remote+' && tar -xf - -C '+remote],stdin=f,check=True)
cmd='source /home/orangepi/drone_platform/env.sh && ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR='+remote+'/ros-logs python3 '+remote+'/test.py '+remote
with (out/'runner.log').open('wb') as log:proc=subprocess.run(ssh+[cmd],stdout=log,stderr=subprocess.STDOUT,timeout=180)
with (out/'evidence.tar.gz').open('wb') as f:subprocess.run(ssh+['tar -czf - -C /tmp recording-crash-check'],stdout=f,check=True)
subprocess.run(['tar','-xzf',str(out/'evidence.tar.gz'),'-C',str(out)],check=True)
result=out/'recording-crash-check/result.json';passed=result.exists() and json.loads(result.read_text()).get('passed') is True
# The test's finally block terminates owned processes, including on assertion failure.
subprocess.run(ssh+['rm -rf -- '+remote],check=True)
(out/'runner-result.json').write_text(json.dumps(dict(returncode=proc.returncode,passed=passed,evidence_retrieved=True,remote_removed=True),indent=2)+'\n')
print('passed',passed,'runner exit',proc.returncode);sys.exit(0 if passed and proc.returncode==0 else 1)
