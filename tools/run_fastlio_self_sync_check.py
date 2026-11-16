#!/usr/bin/env python3
"""Compare installed and candidate FAST-LIO with a forced IMU clock offset."""
from pathlib import Path
import json,subprocess,sys,tarfile
repo=Path(__file__).resolve().parents[1];out=Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=False)
remote='/tmp/fastlio-self-sync-check'
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
with tarfile.open(out/'inputs.tar','w') as a:
 for name in ['test_fastlio_self_sync_ros2.py','validate_fastlio_replay.py']:a.add(repo/'tools'/name,arcname=name)
 a.add('/home/tyzk/drone-migration-work/reference/livox235-sample.jsonl.gz',arcname='livox.jsonl.gz')
with (out/'inputs.tar').open('rb') as f:subprocess.run(ssh+['mkdir '+remote+' && tar -xf - -C '+remote],stdin=f,check=True)
cmd='source /home/orangepi/drone_platform/env.sh && ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR='+remote+'/ros-log python3 '+remote+'/test_fastlio_self_sync_ros2.py '+remote
with (out/'runner.log').open('wb') as f:p=subprocess.run(ssh+[cmd],stdout=f,stderr=subprocess.STDOUT,timeout=180)
with (out/'evidence.tar.gz').open('wb') as f:subprocess.run(ssh+['tar -czf - -C /tmp fastlio-self-sync-check'],stdout=f,check=True)
subprocess.run(['tar','-xzf',str(out/'evidence.tar.gz'),'-C',str(out)],check=True)
result=out/'fastlio-self-sync-check/result.json';passed=result.exists() and json.loads(result.read_text()).get('passed') is True
subprocess.run(ssh+['rm -rf -- '+remote],check=True)
(out/'runner-result.json').write_text(json.dumps(dict(passed=passed,returncode=p.returncode,evidence_retrieved=True,remote_removed=True),indent=2)+'\n')
print('passed',passed,'exit',p.returncode);raise SystemExit(0 if passed and p.returncode==0 else 1)
