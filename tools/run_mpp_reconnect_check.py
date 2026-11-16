#!/usr/bin/env python3
"""Build temporary observer, collect DDS delivery evidence, then remove owned temp files."""
from pathlib import Path
import subprocess,sys,tarfile,json,os,socket,time
repo=Path(__file__).resolve().parents[1];out=Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=False)
remote='/tmp/mpp-reconnect-check'
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
with tarfile.open(out/'inputs.tar','w') as a:
 if os.environ.get('MPP_RECONNECT_CANDIDATE')=='1':a.add(repo/'platform/01_apps/mission_demo/workspace/src/rtsp_test',arcname='candidate-src',filter=lambda i:None if '__pycache__' in i.name else i)
 a.add(repo/'tools/gimbal_delivery_observer',arcname='observer-src');a.add(repo/'tools/test_mpp_reconnect_ros2.py',arcname='test.py')
with (out/'inputs.tar').open('rb') as f:subprocess.run(ssh+['mkdir '+remote+' && tar -xf - -C '+remote],stdin=f,check=True)
candidate=''
if os.environ.get('MPP_RECONNECT_CANDIDATE')=='1':
 candidate='cmake -S '+remote+'/candidate-src -B '+remote+'/candidate-build -DCMAKE_INSTALL_PREFIX='+remote+'/candidate-install -DBUILD_TESTING=ON && cmake --build '+remote+'/candidate-build -j2 && ctest --test-dir '+remote+'/candidate-build --output-on-failure && cmake --install '+remote+'/candidate-build && '
cmd='source /home/orangepi/drone_platform/env.sh && '+candidate+'cmake -S '+remote+'/observer-src -B '+remote+'/observer-build && cmake --build '+remote+'/observer-build -j2 && python3 '+remote+'/test.py '+remote
if os.environ.get('MPP_RECONNECT_DEFAULTS')=='1':cmd=cmd.replace('python3 '+remote+'/test.py', 'MPP_RECONNECT_DEFAULTS=1 python3 '+remote+'/test.py')
with (out/'runner.log').open('wb') as f:p=subprocess.run(ssh+[cmd],stdout=f,stderr=subprocess.STDOUT,timeout=300)
with (out/'evidence.tar.gz').open('wb') as f:subprocess.run(ssh+['tar -czf - -C /tmp mpp-reconnect-check'],stdout=f,check=True)
subprocess.run(['tar','-xzf',str(out/'evidence.tar.gz'),'-C',str(out)],check=True)
if p.returncode==0:subprocess.run(ssh+['rm -rf -- '+remote],check=True)
(out/'runner-result.json').write_text(json.dumps(dict(returncode=p.returncode,evidence_retrieved=True,remote_removed=p.returncode==0),indent=2)+'\n')
print('runner exit',p.returncode);raise SystemExit(p.returncode)
