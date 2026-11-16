#!/usr/bin/env python3
"""Build temporary observer, collect DDS delivery evidence, then remove owned temp files."""
from pathlib import Path
import subprocess,sys,tarfile,json,os,socket,time
repo=Path(__file__).resolve().parents[1];out=Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=False)
remote='/tmp/gimbal-delivery-check'
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
with tarfile.open(out/'inputs.tar','w') as a:
 if os.environ.get('GIMBAL_DELIVERY_CANDIDATE')=='1':a.add(repo/'platform/01_apps/mission_demo/workspace/src/rtsp_test',arcname='candidate-src',filter=lambda i:None if '__pycache__' in i.name else i)
 a.add(repo/'tools/gimbal_delivery_observer',arcname='observer-src');a.add(repo/'tools/test_gimbal_delivery_ros2.py',arcname='test.py')
with (out/'inputs.tar').open('rb') as f:subprocess.run(ssh+['mkdir '+remote+' && tar -xf - -C '+remote],stdin=f,check=True)
candidate=''
if os.environ.get('GIMBAL_DELIVERY_CANDIDATE')=='1':
 candidate='cmake -S '+remote+'/candidate-src -B '+remote+'/candidate-build -DCMAKE_INSTALL_PREFIX='+remote+'/candidate-install -DBUILD_TESTING=ON && cmake --build '+remote+'/candidate-build -j2 && ctest --test-dir '+remote+'/candidate-build --output-on-failure && cmake --install '+remote+'/candidate-build && '
cmd='source /home/orangepi/drone_platform/env.sh && '+candidate+'cmake -S '+remote+'/observer-src -B '+remote+'/observer-build && cmake --build '+remote+'/observer-build -j2 && python3 '+remote+'/test.py '+remote
tunnels=[];tunnel_logs=[]
try:
 if os.environ.get('GIMBAL_DELIVERY_LIVE')=='1':
  common=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ExitOnForwardFailure=yes']
  for label,args in [('reference',common+['-N','-L','127.0.0.1:21654:127.0.0.1:8554','orangepi@192.168.1.235']),('target',ssh[:-1]+['-o','ExitOnForwardFailure=yes','-N','-R','127.0.0.1:18655:127.0.0.1:21654',ssh[-1]])]:
   log=(out/(label+'-tunnel.log')).open('wb');tunnel_logs.append(log);tunnels.append(subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT))
  deadline=time.monotonic()+10
  while True:
   assert all(t.poll() is None for t in tunnels)
   try:
    with socket.create_connection(('127.0.0.1',21654),.5):break
   except OSError:
    assert time.monotonic()<deadline;time.sleep(.1)
  cmd=cmd.replace('python3 '+remote+'/test.py', 'GIMBAL_DELIVERY_SOURCE=rtsp://127.0.0.1:18655/gimbal python3 '+remote+'/test.py')
 with (out/'runner.log').open('wb') as f:p=subprocess.run(ssh+[cmd],stdout=f,stderr=subprocess.STDOUT,timeout=240)
finally:
 for t in reversed(tunnels):
  if t.poll() is None:t.terminate()
  try:t.wait(timeout=8)
  except subprocess.TimeoutExpired:t.kill();t.wait()
 for log in tunnel_logs:log.close()
with (out/'evidence.tar.gz').open('wb') as f:subprocess.run(ssh+['tar -czf - -C /tmp gimbal-delivery-check'],stdout=f,check=True)
subprocess.run(['tar','-xzf',str(out/'evidence.tar.gz'),'-C',str(out)],check=True)
if p.returncode==0:subprocess.run(ssh+['rm -rf -- '+remote],check=True)
(out/'runner-result.json').write_text(json.dumps(dict(returncode=p.returncode,evidence_retrieved=True,remote_removed=p.returncode==0),indent=2)+'\n')
print('runner exit',p.returncode);raise SystemExit(p.returncode)
