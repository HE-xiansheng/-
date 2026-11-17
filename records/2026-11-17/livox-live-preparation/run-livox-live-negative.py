from pathlib import Path
import sys,subprocess,tarfile,time,json,io,os,signal
repo=Path('/home/tyzk/drone-migration-github');sys.path.insert(0,str(repo/'tools'))
from livox_live_protocol import write_frame
out=Path('/home/tyzk/drone-migration-work/livox-live-negative');out.mkdir(exist_ok=False)
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224'];remote='/tmp/livox-live-check'
with tarfile.open(out/'inputs.tar','w') as a:
 for name in ['test_livox_live_ros2.py','livox_live_protocol.py','validate_fastlio_replay.py']:a.add(repo/'tools'/name,arcname=name)
with (out/'inputs.tar').open('rb') as f:subprocess.run(ssh+['mkdir '+remote+' && tar -xf - -C '+remote],stdin=f,check=True,timeout=15)
p=None
try:
 with (out/'consumer.log').open('wb') as log:
  p=subprocess.Popen(ssh+['source /home/orangepi/drone_platform/env.sh && ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR='+remote+'/ros-log python3 -u '+remote+'/test_livox_live_ros2.py '+remote],stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  deadline=time.monotonic()+60
  while time.monotonic()<deadline:
   if p.poll() is not None:raise RuntimeError('consumer exited before ready')
   if 'LIVE_INPUT_READY' in (out/'consumer.log').read_text():break
   time.sleep(.1)
  else:raise TimeoutError('readiness')
  frame=io.BytesIO();write_frame(frame,1,b'invalid-data-should-never-reach-decoder',time.time_ns());raw=bytearray(frame.getvalue());raw[8]^=1
  (out/'corrupt.stream').write_bytes(raw);p.stdin.write(raw);p.stdin.close();p.wait(timeout=45)
finally:
 if p and p.poll() is None:
  if p.stdin and not p.stdin.closed:p.stdin.close()
  try:p.wait(timeout=40)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
 with (out/'evidence.tar.gz').open('wb') as f:subprocess.run(ssh+['tar -czf - -C /tmp livox-live-check'],stdout=f,check=True,timeout=30)
 subprocess.run(['tar','-xzf',str(out/'evidence.tar.gz'),'-C',str(out)],check=True)
 subprocess.run(ssh+['rm -rf -- '+remote],check=True,timeout=15)
result=json.loads((out/'livox-live-check/result.json').read_text())
checks={'actual_lio_startup_ready':result['checks'].get('actual_lio_consumers_ready') is True,'damaged_transport_rejected':any('checksum mismatch' in e for e in result['reader_errors']),'no_input_was_published':not result['input_counts'],'does_not_claim_live_pass':result['passed'] is False,'consumer_returns_failure':p.returncode==1,'owned_LIO_relay_exit_normally':all(v==0 for v in result['returncodes'].values())}
(out/'guard-result.json').write_text(json.dumps({'checks':checks,'passed':all(checks.values()),'remote_removed':True,'scope':'corrupt transport negative test; no source .235 process or live data'},indent=2)+'\n');print(checks);assert all(checks.values())
