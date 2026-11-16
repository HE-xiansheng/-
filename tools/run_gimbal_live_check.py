#!/usr/bin/env python3
"""Own temporary localhost SSH tunnels; leave both boards' services unchanged."""
import json,os,selectors,signal,socket,subprocess,time,sys,shlex
from pathlib import Path
repo=Path(__file__).resolve().parents[1];out=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else repo/'records/2026-11-16/gimbal-live';out.mkdir(parents=True,exist_ok=True)
common=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ServerAliveInterval=5','-o','ExitOnForwardFailure=yes']
ref=common+['orangepi@192.168.1.235']
board=common+['-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
procs=[];logs=[];remote=None;passed=False

def tunnel(name,cmd):
 f=(out/(name+'.log')).open('w');logs.append(f)
 p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p);return p

def ready(port,p):
 end=time.monotonic()+12
 while time.monotonic()<end:
  assert p.poll() is None,'tunnel terminated'
  try:
   with socket.create_connection(('127.0.0.1',port),.3):return
  except OSError:time.sleep(.1)
 raise RuntimeError('local tunnel unavailable')

try:
 subprocess.run(board+['mkdir /tmp/gimbal-live-ros2-check && cat > /tmp/gimbal-live-ros2-check/test.py'],input=(repo/'tools/test_gimbal_live_ros2.py').read_bytes(),check=True)
 a=tunnel('reference-tunnel',ref[:-1]+['-N','-L','127.0.0.1:21654:127.0.0.1:8554',ref[-1]]);ready(21654,a)
 b=tunnel('target-tunnel',board[:-1]+['-N','-R','127.0.0.1:18655:127.0.0.1:21654',board[-1]])
 deadline=time.monotonic()+12
 while True:
  assert b.poll() is None,'target tunnel terminated'
  probe=subprocess.run(board+['python3 -c '+shlex.quote("import socket;socket.create_connection(('127.0.0.1',18655),2).close()")],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=5)
  if probe.returncode==0:break
  if time.monotonic()>deadline:raise RuntimeError('reverse tunnel unavailable')
  time.sleep(.1)
 stderr=(out/'target-stderr.log').open('w');logs.append(stderr)
 command='source /home/orangepi/drone_platform/env.sh && export ROS_DOMAIN_ID=79 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/gimbal-live-ros2-check/ros-log && python3 -u /tmp/gimbal-live-ros2-check/test.py'
 remote=subprocess.Popen(board+[command],stdout=subprocess.PIPE,stderr=stderr,start_new_session=True)
 sel=selectors.DefaultSelector();sel.register(remote.stdout,selectors.EVENT_READ);received=b'';deadline=time.monotonic()+50
 while b'RELAY_READY' not in received:
  if time.monotonic()>deadline:raise RuntimeError('remote relay startup timeout')
  for key,_ in sel.select(.5):
   chunk=os.read(key.fd,65536)
   if not chunk:raise RuntimeError('remote exited before relay became ready: '+received.decode(errors='replace'))
   received+=chunk
 print('Live camera relay ready on .224',flush=True)
 env=dict(os.environ,GIMBAL_DESKTOP_TEST_OUTPUT=str(out/'desktop'),GIMBAL_DESKTOP_TEST_SCOPE='actual physical camera via .235 -> SSH -> .224 relay -> SSH -> desktop GStreamer; no gimbal commands')
 with (out/'desktop-test.log').open('w') as log:
  desktop=subprocess.run(['python3',str(repo/'tools/test_gimbal_rtsp_desktop.py')],cwd=repo,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=30)
 # Always allow the bounded remote test to finish, including when desktop failed.
 remaining,_=remote.communicate(timeout=70);received+=remaining
 (out/'target-stdout.log').write_bytes(received)
 passed=remote.returncode==0 and desktop.returncode==0
finally:
 if remote and remote.poll() is None:
  remote.terminate()
  try:remote.wait(timeout=5)
  except subprocess.TimeoutExpired:remote.kill();remote.wait(timeout=5)
 for p in reversed(procs):
  if p.poll() is None:os.killpg(p.pid,signal.SIGTERM)
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
 for f in logs:f.close()
 # Retrieve even failed tests. Never remove evidence until retrieval succeeds.
 with (out/'target-evidence.tar.gz').open('wb') as f:
  saved=subprocess.run(board+['tar -czf - -C /tmp gimbal-live-ros2-check'],stdout=f).returncode==0
 if saved:
  subprocess.run(['tar','-xzf',str(out/'target-evidence.tar.gz'),'-C',str(out)],check=True)
  # A healthy bounded script cleans its children before emitting result.json.
  if remote and remote.returncode==0:
   subprocess.run(board+['rm -rf -- /tmp/gimbal-live-ros2-check'],check=True)
 (out/'runner-result.json').write_text(json.dumps({'passed':passed,'remote_exit':None if remote is None else remote.returncode,'evidence_retrieved':saved},indent=2)+'\n')
print(json.dumps({'passed':passed,'output':str(out)}));assert passed
