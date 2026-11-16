#!/usr/bin/env python3
"""Real VideoPanel buttons/render/reconnect through owned read-only tunnels."""
import json,os,selectors,shlex,signal,socket,subprocess,sys,time
from pathlib import Path
repo=Path(__file__).resolve().parents[1]
out=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else repo/'records/2026-11-16/gimbal-qt-live'
out.mkdir(parents=True,exist_ok=True)
common=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ServerAliveInterval=5','-o','ExitOnForwardFailure=yes']
reference=common+['orangepi@192.168.1.235']
board=common+['-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
owner_code=r'''import json,os,signal,subprocess,sys,time
from pathlib import Path
import yaml
root=Path('/home/orangepi/drone_platform/03_tools/gimbal_relay');out=Path('/tmp/gimbal-qt-live-check')
config=yaml.safe_load((root/'gimbal.yml').read_text());config['rtspAddress']='127.0.0.1:18554';config['rtspTransports']=['tcp'];config['rtpAddress']='127.0.0.1:18000';config['rtcpAddress']='127.0.0.1:18001';config['paths']['gimbal']['source']='rtsp://127.0.0.1:18655/gimbal'
(out/'relay.yml').write_text(yaml.safe_dump(config))
with (out/'relay.log').open('w') as log:
 p=subprocess.Popen(['bash',str(root/'start_relay.sh'),str(out/'relay.yml')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 (out/'owned-process.json').write_text(json.dumps({'pid':p.pid,'start_ticks':Path('/proc/'+str(p.pid)+'/stat').read_text().rsplit(')',1)[1].split()[19]}))
 try:
  end=time.monotonic()+25
  while 'stream is available and online' not in (out/'relay.log').read_text():
   assert p.poll() is None and time.monotonic()<end,'relay startup failed';time.sleep(.1)
  print('RELAY_READY',flush=True)
  sys.stdin.buffer.read(1)
 finally:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
  (out/'owner-result.json').write_text(json.dumps({'relay_exit':p.returncode,'stopped':p.poll() is not None}))
  print('RELAY_STOPPED',p.returncode,flush=True)
'''
handles=[];processes=[];owner=None;qt=None;passed=False;retrieved=False;dropped=False;restored=False;events=[]

def start(name,command,**kw):
 f=(out/(name+'.log')).open('w');handles.append(f)
 p=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,**kw);processes.append(p);return p

def stop(p):
 if p.poll() is None:os.killpg(p.pid,signal.SIGTERM)
 try:p.wait(timeout=8)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)

def local_ready(port,p):
 end=time.monotonic()+10
 while time.monotonic()<end:
  assert p.poll() is None,'owned tunnel stopped'
  try:
   with socket.create_connection(('127.0.0.1',port),.2):return
  except OSError:time.sleep(.1)
 raise RuntimeError('tunnel listener not ready')

def target_tunnel(name):
 p=start(name,board[:-1]+['-N','-R','127.0.0.1:18655:127.0.0.1:21654','-L','127.0.0.1:8554:127.0.0.1:18554',board[-1]])
 local_ready(8554,p);return p

try:
 # Fail rather than take over existing listeners or phase directories.
 for port in [21654,8554]:
  with socket.socket() as s:s.bind(('127.0.0.1',port))
 subprocess.run(board+['mkdir /tmp/gimbal-qt-live-check && cat > /tmp/gimbal-qt-live-check/owner.py'],input=owner_code.encode(),check=True)
 a=start('reference-tunnel',reference[:-1]+['-N','-L','127.0.0.1:21654:127.0.0.1:8554',reference[-1]]);local_ready(21654,a)
 b=target_tunnel('target-tunnel-first')
 stderr=(out/'owner-stderr.log').open('w');handles.append(stderr)
 owner=subprocess.Popen(board+['python3 -u /tmp/gimbal-qt-live-check/owner.py'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=stderr,start_new_session=True)
 sel=selectors.DefaultSelector();sel.register(owner.stdout,selectors.EVENT_READ);buffer=b'';end=time.monotonic()+35
 while b'RELAY_READY' not in buffer:
  if time.monotonic()>end:raise RuntimeError('relay owner startup timed out')
  for key,_ in sel.select(.3):
   chunk=os.read(key.fd,65536)
   if not chunk:raise RuntimeError('relay owner exited: '+buffer.decode(errors='replace'))
   buffer+=chunk
 (out/'owner-stdout.log').write_bytes(buffer)
 qt_root='/home/tyzk/dianyunGCS-review-20261026/.deps/Qt'
 env=dict(os.environ,QT_QPA_PLATFORM='offscreen',QT_QUICK_BACKEND='software',LD_LIBRARY_PATH=qt_root+'/lib',QML_IMPORT_PATH=qt_root+'/qml',XDG_CONFIG_HOME=str(out/'private-config'),XDG_DATA_HOME=str(out/'private-data'),XDG_CACHE_HOME=str(out/'private-cache'),no_proxy='127.0.0.1',NO_PROXY='127.0.0.1')
 qt=start('qt', ['/home/tyzk/drone-migration-work/build-groundstation-video-test/video_reconnect_test','--real-gimbal'],env=env)
 end=time.monotonic()+110
 while qt.poll() is None:
  if time.monotonic()>end:raise RuntimeError('Qt live test timeout')
  log=(out/'qt.log').read_text(errors='replace')
  if 'REQUEST_TRANSPORT_DROP' in log and not dropped:
   stop(b);dropped=True;events.append({'event':'stopped_owned_target_tunnel','monotonic':time.monotonic()})
  if 'TRANSPORT_LOSS_OBSERVED' in log and dropped and not restored:
   b=target_tunnel('target-tunnel-restored');restored=True;events.append({'event':'restored_owned_target_tunnel','monotonic':time.monotonic()})
  time.sleep(.1)
 passed=qt.returncode==0 and dropped and restored
finally:
 if qt:
  # If an assertion aborted the Qt parent, clean only its process group as well.
  try:os.killpg(qt.pid,signal.SIGTERM)
  except ProcessLookupError:pass
  if qt.poll() is None:qt.wait(timeout=8)
 if owner and owner.poll() is None:
  remaining,_=owner.communicate(input=b'q',timeout=15)
  with (out/'owner-stdout.log').open('ab') as f:f.write(remaining)
 for p in reversed(processes):stop(p)
 for f in handles:f.close()
 with (out/'target-evidence.tar.gz').open('wb') as f:
  retrieved=subprocess.run(board+['tar -czf - -C /tmp gimbal-qt-live-check'],stdout=f).returncode==0
 if retrieved:
  subprocess.run(['tar','-xzf',str(out/'target-evidence.tar.gz'),'-C',str(out)],check=True)
  result_file=out/'gimbal-qt-live-check/owner-result.json'
  if result_file.exists() and json.loads(result_file.read_text())=={'relay_exit':0,'stopped':True}:
   subprocess.run(board+['rm -rf -- /tmp/gimbal-qt-live-check'],check=True)
 result={'passed':passed,'qt_exit':None if qt is None else qt.returncode,'dropped':dropped,'restored':restored,'evidence_retrieved':retrieved,'events':events,'scope':'actual VideoPanel/QImage/renderer/buttons and real camera RTSP; temporary localhost tunnels, no flight task or gimbal movement'}
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result));assert passed
