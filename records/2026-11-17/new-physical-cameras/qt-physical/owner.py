import json,os,signal,subprocess,sys,time
from pathlib import Path
import yaml
root=Path('/home/orangepi/drone_platform/03_tools/gimbal_relay');out=Path('/tmp/gimbal-qt-live-check')
config=yaml.safe_load((root/'gimbal.yml').read_text());config['rtspAddress']='127.0.0.1:18554';config['rtspTransports']=['tcp'];config['rtpAddress']='127.0.0.1:18000';config['rtcpAddress']='127.0.0.1:18001';config['paths']['gimbal']['source']='rtsp://192.168.144.108:554' if sys.argv[1]=='physical' else 'rtsp://127.0.0.1:18655/gimbal'
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
