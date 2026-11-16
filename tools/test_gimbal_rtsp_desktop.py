#!/usr/bin/env python3
"""Read synthetic relay through SSH forwarding with the Linux GCS decoder pipeline."""
import hashlib,json,os,selectors,signal,socket,subprocess,time
from pathlib import Path
out=Path(os.environ.get('GIMBAL_DESKTOP_TEST_OUTPUT','records/2026-11-16/gimbal-relay'));out.mkdir(parents=True,exist_ok=True)
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ExitOnForwardFailure=yes','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','-N','-L','127.0.0.1:11954:127.0.0.1:18554','orangepi@192.168.1.224']
procs=[];frames=0;unique=set();times=[];completed=False
with (out/'ssh-tunnel.log').open('w') as tunnel_log,(out/'desktop-decoder.log').open('w') as decoder_log:
 try:
  tunnel=subprocess.Popen(ssh,stdout=tunnel_log,stderr=subprocess.STDOUT,start_new_session=True);procs.append(tunnel)
  deadline=time.monotonic()+8
  while time.monotonic()<deadline:
   if tunnel.poll() is not None:raise RuntimeError('SSH tunnel exited')
   try:
    with socket.create_connection(('127.0.0.1',11954),.3):break
   except OSError:time.sleep(.1)
  cmd=['/usr/bin/gst-launch-1.0','-q','rtspsrc','location=rtsp://127.0.0.1:11954/gimbal','protocols=tcp','latency=100','drop-on-latency=true','tcp-timeout=5000000','!','rtph264depay','!','h264parse','!','avdec_h264','!','queue','max-size-buffers=1','max-size-bytes=0','max-size-time=0','leaky=downstream','!','videoconvert','!','videoscale','add-borders=true','!','video/x-raw,format=RGB,width=640,height=360,pixel-aspect-ratio=1/1','!','fdsink','fd=1','sync=false']
  (out/'desktop-command.json').write_text(json.dumps(cmd,indent=2)+'\n')
  env=dict(os.environ,no_proxy='127.0.0.1',NO_PROXY='127.0.0.1')
  decoder=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=decoder_log,env=env,start_new_session=True);procs.append(decoder)
  sel=selectors.DefaultSelector();sel.register(decoder.stdout,selectors.EVENT_READ);buf=b'';start=time.monotonic();deadline=start+18
  while time.monotonic()<deadline:
   if decoder.poll() is not None:raise RuntimeError('desktop decoder exited')
   for key,_ in sel.select(.2):
    chunk=os.read(key.fd,1024*1024)
    if not chunk:raise RuntimeError('decoder EOF')
    buf+=chunk
    while len(buf)>=640*360*3:
     frame=buf[:640*360*3];buf=buf[len(frame):];frames+=1;unique.add(hashlib.sha256(frame).hexdigest());times.append(time.monotonic())
   if times and time.monotonic()-times[0]>8:break
  completed=True
 finally:
  for p in reversed(procs):
   if p.poll() is None:os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=8)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
  hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0
  result={'frames':frames,'different_frames':len(unique),'fps':hz,'size':[640,360],'format':'RGB','passed':completed and frames>=60 and len(unique)>=30 and hz>10,'scope':os.environ.get('GIMBAL_DESKTOP_TEST_SCOPE','actual SSH-forwarded RTSP and same GStreamer pipeline as Qt; synthetic input, no gimbal motion')}
  (out/'desktop-result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
assert result['passed']
