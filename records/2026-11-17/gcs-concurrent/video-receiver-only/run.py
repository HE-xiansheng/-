from pathlib import Path
import subprocess,sys,json,tarfile,time,select,os,shlex
out=Path('/home/tyzk/drone-migration-work/gcs-video-diag');out.mkdir(exist_ok=False)
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
remote='/tmp/gcs-video-diag'
code='''import os,subprocess,time,yaml,signal
from pathlib import Path
p=Path('/tmp/gcs-video-diag');p.mkdir(exist_ok=False)
c=yaml.safe_load(Path('/home/orangepi/drone_platform/03_tools/gimbal_relay/gimbal.yml').read_text());c['rtspAddress']='0.0.0.0:18555';c['rtspTransports']=['tcp'];c['paths']={'gimbal':{'source':'publisher'}};c['authInternalUsers']=[{'user':'any','ips':['127.0.0.1'],'permissions':[{'action':'publish','path':'gimbal'},{'action':'read','path':'gimbal'}]},{'user':'any','ips':[os.environ['SSH_CONNECTION'].split()[0]],'permissions':[{'action':'read','path':'gimbal'}]}];(p/'stream.yml').write_text(yaml.safe_dump(c))
subprocess.run(['ffmpeg','-v','error','-f','lavfi','-i','testsrc2=size=1920x1080:rate=30','-t','3','-c:v','libx264','-preset','ultrafast','-tune','zerolatency','-threads','2','-pix_fmt','yuv420p','-g','30',str(p/'video.mp4')],check=True)
children=[];logs=[]
try:
 for name,args in [('server',['/home/orangepi/drone_platform/03_tools/gimbal_relay/mediamtx',str(p/'stream.yml')]),('source',['ffmpeg','-v','warning','-re','-stream_loop','-1','-i',str(p/'video.mp4'),'-an','-c:v','copy','-f','rtsp','-rtsp_transport','tcp','rtsp://127.0.0.1:18555/gimbal'])]:
  f=(p/(name+'.log')).open('w');logs.append(f);children.append(subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True));time.sleep(.5)
 print('READY',flush=True)
 deadline=time.monotonic()+110
 while time.monotonic()<deadline and not (p/'stop').exists():time.sleep(.2)
finally:
 for child in reversed(children):
  if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
  try:child.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
 for f in logs:f.close()
'''
p=subprocess.Popen(ssh+['python3 -u -c '+shlex.quote(code)],stdout=subprocess.PIPE,stderr=(out/'remote.log').open('wb'),text=True)
assert p.stdout.readline().strip()=='READY'
results={}
try:
 for name,extra in [('production',[]),('no-drop',['drop-on-latency=false'])]:
  args=['gst-launch-1.0','-q','rtspsrc','location=rtsp://192.168.1.224:18555/gimbal','protocols=tcp','latency=100','drop-on-latency=true','tcp-timeout=5000000']
  if extra:args[-2]='drop-on-latency=false'
  args+=['!','rtph264depay','!','h264parse','!','avdec_h264','!','queue','max-size-buffers=1','max-size-bytes=0','max-size-time=0','leaky=downstream','!','videoconvert','!','videoscale','add-borders=true','!','video/x-raw,format=RGB,width=640,height=360,pixel-aspect-ratio=1/1','!','fdsink','fd=1','sync=false']
  f=(out/(name+'.log')).open('wb');client=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=f);start=time.monotonic();received=0;frames=[];size=640*360*3
  try:
   while time.monotonic()-start<35:
    ready,_,_=select.select([client.stdout],[],[],.3)
    if ready:
     data=os.read(client.stdout.fileno(),262144)
     if not data:break
     before=received//size;received+=len(data)
     if received//size>before:frames.extend([round(time.monotonic()-start,4)]*(received//size-before))
   results[name]={'bytes':received,'frames':len(frames),'times':frames,'max_gap_s':max([b-a for a,b in zip(frames,frames[1:])] or [0]),'args':args}
  finally:client.terminate();client.wait(timeout=10);f.close()
finally:
 subprocess.run(ssh+['touch '+remote+'/stop'],check=True);p.wait(timeout=20)
 (out/'result.json').write_text(json.dumps(results,indent=2))
 with (out/'remote-evidence.tar.gz').open('wb') as f:subprocess.run(ssh+['tar -czf - -C /tmp gcs-video-diag'],stdout=f,check=True)
print(json.dumps({k:{a:b for a,b in v.items() if a!='times'} for k,v in results.items()},indent=2))
