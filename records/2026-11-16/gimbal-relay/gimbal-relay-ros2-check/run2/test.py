#!/usr/bin/env python3
"""Local-only synthetic H264 source -> actual relay -> MPP ROS2 images."""
import json,os,signal,socket,subprocess,time
from pathlib import Path
import yaml
import rclpy
from sensor_msgs.msg import Image
assert os.environ['ROS_DOMAIN_ID']=='80' and os.environ['ROS_LOCALHOST_ONLY']=='1'
out=Path(os.environ['GIMBAL_TEST_OUTPUT']);out.mkdir(parents=True,exist_ok=True)
root=Path('/home/orangepi/drone_platform/03_tools/gimbal_relay')
production=yaml.safe_load((root/'gimbal.yml').read_text())
relay=json.loads(json.dumps(production));relay['rtspAddress']='127.0.0.1:18554';relay['rtspTransports']=['tcp']
relay['paths']['gimbal']['source']='rtsp://127.0.0.1:18555/upstream'
relay['rtpAddress']='127.0.0.1:18000';relay['rtcpAddress']='127.0.0.1:18001'
(out/'relay.yml').write_text(yaml.safe_dump(relay))
source=json.loads(json.dumps(relay));source['rtspAddress']='127.0.0.1:18555';source['paths']={'upstream':{'source':'publisher'}}
source['authInternalUsers']=[{'user':'any','ips':['127.0.0.1'],'permissions':[{'action':'publish','path':'upstream'},{'action':'read','path':'upstream'}]}]
(out/'source.yml').write_text(yaml.safe_dump(source))
procs=[];handles=[];records={};checks={};completed=False

def start(name,cmd):
 f=(out/(name+'.log')).open('w');handles.append(f)
 p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p);return p

def wait_port(port):
 deadline=time.monotonic()+8
 while time.monotonic()<deadline:
  try:
   with socket.create_connection(('127.0.0.1',port),.3):return
  except OSError:time.sleep(.2)
 raise RuntimeError('listener timeout '+str(port))

rclpy.init();node=rclpy.create_node('gimbal_relay_ground_check');subs=[]
def receive(key,m):
 row=records.setdefault(key,{'times':[],'stamps':[],'ages':[]})
 ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
 row['times'].append(time.monotonic());row['stamps'].append(ns);row['ages'].append((node.get_clock().now().nanoseconds-ns)/1e9)
 row.update(width=m.width,height=m.height,encoding=m.encoding,frame=m.header.frame_id)
for k in ['raw','color']:
 subs.append(node.create_subscription(Image,'/rtsp_cam/image_'+k,lambda m,k=k:receive(k,m),1))
try:
 start('upstream',[str(root/'mediamtx'),str(out/'source.yml')]);wait_port(18555)
 start('publisher',['ffmpeg','-hide_banner','-loglevel','warning','-re','-f','lavfi','-i','testsrc2=size=1920x1080:rate=15','-an','-c:v','libx264','-preset','ultrafast','-tune','zerolatency','-threads','2','-pix_fmt','yuv420p','-g','15','-f','rtsp','-rtsp_transport','tcp','rtsp://127.0.0.1:18555/upstream'])
 deadline=time.monotonic()+15
 while 'stream is available and online' not in (out/'upstream.log').read_text():
  assert time.monotonic()<deadline and all(p.poll() is None for p in procs),'upstream not ready'
  time.sleep(.1)
 start('relay',['bash',str(root/'start_relay.sh'),str(out/'relay.yml')]);wait_port(18554)
 deadline=time.monotonic()+15
 while 'stream is available and online' not in (out/'relay.log').read_text():
  assert time.monotonic()<deadline and all(p.poll() is None for p in procs),'relay not ready'
  time.sleep(.1)
 print('RELAY_READY',flush=True)
 probe=subprocess.run(['/usr/bin/python3',str(root/'check_rtsp.py'),'rtsp://127.0.0.1:18554/gimbal'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=18)
 (out/'rtp-check.txt').write_text(probe.stdout);assert probe.returncode==0,probe.stdout
 packets=json.loads(probe.stdout);checks['actual_relay_h264_packets']=packets['frame_markers']>=50
 start('mpp',['ros2','run','rtsp_test','rtsp_hw_pub_ultra','--ros-args','-p','rtsp_url:=rtsp://127.0.0.1:18554/gimbal'])
 end=time.monotonic()+35
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.05)
  assert all(p.poll() is None for p in procs),'owned process exited'
 for k in ['raw','color']:
  row=records[k];row['hz']=(len(row['times'])-1)/(row['times'][-1]-row['times'][0])
  checks[k+'_rate']=row['hz']>10
  checks[k+'_source_stamps']=all(b>a for a,b in zip(row['stamps'],row['stamps'][1:])) and all(-.1<a<1 for a in row['ages'])
 checks['image_formats']=records['raw']['encoding']=='mono8' and records['raw']['width']==1920 and records['color']['encoding']=='bgr8' and records['color']['width']==960
 checks['optical_frames']=all(v['frame']=='gimbal_camera_optical_frame' for v in records.values())
 completed=True
finally:
 for p in reversed(procs):
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 for f in handles:f.close()
 result={'checks':checks,'streams':{k:{n:v for n,v in row.items() if n not in ['times','stamps','ages']}|{'messages':len(row['times']),'age_min_s':min(row['ages']),'age_max_s':max(row['ages'])} for k,row in records.items()},'shutdown_codes':[p.returncode for p in procs],'passed':completed and all(checks.values()),'scope':'synthetic source, localhost relay and real RK3588 MPP decoder; timestamps are decoder receipt times'}
 checks['shutdown']=all(p.returncode in (0,255) for p in procs);result['passed']=result['passed'] and checks['shutdown']
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
 node.destroy_node();rclpy.shutdown()
assert result['passed']
