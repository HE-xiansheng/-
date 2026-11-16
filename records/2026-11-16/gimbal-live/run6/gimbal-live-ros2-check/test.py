#!/usr/bin/env python3
"""Read-only real camera stream through localhost SSH tunnel -> relay -> MPP."""
import csv,hashlib,json,os,re,signal,subprocess,time
from pathlib import Path
import yaml,rclpy
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
assert os.environ['ROS_DOMAIN_ID']=='79' and os.environ['ROS_LOCALHOST_ONLY']=='1'
out=Path('/tmp/gimbal-live-ros2-check');out.mkdir(exist_ok=True)
root=Path('/home/orangepi/drone_platform/03_tools/gimbal_relay')
source='rtsp://127.0.0.1:18655/gimbal'
probe=subprocess.run(['ffprobe','-v','error','-rtsp_transport','tcp','-select_streams','v:0','-show_entries','stream=codec_name,width,height,avg_frame_rate','-of','json',source],text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=25)
(out/'ffprobe.json').write_text(probe.stdout);(out/'ffprobe.stderr').write_text(probe.stderr);assert probe.returncode==0
video=json.loads(probe.stdout)['streams'][0];assert video['codec_name']=='h264'
config=yaml.safe_load((root/'gimbal.yml').read_text());config['rtspAddress']='127.0.0.1:18554';config['rtspTransports']=['tcp'];config['rtpAddress']='127.0.0.1:18000';config['rtcpAddress']='127.0.0.1:18001';config['paths']['gimbal']['source']=source
(out/'relay.yml').write_text(yaml.safe_dump(config))
procs=[];files=[];rows={};checks={};completed=False

def start(name,cmd):
 f=(out/(name+'.log')).open('w');files.append(f)
 p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p);return p

rclpy.init();node=rclpy.create_node('gimbal_live_readonly_check');subs=[]
def receive(key,msg):
 row=rows.setdefault(key,{'times':[],'stamps':[],'ages':[],'hashes':set()})
 ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
 row['times'].append(time.monotonic());row['stamps'].append(ns);row['ages'].append((node.get_clock().now().nanoseconds-ns)/1e9)
 row.update(width=msg.width,height=msg.height,encoding=msg.encoding,frame=msg.header.frame_id,step=msg.step,bytes=len(msg.data))
 if len(row['times'])%10==0:row['hashes'].add(hashlib.sha256(bytes(msg.data[::1024])).hexdigest())
for key in ['raw','color']:subs.append(node.create_subscription(Image,'/rtsp_cam/image_'+key,lambda m,k=key:receive(k,m),1))
try:
 start('relay',['bash',str(root/'start_relay.sh'),str(out/'relay.yml')])
 deadline=time.monotonic()+20
 while 'stream is available and online' not in (out/'relay.log').read_text():
  assert time.monotonic()<deadline and all(p.poll() is None for p in procs),'relay unavailable'
  time.sleep(.1)
 print('RELAY_READY',flush=True)
 packet=subprocess.run(['python3',str(root/'check_rtsp.py'),'rtsp://127.0.0.1:18554/gimbal'],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=18)
 (out/'rtp.txt').write_text(packet.stdout);assert packet.returncode==0,packet.stdout
 rtp=json.loads(packet.stdout);checks['live_h264_packets']=rtp['fps']>20
 native=start('native-observer',['/tmp/gimbal-native-observer/build/observer'])
 start('mpp',['ros2','run','rtsp_test','rtsp_hw_pub_ultra','--ros-args','-p','rtsp_url:=rtsp://127.0.0.1:18554/gimbal','--log-level','rtsp_hw_pub_ultra.frames:=debug'])
 end=time.monotonic()+35
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.05)
  assert all(p.poll() is None for p in procs),'owned process exited'
 for key,minimum in [('raw',20),('color',12)]:
  row=rows[key];row['hz']=(len(row['times'])-1)/(row['times'][-1]-row['times'][0]);checks[key+'_rate']=row['hz']>=minimum
  checks[key+'_stamps']=all(b>a for a,b in zip(row['stamps'],row['stamps'][1:])) and all(-.1<a<1 for a in row['ages'])
  checks[key+'_changing_pixels']=len(row['hashes'])>1
  checks[key+'_buffer_geometry']=row['bytes']==row['step']*row['height']
 checks['formats_match_real_source']=rows['raw']['encoding']=='mono8' and (rows['raw']['width'],rows['raw']['height'])==(video['width'],video['height']) and rows['color']['encoding']=='bgr8' and rows['color']['width']==min(960,video['width'])
 checks['optical_frames']=all(row['frame']=='gimbal_camera_optical_frame' for row in rows.values())
 matched=len(set(rows['color']['stamps'])&set(rows['raw']['stamps']))/len(rows['color']['stamps'])
 (out/'source-stamp-matches.json').write_text(json.dumps({'fraction':matched,'raw_stamps':rows['raw']['stamps'],'color_stamps':rows['color']['stamps']},indent=2))
 os.killpg(native.pid,signal.SIGINT);native.wait(timeout=8)
 native_rows=list(csv.DictReader((out/'native-headers.csv').open()))
 native_stamps={key:{int(row['stamp_ns']) for row in native_rows if row['key']==key} for key in ['raw','color']}
 native_match=len(native_stamps['raw']&native_stamps['color'])/max(1,len(native_stamps['color']))
 native_summary={'python_match_fraction':matched,'native_match_fraction':native_match,'streams':{}}
 for key in ['raw','color']:
  received=[int(row['receipt_ns']) for row in native_rows if row['key']==key]
  hz=(len(received)-1)*1e9/(received[-1]-received[0]) if len(received)>1 else 0
  native_summary['streams'][key]={'messages':len(received),'hz':hz}
 checks['native_observer_raw_rate']=native_summary['streams']['raw']['hz']>25
 checks['native_observer_color_rate']=native_summary['streams']['color']['hz']>13
 trace=(out/'mpp.log').read_text()
 published={key:[int(value) for value in re.findall('published_'+key+r' stamp_ns=([0-9]+)',trace)] for key in ['raw','color']}
 checks['publisher_source_header_trace']=len(published['raw'])>500 and len(published['color'])>200
 checks['color_retains_decoded_frame_stamp']=bool(published['color']) and set(published['color']).issubset(set(published['raw']))
 for key in ['raw','color']:
  observed=native_stamps[key];emitted=set(published[key])
  window={stamp for stamp in emitted if observed and min(observed)<=stamp<=max(observed)}
  native_summary['streams'][key].update(published_in_observer_window=len(window),missing_received_frames=len(window-observed),published_unique=len(emitted))
  checks[key+'_received_stamps_match_publisher_trace']=bool(observed) and observed.issubset(emitted)
  checks[key+'_published_stamps_unique']=len(published[key])==len(emitted)
 (out/'native-comparison.json').write_text(json.dumps(native_summary,indent=2)+'\n')
 completed=True
finally:
 for p in reversed(procs):
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 for f in files:f.close()
 checks['normal_shutdown']=all(p.returncode==0 for p in procs)
 result={'source_probe':video,'checks':checks,'streams':{k:{n:v for n,v in row.items() if n not in ['times','stamps','ages','hashes']}|{'messages':len(row['times']),'different_sample_hashes':len(row['hashes']),'age_min_s':min(row['ages']),'age_max_s':max(row['ages'])} for k,row in rows.items()},'shutdown_codes':[p.returncode for p in procs],'passed':completed and all(checks.values()),'scope':'actual camera via .235 read-only relay and temporary SSH tunnels; .224 physical camera network not validated; ROS stamps are decoder receipt time'}
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
 node.destroy_node();rclpy.shutdown()
assert result['passed']
