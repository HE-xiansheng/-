#!/usr/bin/env python3
"""Compare default/large SHM using actual MPP, synthetic 1080p30 H264 and native readers."""
import csv,json,os,re,signal,socket,subprocess,sys,time
from pathlib import Path
import yaml
out=Path(sys.argv[1]);assert str(out)=='/tmp/gimbal-delivery-check'
root=Path('/home/orangepi/drone_platform');relayroot=root/'03_tools/gimbal_relay'
processes=[];logs=[];rounds=[]
def start(name,args,env=None):
 f=(out/(name+'.log')).open('w');logs.append(f)
 p=subprocess.Popen(args,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
def stop(p):
 if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=8)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
def wait_online(name,p):
 deadline=time.monotonic()+18
 while 'stream is available and online' not in (out/(name+'.log')).read_text():
  assert p.poll() is None and time.monotonic()<deadline,name+' not online';time.sleep(.1)
try:
 # Encode before measuring, then replay with stream copy so encoder load is absent.
 p=subprocess.run(['ffmpeg','-hide_banner','-loglevel','warning','-f','lavfi','-i','testsrc2=size=1920x1080:rate=30','-t','3','-an','-c:v','libx264','-preset','ultrafast','-tune','zerolatency','-threads','2','-pix_fmt','yuv420p','-g','30','-y',str(out/'synthetic.mp4')],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=40)
 (out/'encode.log').write_bytes(p.stdout);assert p.returncode==0
 config=yaml.safe_load((relayroot/'gimbal.yml').read_text());config['rtspAddress']='127.0.0.1:18554';config['rtspTransports']=['tcp'];config['rtpAddress']='127.0.0.1:18000';config['rtcpAddress']='127.0.0.1:18001';config['paths']['gimbal']['source']='rtsp://127.0.0.1:18555/upstream'
 (out/'relay.yml').write_text(yaml.safe_dump(config));source=json.loads(json.dumps(config));source['rtspAddress']='127.0.0.1:18555';source['paths']={'upstream':{'source':'publisher'}}
 source['authInternalUsers']=[{'user':'any','ips':['127.0.0.1'],'permissions':[{'action':'publish','path':'upstream'},{'action':'read','path':'upstream'}]}]
 (out/'upstream.yml').write_text(yaml.safe_dump(source))
 upstream=start('upstream',[str(relayroot/'mediamtx'),str(out/'upstream.yml')]);time.sleep(.4)
 publisher=start('publisher',['ffmpeg','-hide_banner','-loglevel','warning','-re','-stream_loop','-1','-i',str(out/'synthetic.mp4'),'-an','-c:v','copy','-f','rtsp','-rtsp_transport','tcp','rtsp://127.0.0.1:18555/upstream']);wait_online('upstream',upstream)
 for label in ['shm64','default','publisher_shm64']:
  relay=start('relay',['bash',str(relayroot/'start_relay.sh'),str(out/'relay.yml')]);wait_online('relay',relay)
  env=dict(os.environ,ROS_DOMAIN_ID='80',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(out/(label+'-ros-log')))
  for key in ['FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','RMW_FASTRTPS_USE_QOS_FROM_XML']:env.pop(key,None)
  reader_env=dict(env)
  if label in ['shm64','publisher_shm64']:env['FASTRTPS_DEFAULT_PROFILES_FILE']=str(out/'observer-src/shm64.xml')
  if label=='shm64':reader_env=dict(env)
  reader=start(label+'-observer',[str(out/'observer-build/observer'),'--ros-args','-p','output:='+str(out/(label+'-headers.csv')),'-p','stop_file:='+str(out/(label+'-observer.stop'))],reader_env)
  decoder=start(label+'-mpp',['ros2','run','rtsp_test','rtsp_hw_pub_ultra','--ros-args','-p','rtsp_url:=rtsp://127.0.0.1:18554/gimbal','--log-level','rtsp_hw_pub_ultra.frames:=debug'],env)
  deadline=time.monotonic()+26
  while time.monotonic()<deadline:
   assert all(p.poll() is None for p in [upstream,publisher,relay,reader,decoder]),'child exited'
   time.sleep(.1)
  (out/(label+'-shm-files.json')).write_text(json.dumps({p.name:p.stat().st_size for p in Path('/dev/shm').glob('fastrtps*')},indent=2)+'\n')
  # End this owned stream before shutting down the decoder context; drain DDS.
  stop(relay);time.sleep(2)
  stop(decoder);time.sleep(1);(out/(label+'-observer.stop')).touch();reader.wait(timeout=10)
  for f in logs:f.flush()
  rows=list(csv.DictReader((out/(label+'-headers.csv')).open()));assert all(None not in row and None not in row.values() for row in rows),'truncated CSV';trace=(out/(label+'-mpp.log')).read_text()
  published={key:[int(v) for v in re.findall('published_'+key+r' stamp_ns=([0-9]+)',trace)] for key in ['raw','color']}
  checks={'owned_decoder_reader_exit':decoder.returncode==0 and reader.returncode==0,'no_xml_errors':'XMLPARSER Error' not in trace}
  streams={}
  for qos in ['reliable','best_effort']:
   for key in ['raw','color']:
    name=qos+'_'+key;items=[r for r in rows if r['qos']==qos and r['key']==key];stamps=[int(r['stamp_ns']) for r in items];emitted=published[key]
    missing=sorted(set(emitted)-set(stamps));extras=sorted(set(stamps)-set(emitted));times=[int(r['receipt_ns']) for r in items]
    hz=(len(times)-1)*1e9/(times[-1]-times[0]) if len(times)>1 else 0
    streams[name]=dict(published=len(emitted),received=len(stamps),missing=len(missing),extra=len(extras),missing_stamps=missing,hz=hz,max_age_ms=max((int(r['age_ns'])/1e6 for r in items),default=None))
    checks[name+'_all_published_received']=bool(stamps) and not missing and not extras and len(stamps)==len(set(stamps))==len(emitted)
    checks[name+'_rate']=hz>(28 if key=='raw' else 14)
    checks[name+'_geometry']=bool(items) and all((int(r['width']),int(r['height']),r['encoding'],int(r['bytes']))==((1920,1080,'mono8',1920*1080) if key=='raw' else (960,540,'bgr8',960*540*3)) and r['frame']=='gimbal_camera_optical_frame' for r in items)
    checks[name+'_changing_pixels']=len({r['hash'] for r in items})>10
    checks[name+'_source_stamp_order']=all(b>a for a,b in zip(stamps,stamps[1:]))
  checks['full_source_frames']=len(published['raw'])>500 and len(published['color'])>250
  checks['color_source_stamps_preserved']=bool(published['color']) and set(published['color'])<=set(published['raw'])
  result=dict(profile=label,passed=all(checks.values()),checks=checks,streams=streams)
  (out/(label+'-result.json')).write_text(json.dumps(result,indent=2)+'\n');rounds.append(result);print(json.dumps(result),flush=True)
finally:
 for p in reversed(processes):
  if p.poll() is None:stop(p)
 for f in logs:f.close()
 (out/'result.json').write_text(json.dumps(dict(completed=len(rounds)==3,rounds=rounds,child_returncodes=[p.returncode for p in processes],scope='synthetic 1080p30 H264 -> real relay/MPP -> native DDS observers; source stamps are decode receipt times'),indent=2)+'\n')
