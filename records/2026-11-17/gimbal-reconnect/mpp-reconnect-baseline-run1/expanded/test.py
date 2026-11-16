#!/usr/bin/env python3
"""Read-only localhost MPP disconnect/reconnect test with synthetic RTSP sources."""
import csv,json,os,re,signal,subprocess,sys,time
from pathlib import Path
import yaml
out=Path(sys.argv[1]);assert str(out)=='/tmp/mpp-reconnect-check'
root=Path('/home/orangepi/drone_platform');relayroot=root/'03_tools/gimbal_relay'
processes=[];logs=[];checks={};phases=[];completed=False
candidate=out/'candidate-install/lib/rtsp_test/rtsp_hw_pub_ultra'
use_candidate=candidate.exists()
def start(name,args,env=None):
 f=(out/(name+'.log')).open('w');logs.append(f);p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,env=env,start_new_session=True);processes.append(p);return p
def stop(p):
 if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=8)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
def read_rows():
 p=out/'headers.csv'
 if not p.exists():return []
 text=p.read_text();text=text[:text.rfind('\n')+1]
 return [r for r in csv.DictReader(text.splitlines()) if r.get('hash') is not None]
def sample(label,seconds):
 begin=time.monotonic_ns();deadline=time.monotonic()+seconds
 while time.monotonic()<deadline:
  assert reader.poll() is None,'reader exited'
  time.sleep(.05)
 end=time.monotonic_ns();rows=[r for r in read_rows() if begin<=int(r['receipt_ns'])<end]
 phase=dict(name=label,start_ns=begin,end_ns=end,decoder_alive=decoder.poll() is None,streams={})
 for key in ['raw','color']:
  items=[r for r in rows if r['qos']=='reliable' and r['key']==key]
  phase['streams'][key]=dict(messages=len(items),sizes=sorted({(int(r['width']),int(r['height'])) for r in items}))
 phases.append(phase);print(json.dumps(phase),flush=True);return phase
def online(name,p):
 deadline=time.monotonic()+15
 while 'stream is available and online' not in (out/(name+'.log')).read_text():
  assert p.poll() is None and time.monotonic()<deadline,name+' unavailable';time.sleep(.1)
def relay_start(name):
 p=start(name,[str(relayroot/'mediamtx'),str(out/'relay.yml')]);online(name,p);return p
def streaming(phase,min_raw=70):return phase['streams']['raw']['messages']>=min_raw and phase['streams']['color']['messages']>=30
reader=decoder=relay=None
try:
 config=yaml.safe_load((relayroot/'gimbal.yml').read_text());config['rtspAddress']='127.0.0.1:18554';config['rtspTransports']=['tcp'];config['rtpAddress']='127.0.0.1:18000';config['rtcpAddress']='127.0.0.1:18001';config['paths']['gimbal']['source']='rtsp://127.0.0.1:18555/upstream';(out/'relay.yml').write_text(yaml.safe_dump(config))
 config['rtspAddress']='127.0.0.1:18555';config['paths']={'upstream':{'source':'publisher'}};config['authInternalUsers']=[{'user':'any','ips':['127.0.0.1'],'permissions':[{'action':'publish','path':'upstream'},{'action':'read','path':'upstream'}]}];(out/'upstream.yml').write_text(yaml.safe_dump(config))
 for size,label in [('1920x1080','1080'),('1280x720','720')]:
  p=subprocess.run(['ffmpeg','-hide_banner','-loglevel','warning','-f','lavfi','-i','testsrc2=size='+size+':rate=30','-t','3','-an','-c:v','libx264','-preset','ultrafast','-tune','zerolatency','-threads','2','-pix_fmt','yuv420p','-g','30','-y',str(out/(label+'.mp4'))],capture_output=True,timeout=40);(out/('encode-'+label+'.log')).write_bytes(p.stdout+p.stderr);assert p.returncode==0
 upstream=start('upstream',[str(relayroot/'mediamtx'),str(out/'upstream.yml')]);time.sleep(.5)
 def publish(label):
  return start('publisher-'+label,['ffmpeg','-hide_banner','-loglevel','warning','-re','-stream_loop','-1','-i',str(out/(label+'.mp4')),'-an','-c:v','copy','-f','rtsp','-rtsp_transport','tcp','rtsp://127.0.0.1:18555/upstream'])
 publisher=publish('1080');online('upstream',upstream)
 env=dict(os.environ,ROS_DOMAIN_ID='80',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(out/'ros-log'))
 for name in ['FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE']:env.pop(name,None)
 reader=start('observer',[str(out/'observer-build/observer'),'--ros-args','-p','output:='+str(out/'headers.csv'),'-p','stop_file:='+str(out/'observer.stop')],env)
 if not use_candidate:relay=relay_start('relay-initial')
 decoder_env=dict(env)
 if use_candidate:decoder_env['AMENT_PREFIX_PATH']=str(out/'candidate-install')+':'+decoder_env.get('AMENT_PREFIX_PATH','')
 executable=str(candidate if use_candidate else root/'01_apps/mission_demo/workspace/install/rtsp_test/lib/rtsp_test/rtsp_hw_pub_ultra')
 args=[executable,'--ros-args','-p','rtsp_url:=rtsp://127.0.0.1:18554/gimbal','--log-level','rtsp_hw_pub_ultra.frames:=debug']
 if use_candidate:args+=['-p','connect_timeout_s:=1.0','-p','read_timeout_s:=1.0','-p','reconnect_delay_s:=0.25','-p','frame_timeout_s:=2.0']
 decoder=start('mpp',args,decoder_env);decoder_pid=decoder.pid
 if use_candidate:
  p=sample('startup unavailable',3);checks['stays_alive_without_initial_rtsp']=p['decoder_alive'] and p['streams']['raw']['messages']==0;relay=relay_start('relay-initial')
 p=sample('initial stream',9);checks['initial_stream']=streaming(p) and p['decoder_alive']
 stop(relay);p=sample('closed source',4);checks['alive_while_source_closed']=p['decoder_alive'];checks['no_stale_frames_when_closed']=p['streams']['raw']['messages']<10
 relay=relay_start('relay-restored');p=sample('stream restored',12);checks['reconnect_after_closed_connection']=streaming(p) and p['decoder_alive']
 if use_candidate:
  os.killpg(relay.pid,signal.SIGSTOP)
  try:p=sample('open TCP but stalled',5);checks['alive_during_stalled_tcp']=p['decoder_alive'];checks['no_repeated_frames_when_stalled']=p['streams']['raw']['messages']<10
  finally:os.killpg(relay.pid,signal.SIGCONT)
  p=sample('stalled TCP resumed',9);checks['reconnect_after_stalled_tcp']=streaming(p) and p['decoder_alive']
  stop(relay);stop(publisher);publisher=publish('720');time.sleep(1);relay=relay_start('relay-720')
  p=sample('resolution changed',10);checks['reconnect_with_new_resolution']=streaming(p) and (1280,720) in [tuple(v) for v in p['streams']['raw']['sizes']]
  checks['same_decoder_process']=decoder.pid==decoder_pid and decoder.poll() is None
  # SIGINT must also break an RTSP read while TCP is connected but silent.
  os.killpg(relay.pid,signal.SIGSTOP)
  try:
   time.sleep(.4);at=time.monotonic();stop(decoder);checks['bounded_shutdown_during_stall']=decoder.returncode==0 and time.monotonic()-at<3
  finally:os.killpg(relay.pid,signal.SIGCONT)
 completed=True
finally:
 if relay and relay.poll() is None:stop(relay)
 if decoder and decoder.poll() is None:stop(decoder)
 time.sleep(.5)
 if reader and reader.poll() is None:(out/'observer.stop').touch();reader.wait(timeout=10)
 for p in reversed(processes):
  if p.poll() is None:stop(p)
 for f in logs:f.close()
 rows=read_rows();trace=(out/'mpp.log').read_text() if (out/'mpp.log').exists() else '';published={key:[int(x) for x in re.findall('published_'+key+r' stamp_ns=([0-9]+)',trace)] for key in ['raw','color']}
 for qos in ['reliable','best_effort']:
  for key in ['raw','color']:
   items=[r for r in rows if r['qos']==qos and r['key']==key];stamps=[int(r['stamp_ns']) for r in items];ns=qos+'_'+key
   checks[ns+'_all_source_stamps']=bool(items) and len(stamps)==len(set(stamps))==len(published[key]) and set(stamps)==set(published[key])
   checks[ns+'_headers_and_buffers']=bool(items) and all(int(r['bytes'])==int(r['step'])*int(r['height']) and r['encoding']==('mono8' if key=='raw' else 'bgr8') and r['frame']=='gimbal_camera_optical_frame' for r in items)
   checks[ns+'_stamps_increase']=all(b>a for a,b in zip(stamps,stamps[1:]))
 checks['color_uses_decoded_frame_stamp']=bool(published['color']) and set(published['color'])<=set(published['raw'])
 checks['native_observer_clean_exit']=reader is not None and reader.returncode==0
 result=dict(passed=completed and all(checks.values()),completed=completed,candidate=use_candidate,checks=checks,phases=phases,source_counts={k:len(v) for k,v in published.items()},returncodes=[p.returncode for p in processes],scope='synthetic source, actual MPP and native subscribers, localhost domain80; no flight controller or physical gimbal commands')
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(passed=result['passed'],checks=checks)),flush=True)
