#!/usr/bin/env python3
"""Computer-owned concurrent check, optionally using physical radar and gimbal."""
from pathlib import Path
import hashlib,json,os,subprocess,sys,tarfile,uuid
repo=Path(__file__).resolve().parents[1]
out=Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=False)
remote='/tmp/platform-soak-check'
physical=os.environ.get('PLATFORM_SOAK_PHYSICAL')=='1'
seconds=int(os.environ.get('PLATFORM_SOAK_SECONDS','180'))
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ServerAliveInterval=15','-o','ServerAliveCountMax=3','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
created=network_created=False
diagnostics=[];diagnostic_logs=[]
original=''
profile='ROS2-Gimbal-soak-'+str(uuid.uuid4())[:8]
result={'passed':False,'physical_inputs':physical,'cleanup_errors':[]}
def run(command,name,timeout=25,check=True):
 if name=='runner':
  with (out/(name+'.log')).open('w') as log:p=subprocess.run(ssh+[command],stdout=log,stderr=subprocess.STDOUT,text=True,timeout=timeout)
 else:
  p=subprocess.run(ssh+[command],capture_output=True,text=True,timeout=timeout)
  (out/(name+'.log')).write_text(p.stdout+p.stderr)
 if check:p.check_returncode()
 return p
try:
 run('test ! -e '+remote+' && git -C /home/orangepi/drone_platform status --porcelain && date -Is','before')
 active=run('pgrep -af "[l]ivox_ros_driver2_node|[f]astlio_mapping|[v]ision2mavros|[o]rbbec_camera_node|[r]tsp_hw_pub_ultra|[m]ediamtx|[p]erson_preview|[p]lanner_node" || true','processes-before')
 if active.stdout.strip():raise RuntimeError('refusing to overlap active processing')
 if physical:
  mac=run('cat /sys/class/net/enx00e09a5af420/address','gimbal-identity').stdout.strip()
  if mac!='00:e0:9a:5a:f4:20':raise RuntimeError('physical gimbal NIC differs')
  original=run('nmcli -g GENERAL.CON-UUID device show enx00e09a5af420','original-profile').stdout.strip()
  run('nmcli connection add save no type ethernet ifname enx00e09a5af420 con-name '+profile+' connection.autoconnect no 802-3-ethernet.mac-address 00:E0:9A:5A:F4:20 ipv4.method manual ipv4.addresses 192.168.144.100/24 ipv4.never-default yes ipv4.ignore-auto-dns yes ipv4.dad-timeout 3000 ipv4.may-fail no ipv6.method disabled','network-create')
  network_created=True
  run('nmcli -w 15 connection up id '+profile+' ifname enx00e09a5af420','network-up')
  routes=run('ip route get 192.168.144.108; ip route get 192.168.1.158; ip route get 1.1.1.1','network-active').stdout
  if ('192.168.144.108 dev enx00e09a5af420 src 192.168.144.100' not in routes or '192.168.1.158 dev enP3p49s0 src 192.168.1.51' not in routes or '1.1.1.1 via 192.168.1.1 dev wlan0' not in routes):raise RuntimeError('unexpected sensor/management routes')
 inputs={}
 for n in ['record_session.py','recording_manage.py','recording_auto.py']:inputs['recording-source/'+n]=repo/'platform/01_apps/ego_planner'/n
 for n in ['test_platform_soak_ros2.py','validate_fastlio_replay.py','analyze_soak_preview_headers.py','cleanup_livox_live_check.py']:inputs[n]=repo/'tools'/n
 for p in (repo/'tools/gimbal_delivery_observer').iterdir():
  if p.is_file():inputs['observer-src/'+p.name]=p
 if physical:inputs['sensor_readiness.py']=repo/'platform/01_apps/fast_lio/sensor_readiness.py'
 else:inputs['livox.jsonl.gz']=Path('/home/tyzk/drone-migration-work/reference/livox235-sample.jsonl.gz')
 if os.environ.get('PLATFORM_SOAK_CAMERA_LAUNCH_CANDIDATE')=='1':
  for name in ['launch/gemini335l.launch.py','config/gemini335l.yaml','config/fastdds_large_images.xml']:inputs['camera-candidate/'+name]=repo/'platform/02_drivers/orbbec'/name
 (out/'inputs-manifest.json').write_text(json.dumps({name:hashlib.sha256(p.read_bytes()).hexdigest() for name,p in inputs.items()},indent=2)+'\n')
 with tarfile.open(out/'inputs.tar','w') as a:
  for name,p in inputs.items():a.add(p,arcname=name)
 run('mkdir '+remote,'create');created=True
 with (out/'inputs.tar').open('rb') as f:subprocess.run(ssh+['tar -xf - -C '+remote],stdin=f,check=True,timeout=25)
 if physical:
  log=(out/'kernel-live.log').open('w');diagnostic_logs.append(log)
  diagnostics.append(subprocess.Popen(ssh+['timeout --signal=TERM '+str(seconds+220)+'s journalctl -k -b -f -n 0 -o short-iso --no-pager'],stdout=log,stderr=subprocess.STDOUT,text=True,start_new_session=True))
 options={'ROS_DOMAIN_ID':'83','ROS_LOCALHOST_ONLY':'1','ROS_LOG_DIR':remote+'/ros-log','PLATFORM_SOAK_SECONDS':str(seconds)}
 for key in ['PLATFORM_SOAK_PHYSICAL','PLATFORM_SOAK_GCS','PLATFORM_SOAK_CAMERA_PROFILE','PLATFORM_SOAK_DISABLE_SELF_SYNC_DIAG']:
  if os.environ.get(key)=='1':options[key]='1'
 if os.environ.get('PLATFORM_SOAK_CAMERA_LAUNCH_CANDIDATE')=='1':options['PLATFORM_SOAK_CAMERA_LAUNCH']=remote+'/camera-candidate/launch/gemini335l.launch.py'
 setup='source /home/orangepi/drone_platform/env.sh && '
 if os.environ.get('PLATFORM_SOAK_FASTLIO_CANDIDATE')=='1':setup+='export AMENT_PREFIX_PATH=/tmp/fastlio-sync-candidate/install:$AMENT_PREFIX_PATH LD_LIBRARY_PATH=/tmp/fastlio-sync-candidate/install/lib:$LD_LIBRARY_PATH && '
 cmd=setup+'cmake -S '+remote+'/observer-src -B '+remote+'/observer-build && cmake --build '+remote+'/observer-build -j2 && '+' '.join(k+'='+v for k,v in options.items())+' timeout --signal=INT --kill-after=25s '+str(seconds+160)+'s python3 -u '+remote+'/test_platform_soak_ros2.py '+remote
 p=run(cmd,'runner',timeout=seconds+210,check=False);result['returncode']=p.returncode
except Exception as exc:
 result['error']=repr(exc)
finally:
 if created:
  try:
   run('python3 '+remote+'/cleanup_livox_live_check.py '+remote,'owned-cleanup')
   result['owned_processes_cleaned']=True
   with (out/'evidence.tar.gz').open('wb') as f:subprocess.run(ssh+['tar -I "gzip -1" -cf - -C /tmp platform-soak-check'],stdout=f,check=True,timeout=60)
   subprocess.run(['tar','-xzf',str(out/'evidence.tar.gz'),'-C',str(out)],check=True,timeout=30)
   result['evidence_retrieved']=True
   run('rm -rf -- '+remote,'remove');result['remote_removed']=True
  except Exception as exc:result['cleanup_errors'].append(repr(exc))
 if network_created:
  steps=[('nmcli -w 5 connection down id '+profile,'network-down'),('nmcli connection delete id '+profile,'network-delete')]
  if original and original!='--':steps.append(('nmcli -w 0 connection up uuid '+original+' ifname enx00e09a5af420','network-restore'))
  for cmd,name in steps:
   try:run(cmd,name)
   except Exception as exc:result['cleanup_errors'].append(repr(exc))
 try:run('date -Is; ip -brief address; ip route get 192.168.1.158; ip route get 1.1.1.1; git -C /home/orangepi/drone_platform status --porcelain; systemctl is-active nxserver','final-state')
 except Exception as exc:result['cleanup_errors'].append(repr(exc))
 for process in diagnostics:
  if process.poll() is None:process.terminate()
  try:process.wait(timeout=8)
  except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
 for log in diagnostic_logs:log.close()
 result['diagnostic_process_returncodes']=[process.returncode for process in diagnostics]
 report=out/'platform-soak-check/result.json'
 result['passed']=bool(report.exists() and json.loads(report.read_text()).get('passed') is True and result.get('returncode')==0 and result.get('owned_processes_cleaned') and not result['cleanup_errors'] and 'error' not in result)
 (out/'runner-result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result));raise SystemExit(0 if result['passed'] else 1)
