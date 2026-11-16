#!/usr/bin/env python3
"""Exercise full production GCS against concurrent, isolated native ROS2 inputs."""
from pathlib import Path
import hashlib,json,os,shlex,shutil,signal,socket,subprocess,sys,time
repo=Path(__file__).resolve().parents[1];out=Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=False)
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ExitOnForwardFailure=yes','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
def free_port():
 with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:s.bind(('0.0.0.0',0));return s.getsockname()[1]
def stop(p):
 if p and p.poll() is None:
  os.killpg(p.pid,signal.SIGTERM)
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
fixture=None;tunnel=None;qt=None;handles=[];result={};env=None
try:
 inputs=[repo/'groundstation/main.cpp',repo/'groundstation/videocontroller.cpp',repo/'tools/gcs_concurrent/automation.cpp',repo/'tools/gcs_concurrent/gst-wrapper.py',Path('/home/tyzk/drone-migration-work/build-gcs-concurrent/gcs/customgeometry')]
 (out/'inputs-manifest.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},indent=2)+'\n')
 shim=out/'shims';shim.mkdir()
 for src,name in [('ssh-wrapper.py','ssh'),('gst-wrapper.py','gst-launch-1.0')]:shutil.copy2(repo/'tools/gcs_concurrent'/src,shim/name);(shim/name).chmod(0o700)
 # Test relay uses an independent port, with read permission only for this PC.
 fixture_env=dict(os.environ,PLATFORM_SOAK_GCS='1',PLATFORM_SOAK_SECONDS='120')
 f=(out/'fixture-runner.log').open('w');handles.append(f)
 fixture=subprocess.Popen([sys.executable,str(repo/'tools/run_platform_soak_check.py'),str(out/'fixture')],env=fixture_env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
 log=out/'fixture/runner.log';deadline=time.monotonic()+90
 while time.monotonic()<deadline:
  assert fixture.poll() is None,'fixture stopped'
  if log.exists() and '"all_actual_processing_streams_ready": true' in log.read_text():break
  time.sleep(.25)
 else:raise RuntimeError('fixture readiness timeout')
 qt_root='/home/tyzk/dianyunGCS-review-20261026/.deps/Qt';bundle_lib='/home/tyzk/drone-migration-work/portable relocation test3/drone-gcs-ros2-20261116-22c9b52-r2-linux-amd64/lib';ports=[free_port() for _ in range(4)]
 env=dict(os.environ,PATH=str(shim)+':'+os.environ['PATH'],LD_LIBRARY_PATH=bundle_lib+':'+qt_root+'/lib',QML_IMPORT_PATH=qt_root+'/qml',GCS_STATE_DIR=str(out/'state'),GCS_TEST_KEYS='/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts',GCS_TEST_SSH_CALLS=str(out/'ssh-calls.jsonl'),GCS_CONCURRENT_RESULT=str(out/'qt-result.json'),GCS_RAW_UDP_PORT=str(ports[3]),XDG_CONFIG_HOME=str(out/'config'),XDG_DATA_HOME=str(out/'data'),XDG_CACHE_HOME=str(out/'cache'),LIBGL_ALWAYS_SOFTWARE='1',NO_PROXY='127.0.0.1',no_proxy='127.0.0.1')
 (out/'ports.json').write_text(json.dumps(ports))
 env['GCS_GST_DIAG']=str(out/'gstreamer-sink.log')
 env.pop('QSG_NO_VSYNC',None)
 env['QSG_INFO']='1'
 (out/'render-launch-settings.json').write_text(json.dumps({'QSG_NO_VSYNC_present_in_launcher':'QSG_NO_VSYNC' in env,'LIBGL_ALWAYS_SOFTWARE':env.get('LIBGL_ALWAYS_SOFTWARE'),'QSG_INFO':env.get('QSG_INFO')},indent=2)+'\n')
 f=(out/'qt.log').open('w');handles.append(f)
 qt=subprocess.Popen(['/home/tyzk/drone-migration-work/build-gcs-concurrent/gcs/customgeometry','--no-splash','--host','192.168.1.224','--cloud-port',str(ports[0]),'--pose-port',str(ports[1]),'--preflight-port',str(ports[2])],env=env,cwd=out,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
 qt.wait(timeout=210)
 result['qt_exit']=qt.returncode
 if qt.returncode:
  # Failed UI checks still need bounded test cleanup; this is not a UI stop pass.
  code="import json,time;from pathlib import Path;Path('/tmp/platform-soak-check/gcs-abort.json').write_text(json.dumps({'qt_exit':"+str(qt.returncode)+",'time_ns':time.time_ns()}))"
  subprocess.run(ssh+['python3 -c '+shlex.quote(code)],check=True,timeout=12)
 # The fixture owns producers and recorder cleanup, and retrieves its evidence.
 fixture.wait(timeout=180);result['fixture_exit']=fixture.returncode
 result['passed']=qt.returncode==0 and fixture.returncode==0
finally:
 stop(qt)
 if env:
  for s in ['tools/pointcloud-only/manage.py','tools/preflight/bridge.py']:subprocess.run(['/usr/bin/python3',str(repo/'groundstation'/s),'stop'],env=env,capture_output=True,timeout=12)
 stop(tunnel)
 if fixture and fixture.poll() is None:
  # Allow the fixture's bounded readiness failure/normal finally to collect data.
  try:fixture.wait(timeout=180)
  except subprocess.TimeoutExpired:stop(fixture)
 for f in handles:f.close()
 result.setdefault('passed',False);result['scope']='Full actual main/QML, SSH, UDP, live camera, replay LIO, synthetic gimbal over direct RTSP on isolated port, isolated recorder commands; no flight commands'
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result));assert result['passed']
