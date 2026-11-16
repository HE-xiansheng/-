#!/usr/bin/env python3
"""真实SSH/UDP→Qt遥测检查。只允许认证、只读采样和合成ROS2输入。"""
import argparse,json,os,shlex,signal,subprocess,tempfile,time,shutil
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
KEYS='/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts'
SSH=['/usr/bin/ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','UserKnownHostsFile='+KEYS,'-o','HostKeyAlias=opi-ros2-emmc']

def main():
 p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
 remote="source /home/orangepi/drone_platform/env.sh && ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 exec python3 /tmp/gcs-telemetry-ros2-check/tools/test_telemetry_ros2.py --reader /tmp/gcs-telemetry-ros2-check/groundstation/tools/preflight/telemetry_reader.py --output /tmp/gcs-telemetry-ros2-user-check --serve 75"
 with tempfile.TemporaryDirectory(prefix='gcs-ros2-link-') as task_tmp:
  tmp=Path(task_tmp);state=tmp/'state';state.mkdir();shim=tmp/'ssh'
  shim.write_text('''#!/usr/bin/python3
import json,os,shlex,sys
args=sys.argv[1:];remote=args[-1]
if remote!='printf GCS_SSH_OK':
 assert remote.startswith('source ~/drone_platform/env.sh && exec /usr/bin/python3 -u - '), 'test only permits read-only samplers'
 assert ('--cloud-topic ' in remote and '--no-rosout --planner' in remote) or '--pc-ip ' in remote
 remote='env ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 bash -lc '+shlex.quote(remote)
 args[-1]=remote
with open(os.environ['GCS_TEST_SSH_CALLS'],'a') as f:f.write(json.dumps(args)+'\\n')
os.execv('/usr/bin/ssh',['ssh','-o','UserKnownHostsFile='+os.environ['GCS_TEST_KEYS'],'-o','HostKeyAlias=opi-ros2-emmc',*args])
''');shim.chmod(0o700)
  env=dict(os.environ,PATH=str(tmp)+':'+os.environ['PATH'],GCS_STATE_DIR=str(state),GCS_TEST_SSH_CALLS=str(out/'ssh-calls.jsonl'),GCS_TEST_KEYS=KEYS)
  fixture_log=(out/'fixture.log').open('w')
  fixture=subprocess.Popen([*SSH,'orangepi@192.168.1.224','bash -lc '+shlex.quote(remote)],stdout=fixture_log,stderr=subprocess.STDOUT)
  result=None
  try:
   deadline=time.monotonic()+15
   while time.monotonic()<deadline and 'FIXTURE_READY' not in (out/'fixture.log').read_text():
    if fixture.poll() is not None:raise RuntimeError('fixture exited: '+(out/'fixture.log').read_text())
    time.sleep(.1)
   if 'FIXTURE_READY' not in (out/'fixture.log').read_text():raise RuntimeError('fixture timeout')
   with (out/'qt.log').open('w') as log:result=subprocess.run([a.binary],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=50)
  finally:
   # 只停止本次独立状态目录拥有的本机桥，不接管原地面站。
   for script in [ROOT/'groundstation/tools/pointcloud-only/manage.py',ROOT/'groundstation/tools/preflight/bridge.py']:
    subprocess.run(['/usr/bin/python3',str(script),'stop'],env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=12)
   if fixture.poll() is None:fixture.terminate()
   try:fixture.wait(timeout=5)
   except subprocess.TimeoutExpired:fixture.kill();fixture.wait()
   fixture_log.close()
   shutil.copytree(state,out/'local-bridge-state',dirs_exist_ok=True)
  passed=result is not None and result.returncode==0
  (out/'result.json').write_text(json.dumps({'passed':passed,'qt_exit':None if result is None else result.returncode,'scope':'actual SSH and cross-host UDP, synthetic ROS2 domain 198 localhost inputs, no flight task'},indent=2)+'\n')
  if not passed:raise SystemExit((out/'qt.log').read_text())
  print((out/'qt.log').read_text())

if __name__=='__main__':main()
