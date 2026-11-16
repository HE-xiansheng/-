#!/usr/bin/env python3
"""Check the staged Linux tools and real read-only SSH telemetry lifecycle."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

stage=Path(sys.argv[1]).resolve();out=Path(sys.argv[2]).resolve();out.mkdir(parents=True,exist_ok=True)
state=out/'state';env=dict(os.environ,GCS_STATE_DIR=str(state),PYTHONDONTWRITEBYTECODE='1')
checks={};packets=[];commands=[]
files=sorted(p.relative_to(stage).as_posix() for p in stage.rglob('*') if p.is_file())
checks['exact_runtime_file_count']=len(files)==12
checks['legacy_installers_not_installed']=not any('automatic-navigation' in p or 'missions/' in p or 'test' in p or 'ros1_reference' in p for p in files)
checks['askpass_executable']=os.access(stage/'tools/communication/gcs-askpass',os.X_OK)

def run(rel,*args):
 p=subprocess.run(['/usr/bin/python3','-B',str(stage/'tools'/rel),*args],env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=30)
 commands.append({'script':rel,'args':args,'code':p.returncode,'output':p.stdout})
 if p.returncode:raise RuntimeError(p.stdout)
 return p.stdout

sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',19103));sock.settimeout(.5)
try:
 for rel in ['communication/manage_link.py','pointcloud-only/manage.py','preflight/bridge.py']:
  run(rel,'--help')
 checks['installed_imports_and_cli']=True
 run('communication/manage_link.py','--host','192.168.1.224','--cloud-port','19100','--pose-port','19101','--preflight-port','19103','--raw-udp-port','19115')
 end=time.monotonic()+25
 while time.monotonic()<end:
  try:raw,_=sock.recvfrom(65536)
  except socket.timeout:continue
  packet=json.loads(raw);packets.append(packet)
  if len([p for p in packets if p.get('type')=='PFS1' and p.get('connected')])>=3:break
 checks['installed_ssh_receives_live_PFS1']=any(p.get('type')=='PFS1' and p.get('connected') and p.get('source_host')=='192.168.1.224' for p in packets)
 checks['cloud_manager_running']=json.loads(run('pointcloud-only/manage.py','status')).get('running') is True
finally:
 for rel in ['pointcloud-only/manage.py','preflight/bridge.py']:
  try:run(rel,'stop')
  except Exception as exc:commands.append({'cleanup_error':str(exc)})
 sock.close()
 checks['cloud_manager_stopped']=not json.loads(run('pointcloud-only/manage.py','status')).get('running')
 checks['monitor_manager_stopped']=not json.loads(run('preflight/bridge.py','status')).get('running')
 (out/'result.json').write_text(json.dumps({'checks':checks,'passed':all(checks.values()),'installed_files':files,'commands':commands,'packets':packets},indent=2)+'\n')
print(json.dumps(checks,indent=2))
raise SystemExit(0 if all(checks.values()) else 1)
