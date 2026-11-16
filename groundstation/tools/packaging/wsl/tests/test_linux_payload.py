"""实际安装、启动、卸载Linux载荷；不接触Windows/WSL或机载设备。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

p=argparse.ArgumentParser();p.add_argument('installer',type=Path);p.add_argument('--report',type=Path,required=True);a=p.parse_args()
base=Path(tempfile.mkdtemp(prefix='gcs-wsl-payload-check-'));prefix=base/'地面站 installed with spaces';apps=base/'apps';apps.mkdir()
previous='previous shortcut\n';(apps/'research-flight-groundstation.desktop').write_text(previous)
result=subprocess.run(['bash',str(a.installer),'--prefix',str(prefix),'--applications-dir',str(apps),'--no-desktop'],capture_output=True,text=True,timeout=90)
assert result.returncode==0,result.stdout+result.stderr
manifest=json.loads((prefix/'payload-manifest.json').read_text())
for name,sha in manifest['files'].items():assert hashlib.sha256((prefix/name).read_bytes()).hexdigest()==sha,name
fake=base/'bin';fake.mkdir();(fake/'ssh').write_text('#!/bin/sh\nexit 0\n');(fake/'ssh').chmod(0o755)
env=dict(os.environ,QT_QPA_PLATFORM='offscreen',QT_OPENGL='software',LIBGL_ALWAYS_SOFTWARE='1',XDG_CONFIG_HOME=str(base/'config'),XDG_CACHE_HOME=str(base/'cache'),XDG_DATA_HOME=str(base/'data'),PATH=str(fake)+':'+os.environ['PATH'])
env.pop('GCS_SSH_PASSWORD',None)
config=base/'config/Unknown Organization/customgeometry.conf';config.parent.mkdir(parents=True)
config.write_text('[onboard]\nhost=192.168.1.13\n')
def check_start(name,args,host):
 log=base/(name+'.log')
 with log.open('w') as f:
  proc=subprocess.Popen([str(prefix/'start-groundstation.sh'),*args,'--no-splash'],env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
  try:
   time.sleep(3);message=log.read_text()
   assert proc.poll() is None and 'QML module loaded' in message and '首帧就绪' in message,message[-3000:]
   assert '启动机载地址（视频与通信）：'+host in message,message[-3000:]
  finally:
   if proc.poll() is None:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=6)
 return log
log=check_start('saved-settings',['--cloud-port','56201'],'192.168.1.13')
connection=prefix/'.pointcloud-link/connection.json';connection.parent.mkdir(exist_ok=True)
connection.write_text(json.dumps({'host':'192.168.1.122'}))
check_start('saved-link',['--live','56201'],'192.168.1.122')
check_start('explicit-host',['--live','56201','--host','192.168.1.123'],'192.168.1.123')
check_start('explicit-equal',['--live','56201','--host=192.168.1.124'],'192.168.1.124')
assert '192.168.1.13' in config.read_text(),'启动覆盖不应改写已保存的认证地址'
invalid=subprocess.run([str(prefix/'start-groundstation.sh'),'--no-splash','--host','--bad'],env=env,capture_output=True,text=True,timeout=8)
assert invalid.returncode==2 and '--host requires' in invalid.stderr,invalid.stderr
result=subprocess.run(['python3',str(prefix/'install_user.py'),'--uninstall','--data-dir',str(base/'saved')],capture_output=True,text=True,timeout=25)
assert result.returncode==0,result.stdout+result.stderr
assert not prefix.exists() and (apps/'research-flight-groundstation.desktop').read_text()==previous
report={'passed':True,'version':manifest['version'],'manifest_hashes_match':True,'install_with_space_and_chinese_path':True,'qt_qml_first_frame':True,'saved_settings_and_link_hosts':True,'explicit_host_both_syntaxes':True,'invalid_host_rejected':True,'startup_does_not_save_unauthenticated_host':True,'uninstall_and_shortcut_restore':True,'log':str(log),'scope':'Ubuntu20.04 local installation and offscreen startup; fake SSH; no WSL or aircraft access'}
a.report.parent.mkdir(parents=True,exist_ok=True);a.report.write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False))
