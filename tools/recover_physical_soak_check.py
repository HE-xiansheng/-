#!/usr/bin/env python3
"""Retrieve a disconnected check before reusing its temporary directory."""
import argparse,json,shlex,subprocess,tarfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args()
run_root=a.run.resolve();fixture=run_root/'fixture'
prior=json.loads((fixture/'runner-result.json').read_text())
if not prior.get('physical_inputs'):raise ValueError('not a physical check')
manifest=json.loads((fixture/'inputs-manifest.json').read_text())
profile=json.loads((run_root/'recovery-plan.json').read_text())
out=run_root/'recovery';out.mkdir(exist_ok=False)
remote='/tmp/platform-soak-check'
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','ServerAliveInterval=15','-o','ServerAliveCountMax=3','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
def call(command,name,timeout=30,check=True):
 r=subprocess.run(ssh+[command],capture_output=True,text=True,timeout=timeout)
 (out/(name+'.log')).write_text(r.stdout+r.stderr)
 if check:r.check_returncode()
 return r
result={'passed':False}
try:
 call('date -Is; cat /proc/sys/kernel/random/boot_id; uptime; git -C /home/orangepi/drone_platform status --porcelain','before')
 exists=call('test -d '+remote,'temporary-directory',check=False).returncode==0
 result['temporary_directory_existed']=exists
 if exists:
  # Match the exact helper uploaded by this experiment before signalling anything.
  digest=call('sha256sum '+remote+'/cleanup_livox_live_check.py','cleanup-identity').stdout.split()[0]
  if digest!=manifest['cleanup_livox_live_check.py']:raise RuntimeError('temporary directory ownership differs')
  call('python3 '+remote+'/cleanup_livox_live_check.py '+remote,'owned-cleanup')
  with (out/'evidence.tar.gz').open('wb') as stream:subprocess.run(ssh+['tar -I "gzip -1" -cf - -C /tmp platform-soak-check'],stdout=stream,check=True,timeout=60)
  with tarfile.open(out/'evidence.tar.gz') as archive:
   if not all(m.name=='platform-soak-check' or m.name.startswith('platform-soak-check/') and '..' not in Path(m.name).parts for m in archive):raise RuntimeError('unexpected archive paths')
  subprocess.run(['tar','-xzf',str(out/'evidence.tar.gz'),'-C',str(out)],check=True,timeout=30)
  result['evidence_retrieved']=True
  call('rm -rf -- '+remote,'remove')
 # Restore only the profile this experiment owns, preserving other active connections.
 name=profile['temporary_profile'];original=profile['original_profile_uuid']
 if not name.startswith('ROS2-Gimbal-soak-'):raise ValueError('unexpected profile')
 profiles=call('nmcli -g NAME connection show','profiles').stdout.splitlines()
 if name in profiles:
  active=call('nmcli -g GENERAL.CONNECTION device show enx00e09a5af420','active-profile').stdout.strip()
  call(shlex.join(['nmcli','connection','delete','id',name]),'delete-owned-profile')
  if active==name and original and original!='--':call(shlex.join(['nmcli','-w','0','connection','up','uuid',original,'ifname','enx00e09a5af420']),'restore-original-profile')
 call('test ! -e '+remote,'directory-removed')
 final=call('date -Is; ip route get 192.168.1.158; ip route get 1.1.1.1; nmcli -t -f NAME,UUID,DEVICE connection show --active; git -C /home/orangepi/drone_platform status --porcelain','final-state').stdout
 if '192.168.1.158 dev enP3p49s0 src 192.168.1.51' not in final or '1.1.1.1 via 192.168.1.1 dev wlan0' not in final:raise RuntimeError('management or sensor route differs')
 active=call('pgrep -af "[l]ivox_ros_driver2_node|[f]astlio_mapping|[v]ision2mavros|[o]rbbec_camera_node|[r]tsp_hw_pub_ultra|[p]erson_preview|[p]lanner_node|[m]ediamtx" || true','processes-after').stdout
 if active.strip():raise RuntimeError('processing remains active; do not overlap another check')
 if name in call('nmcli -g NAME connection show','profiles-after').stdout.splitlines():raise RuntimeError('owned temporary network still present')
 result['passed']=True
except Exception as exc:result['error']=repr(exc)
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result));raise SystemExit(0 if result['passed'] else 1)
