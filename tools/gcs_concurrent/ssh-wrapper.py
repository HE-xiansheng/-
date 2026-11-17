#!/usr/bin/python3
"""Confine the GUI to a test ROS domain and an independent recorder."""
import os,sys,json,shlex
args=sys.argv[1:];remote=args[-1];original=remote
assert 'orangepi@192.168.1.224' in args,'unexpected destination'
words=shlex.split(remote)
if words[:2]==['bash','-lc']:
 assert len(words)==3
 remote=words[2]
root='/tmp/platform-soak-check/recording-source'
if remote=='printf GCS_SSH_OK':kind='authenticate'
elif 'gcs_task_manager.py' in remote:
 assert shlex.split(remote)[-1]=='watch','only mission watch is allowed'
 remote+=' --store /tmp/platform-soak-check/task-state';kind='mission_readonly'
elif 'recording_manage.py' in remote:
 assert remote.endswith(' start') or remote.endswith(' stop'),'only manual recording is allowed'
 remote=remote.replace('~/drone_platform/01_apps/ego_planner/recording_manage.py',root+'/recording_manage.py');kind='recording'
 if remote.endswith(' stop'):
  remote="python3 -c "+shlex.quote("import time,json;from pathlib import Path;Path('/tmp/platform-soak-check/ui-stop-request.json').write_text(json.dumps({'time_ns':time.time_ns()}))")+' && '+remote
elif '--cloud-topic /factory_fastlio/cloud_world --no-rosout --planner' in remote:kind='telemetry_readonly'
elif '--pc-ip ' in remote and '# dianyun-pointcloud-only' in remote:kind='cloud_readonly'
elif 'b64decode' in remote and ('--source raw' in remote or '--source tracking' in remote):kind='video_readonly'
else:raise RuntimeError('unexpected GUI command: '+remote)
if kind=='recording':remote='cd /tmp/platform-soak-check && '+remote
if kind!='authenticate':remote='env ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 RECORDING_ROOT='+root+' ROS_LOG_DIR=/tmp/platform-soak-check/gcs-ros-log bash -lc '+shlex.quote(remote)
args[-1]=remote
with open(os.environ['GCS_TEST_SSH_CALLS'],'a') as f:f.write(json.dumps({'kind':kind,'original':original,'effective':remote})+'\n')
os.execv('/usr/bin/ssh',['ssh','-o','UserKnownHostsFile='+os.environ['GCS_TEST_KEYS'],'-o','HostKeyAlias=opi-ros2-emmc',*args])
