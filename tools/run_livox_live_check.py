#!/usr/bin/env python3
"""Own a read-only .235 subscription and isolated .224 live LIO check."""
import base64
import csv
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import selectors
import secrets
import subprocess
import sys
import tarfile
import time

repo=Path(__file__).resolve().parents[1]
out=Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=False)
seconds=float(sys.argv[2]) if len(sys.argv)>2 else 60
assert 30 <= seconds <= 90
ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8']
target=ssh+['-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc','orangepi@192.168.1.224']
source_address=os.environ.get('LIVOX_LIVE_SOURCE','192.168.1.235')
source_host=ssh+['-o','HostKeyAlias=192.168.1.235','-o','StrictHostKeyChecking=yes','orangepi@'+source_address]
remote='/tmp/livox-live-check'
direct = os.environ.get('LIVOX_LIVE_DIRECT') == '1'
direct_token = secrets.token_hex(32) if direct else None
processes=[];handles=[];result={'passed':False};consumer=None;source=None;created=False;before=None
files=['livox_live_protocol.py','test_livox_live_ros2.py','validate_fastlio_replay.py','cleanup_livox_live_check.py',
       'stream_ros1_livox_live.py','run_livox_live_check.py']
manifest={str(repo/'tools'/n):hashlib.sha256((repo/'tools'/n).read_bytes()).hexdigest()
          for n in files}
(out/'inputs-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')


def stop(p):
    if p is not None and p.poll() is None:
        os.killpg(p.pid,signal.SIGTERM)
        try:p.wait(timeout=25)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)


try:
    before=subprocess.run(source_host+['date -Is; ps -eo pid,args | grep -E "[l]ivox_ros_driver2_node|[f]astlio_mapping|[m]ediamtx"'],capture_output=True,text=True,timeout=12)
    (out/'source-before.log').write_text(before.stdout+before.stderr)
    if before.returncode:raise RuntimeError('source preflight failed: '+before.stderr.strip())
    with tarfile.open(out/'inputs.tar','w') as archive:
        for name in files:archive.add(repo/'tools'/name,arcname=name)
    with (out/'inputs.tar').open('rb') as f:
        subprocess.run(target+['mkdir '+remote+' && tar -xf - -C '+remote],stdin=f,check=True,timeout=15)
    created=True
    log=(out/'consumer.log').open('wb');handles.append(log)
    extra=('LIVOX_LIVE_DIRECT=1 LIVOX_LIVE_TOKEN='+direct_token+' ') if direct else ''
    command='source /home/orangepi/drone_platform/env.sh && '+extra+'ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR='+remote+'/ros-log python3 -u '+remote+'/test_livox_live_ros2.py '+remote
    consumer=subprocess.Popen(target+[command],stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(consumer)
    deadline=time.monotonic()+60
    while time.monotonic()<deadline:
        if consumer.poll() is not None:raise RuntimeError('consumer stopped before readiness')
        if 'LIVE_INPUT_READY' in (out/'consumer.log').read_text():break
        time.sleep(.1)
    else:raise TimeoutError('consumer readiness')
    protocol=base64.b64encode((repo/'tools/livox_live_protocol.py').read_bytes()).decode()
    observer=base64.b64encode((repo/'tools/stream_ros1_livox_live.py').read_bytes()).decode()
    bootstrap="import base64,sys,types;m=types.ModuleType('livox_live_protocol');exec(base64.b64decode('"+protocol+"'),m.__dict__);sys.modules[m.__name__]=m;sys.argv=['live-observer','"+str(seconds)+"'];exec(compile(base64.b64decode('"+observer+"'),'live-observer','exec'),{'__name__':'__main__'})"
    source_log=(out/'source.log').open('wb');handles.append(source_log)
    source=subprocess.Popen(source_host+['source /opt/ros/noetic/setup.bash && '+extra+'python3 -u -c '+shlex.quote(bootstrap)],stdout=subprocess.PIPE,stderr=source_log,start_new_session=True,bufsize=0)
    processes.append(source)
    with (out/'source.stream').open('wb') as capture, (out/'transport-chunks.csv').open('w') as trace:
        writer=csv.writer(trace)
        writer.writerow(['offset_begin','offset_end','read_begin_wall_ns','read_end_wall_ns','forward_end_wall_ns'])
        offset=0
        deadline=time.monotonic()+seconds+45
        with selectors.DefaultSelector() as selector:
            selector.register(source.stdout,selectors.EVENT_READ)
            while not direct:
                if consumer.poll() is not None:raise RuntimeError('consumer stopped while receiving live input')
                if time.monotonic()>deadline:raise TimeoutError('live source exceeded bounded duration')
                if not selector.select(timeout=1):continue
                begin_wall=time.time_ns()
                data=source.stdout.read(64*1024)
                read_wall=time.time_ns()
                if not data:break
                capture.write(data)
                consumer.stdin.write(data)
                consumer.stdin.flush()
                writer.writerow([offset,offset+len(data),begin_wall,read_wall,time.time_ns()])
                offset+=len(data)
    consumer.stdin.close()
    source.wait(timeout=seconds+45 if direct else 15);consumer.wait(timeout=45)
    result.update(source_exit=source.returncode,consumer_exit=consumer.returncode)
except Exception as exc:
    result['error']=repr(exc)
finally:
    # Stop only owned source observer/SSH; original ROS1 services are never stopped.
    stop(source)
    if consumer and consumer.poll() is None:
        if consumer.stdin and not consumer.stdin.closed:consumer.stdin.close()
        try:consumer.wait(timeout=40)
        except subprocess.TimeoutExpired:stop(consumer)
    for handle in handles:handle.close()
    result.update(source_exit=source.poll() if source else None,
                  consumer_exit=consumer.poll() if consumer else None)
    if created:
        # The owner may die in native code or lose its SSH session before finally.
        cleanup=subprocess.run(target+['python3 '+remote+'/cleanup_livox_live_check.py'],
                               capture_output=True,text=True,timeout=25)
        (out/'remote-cleanup.log').write_text(cleanup.stdout+cleanup.stderr)
        result['owned_processes_cleaned']=cleanup.returncode==0
        with (out/'evidence.tar.gz').open('wb') as f:
            collected=subprocess.run(target+['tar -czf - -C /tmp livox-live-check'],stdout=f,timeout=60)
        if collected.returncode==0:
            subprocess.run(['tar','-xzf',str(out/'evidence.tar.gz'),'-C',str(out)],check=True,timeout=30)
            result['evidence_retrieved']=True
            if cleanup.returncode==0:
                subprocess.run(target+['rm -rf -- '+remote],check=True,timeout=15)
                result['remote_removed']=True
    after=subprocess.run(source_host+['date -Is; ps -eo pid,args | grep -E "[l]ivox_ros_driver2_node|[f]astlio_mapping|[m]ediamtx"'],capture_output=True,text=True,timeout=12)
    (out/'source-after.log').write_text(after.stdout+after.stderr)
    def original_pids(text):
        return {line.split()[0] for line in text.splitlines() if line.split() and line.split()[0].isdigit()}
    result['original_source_process_ids_unchanged']=bool(before is not None and before.returncode==0 and after.returncode==0 and original_pids(before.stdout)==original_pids(after.stdout))
    p=out/'livox-live-check/result.json'
    result['passed']=bool(p.exists() and json.loads(p.read_text()).get('passed') and result.get('source_exit')==0 and result.get('consumer_exit')==0 and result.get('owned_processes_cleaned') and result['original_source_process_ids_unchanged'] and 'error' not in result)
    (out/'runner-result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result));raise SystemExit(0 if result['passed'] else 1)
