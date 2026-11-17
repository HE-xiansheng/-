#!/usr/bin/env python3
"""No camera streaming: physical device rejection and Empty service compatibility."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool

base=Path(sys.argv[1]); module=Path(sys.argv[2]); checks=[]
assert os.environ.get('ROS_DOMAIN_ID')=='84' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
spec=importlib.util.spec_from_file_location('usb_launch',str(module/'launch/usb_camera.launch.py'))
launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)

def check(ok,label):
    checks.append({'check':label,'passed':bool(ok)}); print(('PASS ' if ok else 'FAIL ')+label,flush=True)
    assert ok,label

def refused(path,expected):
    try: launch.validate_device(path)
    except RuntimeError as exc:return expected in str(exc)
    return False

check(refused('/dev/usb_cam','not connected'),'missing reference alias fails without a fallback')
check(refused('/etc/hosts','Not a character'),'regular file rejected')
check(refused('/dev/null','Not a V4L2'),'non-video character device rejected')
for p in Path('/sys/class/video4linux').glob('video*'):
    label=(p/'name').read_text()
    if 'hdmirx' in label:check(refused('/dev/'+p.name,'not a USB camera'),'physical HDMI receiver is rejected')
    elif 'Orbbec' in label:
        check(refused('/dev/'+p.name,'Orbbec'),'physical Gemini '+p.name+' is rejected')

rclpy.init();node=rclpy.create_node('usb_capture_fixture');calls=[];statuses=[]; mode='accept'
def capture(request,response):
    calls.append(request.data)
    if mode=='slow':time.sleep(4)
    response.success=mode=='accept';response.message='fixture '+mode
    return response
server=node.create_service(SetBool,'/usb_cam/set_capture',capture,callback_group=ReentrantCallbackGroup())
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String,'/usb_cam/capture_status',lambda m:statuses.append(json.loads(m.data)),qos)
start=node.create_client(Empty,'/usb_cam/start_capture');stop=node.create_client(Empty,'/usb_cam/stop_capture')
executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
thread=threading.Thread(target=executor.spin);thread.start()
log=(base/'compat.log').open('wb')
proc=subprocess.Popen([sys.executable,str(module/'capture_compat.py')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)

def wait(predicate,timeout=8):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return True
        time.sleep(.03)
    return bool(predicate())

def invoke(client):
    count=len(statuses);begin=time.monotonic();future=client.call_async(Empty.Request())
    assert wait(future.done,6) and future.result() is not None
    assert wait(lambda:len(statuses)>count,2)
    return statuses[-1],time.monotonic()-begin

try:
    check(wait(lambda:start.service_is_ready() and stop.service_is_ready()),'legacy Empty service names appear')
    status,_=invoke(start)
    check(calls[-1] is True and status['success'] and status['requested_capture'],'start Empty forwards true to native SetBool')
    status,_=invoke(stop)
    check(calls[-1] is False and status['success'] and not status['requested_capture'],'stop Empty forwards false to native SetBool')
    mode='reject';status,_=invoke(start)
    check(not status['success'] and 'reject' in status['message'],'native rejection is retained in operator status')
    mode='slow';status,elapsed=invoke(stop)
    check(not status['success'] and 'timed out' in status['message'] and elapsed<3.6,'nonresponsive native service has bounded timeout')
    time.sleep(2.2)
    node.destroy_service(server);server=None
    status,elapsed=invoke(start)
    check(not status['success'] and 'unavailable' in status['message'] and elapsed<2.5,'missing native service is reported within bound')
    # An in-flight service callback must also survive launch/group double SIGINT.
    mode='slow'
    server=node.create_service(SetBool,'/usb_cam/set_capture',capture,callback_group=ReentrantCallbackGroup())
    count=len(calls)
    pending=start.call_async(Empty.Request())
    check(wait(lambda:len(calls)>count,4),'shutdown case has a real service callback in flight')
    proc.send_signal(signal.SIGINT)
    time.sleep(.05)
    check(proc.poll() is None,'compatibility node is finishing its pending callback')
    proc.send_signal(signal.SIGINT);proc.wait(timeout=6)
    check(proc.returncode==0,'compatibility node shuts down normally')
finally:
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT);proc.wait(timeout=6)
    log.close();executor.shutdown();thread.join(timeout=3);node.destroy_node();rclpy.shutdown()
    (base/'result.json').write_text(json.dumps({'passed':all(x['passed'] for x in checks),'checks':checks},indent=2))
