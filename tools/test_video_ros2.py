#!/usr/bin/env python3
"""在机载隔离DDS域验证图像桥/真实NPU及进程所属关系，不启动任务控制器。"""
import os, sys, time, json, threading, subprocess, struct, signal
from pathlib import Path
import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
BASE=Path(sys.argv[1]);checks=[]
rclpy.init();n=rclpy.create_node('video_fixture')
bridge=CvBridge();qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
camera=n.create_publisher(Image,'/camera/color/image_raw',qos)
image=cv2.imread(str(BASE/'bus.jpg'))
assert image is not None

def check(ok,label):
    checks.append({'label':label,'passed':bool(ok)});print(('PASS ' if ok else 'FAIL ')+label,flush=True)
    assert ok,label

def publish(pub,frame=None,age=0):
    pixels=image if frame is None else frame
    stamp=n.get_clock().now().nanoseconds-int(age*1e9)
    if pub.msg_type is CompressedImage:
        msg=CompressedImage();msg.format='jpeg';msg.data=cv2.imencode('.jpg',pixels)[1].tobytes()
    else:msg=bridge.cv2_to_imgmsg(pixels,encoding='bgr8')
    msg.header.frame_id='fixture_camera';msg.header.stamp.sec=stamp//1000000000;msg.header.stamp.nanosec=stamp%1000000000
    pub.publish(msg)

def spin(seconds,send=None):
    end=time.monotonic()+seconds;next_send=0
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.02)
        if send and time.monotonic()>=next_send:
            send();next_send=time.monotonic()+.12

class Stream:
    def __init__(self,source,extra_env=None):
        self.log=open(BASE/('bridge-'+source+'-'+str(time.time_ns())+'.log'),'wb')
        self.frames=[]
        self.p=subprocess.Popen([sys.executable,str(BASE/'ros_image_bridge.py'),'--source',source],stdout=subprocess.PIPE,stderr=self.log,env=dict(os.environ,**(extra_env or {})))
        self.thread=threading.Thread(target=self.read,daemon=True);self.thread.start()
    def read(self):
        while True:
            head=self.p.stdout.read(8)
            if len(head)!=8:return
            if head[:4]!=b'VID1':return
            length=struct.unpack('>I',head[4:])[0];data=self.p.stdout.read(length)
            if len(data)!=length:return
            self.frames.append(cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR))
    def stop(self):
        if self.p.poll() is None:
            self.p.terminate()
            try:self.p.wait(timeout=9)
            except subprocess.TimeoutExpired:self.p.kill();self.p.wait()
        self.thread.join(timeout=1);self.log.close()
streams=[]
samples=[]
n.create_subscription(Image,'/manual_person_follow/image',lambda msg:samples.append(msg),qos)
try:
    for mode,topic,kind in [('raw','/camera/color/image_raw/compressed',CompressedImage),('tracking','/yoloUltra_imgresult',Image),('aruco','/aruco_imgresult',Image)]:
        pub=n.create_publisher(kind,topic,qos);s=Stream(mode);streams.append(s)
        spin(2,lambda:publish(pub,image))
        check(len(s.frames)>0,mode+' VID1 JPEG output')
        check(s.frames[-1].shape[1]==640,mode+' width limit')
        spin(1.2);before=len(s.frames)
        spin(.5,lambda:publish(pub,image,age=4))
        check(len(s.frames)==before,mode+' stale source rejected')
        spin(.5,lambda:publish(pub,image,age=-4))
        check(len(s.frames)==before,mode+' future source rejected')
        s.stop();check(s.p.returncode==0,mode+' graceful shutdown')
        n.destroy_publisher(pub)
    s=Stream('person');streams.append(s)
    spin(6,lambda:publish(camera,image))
    check(len(samples)>0 and len(s.frames)>0,'actual NPU image to VID1')
    check(all(m.header.frame_id=='fixture_camera' and m.header.stamp.sec>0 for m in samples),'preview preserves source header')
    pixels=bridge.imgmsg_to_cv2(samples[-1],'bgr8')
    check(np.count_nonzero((pixels[:,:,1]>220)&(pixels[:,:,0]<40)&(pixels[:,:,2]<40))>100,'actual NPU person annotation')
    providers=n.get_publishers_info_by_topic('/manual_person_follow/image')
    check(len(providers)==1,'single preview provider')
    topics=[t for t,_ in n.get_publisher_names_and_types_by_node(providers[0].node_name,providers[0].node_namespace)]
    check(not any('mavros' in t or t.endswith('/target') or t.endswith('/track') for t in topics),'preview has no control publishers')
    spin(1);before=len(samples)
    spin(.5,lambda:publish(camera,image,age=4))
    check(len(samples)==before,'preview rejects stale images')
    spin(.5,lambda:publish(camera,image,age=-4))
    check(len(samples)==before,'preview rejects future images')
    s2=Stream('person');streams.append(s2)
    spin(4,lambda:publish(camera,image))
    check(len(n.get_publishers_info_by_topic('/manual_person_follow/image'))==1 and len(s2.frames)>0,'two viewers share one NPU provider')
    s.stop();spin(6,lambda:publish(camera,image))
    check(len(n.get_publishers_info_by_topic('/manual_person_follow/image'))==1 and len(s2.frames)>10,'remaining viewer takes over provider')
    s2.stop();spin(2)
    check(n.count_publishers('/manual_person_follow/image')==0,'last viewer releases owned provider')
    external=n.create_publisher(Image,'/manual_person_follow/image',qos)
    s3=Stream('person');streams.append(s3);spin(4,lambda:publish(external,image))
    check(n.count_publishers('/manual_person_follow/image')==1 and len(s3.frames)>0,'external provider reused')
    s3.stop();spin(1)
    check(n.count_publishers('/manual_person_follow/image')==1,'external provider survives viewer close')
    n.destroy_publisher(external)
    spin(2)
    missing=Stream('person',{'GCS_PERSON_MODEL':str(BASE/'missing-model.rknn')});streams.append(missing)
    spin(4)
    check(missing.p.poll()==1,'missing model produces nonzero exit')
    missing.stop()
    check('GCS_PERSON_ERROR:' in Path(missing.log.name).read_text(),'missing model reports explicit UI error')
    killed=Stream('person');streams.append(killed);spin(5,lambda:publish(camera,image))
    check(n.count_publishers('/manual_person_follow/image')==1,'provider exists before SIGKILL')
    killed.p.kill();killed.p.wait();spin(4)
    check(n.count_publishers('/manual_person_follow/image')==0,'SIGKILL bridge releases own provider through parent death signal')
finally:
    for stream in streams:stream.stop()
    (BASE/'results.json').write_text(json.dumps(checks,indent=2))
    n.destroy_node();rclpy.shutdown()
