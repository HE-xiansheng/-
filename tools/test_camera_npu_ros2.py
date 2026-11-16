#!/usr/bin/env python3
"""真实Gemini→RKNN图像预览；隔离域91，不启动任何飞控/跟踪控制器。"""
import hashlib,json,math,os,signal,subprocess,time
from collections import deque
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='91' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo
from tf2_ros import Buffer,TransformListener
out=Path(os.environ['CAMERA_NPU_OUTPUT']);out.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('physical_camera_npu_audit');buffer=Buffer();listener=TransformListener(buffer,node)
received={};headers=deque(maxlen=2000);checks=[];procs=[];logs=[];info={}
def stamp(m):return m.header.stamp.sec*1000000000+m.header.stamp.nanosec
def image(key,m):
    now=time.monotonic();ns=stamp(m);row=received.setdefault(key,{'times':[],'ages':[],'stamps':[],'sizes':[]})
    row['times'].append(now);row['ages'].append((node.get_clock().now().nanoseconds-ns)*1e-9);row['stamps'].append(ns);row['sizes'].append([m.width,m.height,m.encoding,m.header.frame_id])
    if key=='color':headers.append((ns,m.header.frame_id))
    elif key=='preview':row.setdefault('source_headers',[]).append((ns,m.header.frame_id))
def camera_info(key,m):info[key]={'width':m.width,'height':m.height,'frame':m.header.frame_id,'k':list(m.k),'d':list(m.d),'distortion_model':m.distortion_model,'stamp':stamp(m)}
for key,topic in [('color','/camera/color/image_raw'),('depth','/camera/depth/image_raw'),('preview','/manual_person_follow/image')]:node.create_subscription(Image,topic,lambda m,k=key:image(k,m),QoSProfile(depth=20))
for key,topic in [('color','/camera/color/camera_info'),('depth','/camera/depth/camera_info')]:node.create_subscription(CameraInfo,topic,lambda m,k=key:camera_info(k,m),QoSProfile(depth=1))
def check(label,v):
    checks.append({'check':label,'passed':bool(v)});print(json.dumps(checks[-1]),flush=True)
    if not v:raise AssertionError(label)
def launch(name,cmd):
    log=(out/(name+'.log')).open('w');logs.append(log);p=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p);return p
def pump(duration):
    end=time.monotonic()+duration
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
        if any(p.poll() is not None for p in procs):raise RuntimeError('camera/preview process exited')
result={}
try:
    # 官方已部署相机入口，不包含MAVROS或任务控制器。
    launch('camera',['ros2','launch','/home/orangepi/drone_platform/02_drivers/orbbec/launch/gemini335l.launch.py'])
    launch('preview',['ros2','run','yolo_ultra_test','person_preview','--ros-args','-p','image_max_age_s:=0.8'])
    pump(12.);received.clear();pump(15.)
    for key,min_rate in [('color',20.),('depth',20.),('preview',2.)]:
        r=received.get(key,{});times=r.get('times',[]);hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0.
        r['hz']=hz;check(key+' physical stream rate',hz>=min_rate)
        check(key+' stamps monotonic',len(r.get('stamps',[]))>1 and all(b>a for a,b in zip(r['stamps'],r['stamps'][1:])))
        check(key+' finite recent source age',bool(r.get('ages')) and all(math.isfinite(a) and -.05<=a<1. for a in r['ages']))
    preview=received['preview'];source_headers=set(headers);matches=[h in source_headers for h in preview.get('source_headers',[])];check('NPU annotated image preserves actual source header',bool(matches) and all(matches))
    check('actual CameraInfo for both streams',set(info)=={'color','depth'})
    transform=buffer.lookup_transform(info['color']['frame'],info['depth']['frame'],rclpy.time.Time()).transform
    q=transform.rotation;t=transform.translation
    x,y,z,w=q.x,q.y,q.z,q.w
    rotation=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)], [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)], [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    translation=[t.x,t.y,t.z]
    check('live depth optical to color optical rigid transform',np.allclose(np.asarray(rotation).T@np.asarray(rotation),np.eye(3),atol=1e-6) and np.linalg.norm(translation)<.1)
    control=['/mavros/setpoint_raw/local','/mavros/setpoint_position/local','/mavros/set_mode','/mavros/cmd/arming','/Safina_track_stamped']
    check('no flight/task/control interfaces',all(not node.get_publishers_info_by_topic(topic) for topic in control) and not any(name in ['/mavros/set_mode','/mavros/cmd/arming'] for name,_ in node.get_service_names_and_types()))
    import yaml
    old=yaml.safe_load(Path('/home/orangepi/drone_platform/01_apps/mission_demo/workspace/install/yolo_ultra_test/share/yolo_ultra_test/config/yolo_track.yaml').read_text())['/**']['ros__parameters']
    result.update(camera_info=info,depth_to_color={'R':rotation,'t':translation},old_config_difference={'rotation_matrix_max_abs':float(np.max(np.abs(np.asarray(rotation)-np.asarray(old['depth.extrinsic_rotation']).reshape(3,3)))),'translation_norm_m':float(np.linalg.norm(np.asarray(translation)-old['depth.extrinsic_translation']))},streams={k:{'messages':len(r['times']),'hz':r['hz'],'age_p95_s':float(np.percentile(r['ages'],95)),'size':r['sizes'][-1]} for k,r in received.items()})
    result['passed']=True
finally:
    for p in reversed(procs):
        if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=10)
        except subprocess.TimeoutExpired:raise RuntimeError('owned camera/preview process failed to stop')
    for log in logs:log.close()
    result['checks']=checks;result['returncodes']=[p.returncode for p in procs];(out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
    node.destroy_node();rclpy.try_shutdown()
