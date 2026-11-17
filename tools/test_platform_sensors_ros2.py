#!/usr/bin/env python3
"""Candidate startup ground checks; never invokes flight command services."""
import json, os, signal, subprocess, sys, time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
assert os.environ['ROS_LOCALHOST_ONLY']=='1' and os.environ['ROS_DOMAIN_ID']=='81'
out=Path(os.environ['SENSOR_TEST_OUTPUT']);out.mkdir(parents=True,exist_ok=True)
base=['bash','/home/orangepi/drone_platform/03_tools/config script/start_platform_sensors.sh']
flags={k:False for k in ['mavros','camera','lidar','rtsp']}
checks=[];streams={};states=[]
def args(**kw):
    f=dict(flags);f.update(kw)
    return ['enable_'+k+':='+str(v).lower() for k,v in f.items()]
def check(name,ok):
    checks.append({'check':name,'passed':bool(ok)});print(json.dumps(checks[-1]),flush=True)
    if not ok:raise AssertionError(name)
def reject(name,extra,expected):
    p=subprocess.run(base+extra,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=25)
    (out/(name+'.log')).write_text(p.stdout)
    check(name,p.returncode!=0 and expected in p.stdout and 'process started with pid' not in p.stdout)
reject('empty',args(),'Select at least one')
reject('invalid_bool',args(camera='maybe'),'must be true or false')
reject('missing_serial',args(mavros=True)+['fcu_url:=/dev/nonexistent_px4'],'serial device missing')
config=json.loads(Path('/home/orangepi/drone_platform/02_drivers/sensors/config/MID360s_config.json').read_text())
for host in config['Mid360s']['host_net_info']:host['host_ip']='192.0.2.17'
missing_config=out/'unassigned-host.json';missing_config.write_text(json.dumps(config,indent=2))
reject('missing_lidar_ip',args(camera=True,lidar=True)+['device_config:='+str(missing_config)],'host IP is not assigned')
show=subprocess.run(base+['--show-args'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=25)
(out/'show-args.log').write_text(show.stdout)
check('launch arguments',show.returncode==0 and all('enable_'+k in show.stdout for k in flags)
      and 'enable_usb_camera' not in show.stdout and 'video_device' not in show.stdout)
if '--preflight-only' in sys.argv:
    (out/'preflight-result.json').write_text(json.dumps({'passed':all(c['passed'] for c in checks),'checks':checks},indent=2))
    raise SystemExit(0)
rclpy.init();node=rclpy.create_node('physical_sensor_candidate_audit')
def receive(key,m):
    row=streams.setdefault(key,{'times':[],'stamps':[],'ages':[],'frames':[]})
    ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
    row['times'].append(time.monotonic());row['stamps'].append(ns)
    row['ages'].append((node.get_clock().now().nanoseconds-ns)/1e9);row['frames'].append(m.header.frame_id)
def state(m):states.append({'connected':m.connected,'armed':m.armed,'mode':m.mode})
subs=[]
for key,topic,typ in [('color','/camera/color/image_raw',Image),('depth','/camera/depth/image_raw',Image),('imu','/mavros/imu/data',Imu),('pose','/mavros/local_position/pose',PoseStamped)]:
    subs.append(node.create_subscription(typ,topic,lambda m,k=key:receive(k,m),QoSProfile(depth=20) if typ is Image else qos_profile_sensor_data))
subs.append(node.create_subscription(State,'/mavros/state',state,10))
p=None;result={}
try:
    with (out/'physical.log').open('w') as log:
        p=subprocess.Popen(base+args(mavros=True,camera=True),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        end=time.monotonic()+50;sample_at=time.monotonic()+25
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            check_alive=p.poll()
            if check_alive is not None:raise RuntimeError('candidate exited '+str(check_alive))
            if time.monotonic()<sample_at:streams.clear()
        check('PX4 connected and disarmed',bool(states) and any(s['connected'] for s in states) and all(not s['armed'] for s in states))
        for k,minimum in [('color',20),('depth',20),('imu',10),('pose',1)]:
            row=streams.get(k,{});ts=row.get('times',[]);hz=(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 else 0
            row['hz']=hz;check(k+' actual rate',hz>=minimum)
            stamps=row['stamps'];check(k+' source stamps increase',all(b>a for a,b in zip(stamps,stamps[1:])))
            check(k+' source ages bounded',all(-.1<a<2 for a in row['ages']))
        services={n for n,_ in node.get_service_names_and_types()}
        check('full MAVROS vision and command plugins loaded','/mavros/cmd/arming' in services and any(n=='/mavros/vision_pose/pose' for n,_ in node.get_topic_names_and_types()))
        result['streams']={k:{'messages':len(v['times']),'hz':v['hz'],'age_min_s':min(v['ages']),'age_max_s':max(v['ages']),'frames':sorted(set(v['frames']))} for k,v in streams.items()}
        result['states']=states[-5:]
finally:
    if p and p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=30)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
    result['streams']={k:{'messages':len(v['times']),'hz':(len(v['times'])-1)/(v['times'][-1]-v['times'][0]) if len(v['times'])>1 else 0,'age_min_s':min(v['ages']),'age_max_s':max(v['ages']),'frames':sorted(set(v['frames']))} for k,v in streams.items()}
    result.update(checks=checks,returncode=p.returncode if p else None)
    (out/'result.json').write_text(json.dumps(result,indent=2))
    node.destroy_node();rclpy.shutdown()
check('normal SIGINT shutdown',p.returncode==0)
result['checks']=checks;(out/'result.json').write_text(json.dumps(result,indent=2))
