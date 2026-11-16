#!/usr/bin/env python3
"""真实检测/降落节点的私有ROS集成测试，仅链接GCU桩的测试二进制可用。
随机localhost master、合成反色231图像、模拟MAVROS服务，禁止使用现场ROS_MASTER_URI。
python3 test_aruco_isolated.py BUILD_DIR [--output EVIDENCE_DIR]
"""
import argparse,csv,json,os,shutil,signal,socket,subprocess,tempfile,threading,time
from pathlib import Path

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('build',type=Path);parser.add_argument('--output',type=Path)
    args=parser.parse_args();build=args.build.resolve()
    # 不能误把正式二进制作为参数；仅使用本仓库CMake生成的指定fixture文件名。
    binaries=[build/'aruco_landing_fixture',build/'aruco_detector_fixture']
    if not all(p.is_file() for p in binaries):parser.error('build the test-native CMake project first')
    if b'GCS_TEST_GIMBAL' not in binaries[0].read_bytes():parser.error('landing fixture must link the test GCU stub')
    temp=tempfile.TemporaryDirectory(prefix='aruco-isolated-');root=Path(temp.name)
    with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d'%port,ROS_IP='127.0.0.1',ROS_LOG_DIR=str(root/'ros'),GCS_TEST_GIMBAL=str(root/'gimbal.txt'))
    os.environ.pop('ROS_HOSTNAME',None)
    import cv2,numpy as np,rospy,rosgraph,yaml
    cv2.setNumThreads(2)
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import PoseStamped,TwistStamped
    from mavros_msgs.msg import State,RCIn,ExtendedState,PositionTarget
    from mavros_msgs.srv import SetMode,SetModeResponse,CommandBool,CommandBoolResponse
    from std_msgs.msg import String
    processes=[];files=[];run=threading.Event();run.set();errors=[]
    sim=dict(mode='POSCTL',armed=True,pwm=1494,rssi=255,channels=18,rc_stamp='now',rc=True,
             marker=True,image_stamp='now',small=False,x=0.,y=0.,z=5.2,ground=3.4,plant=False)
    commands=[];requests=[];rows=[];header=[];metrics=[];images=[];bridge=CvBridge()
    def check(condition,label):
        if not condition:raise AssertionError(label)
        print('PASS',label,flush=True)
    def wait(predicate,label,seconds=10):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            if errors:raise RuntimeError(errors[0])
            if predicate():return
            time.sleep(.02)
        raise AssertionError(label+' last='+repr(rows[-1:]))
    def spawn(argv,name):
        f=(root/(name+'.log')).open('w');files.append(f)
        p=subprocess.Popen(argv,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
    def mode(req):requests.append(('mode',req.custom_mode));sim['mode']=req.custom_mode;return SetModeResponse(mode_sent=True)
    def arm(req):requests.append(('arm',req.value));return CommandBoolResponse(success=True,result=0)
    def status(msg):
        values=next(csv.reader([msg.data]))
        if len(header)==len(values):rows.append(dict(zip(header,values)))
    def cancel(reason,change):
        sim.update(rc=True,channels=18,rssi=255,rc_stamp='now',pwm=1494,mode='POSCTL',plant=False)
        time.sleep(.65);old=len(rows);sim['pwm']=1913
        wait(lambda:len(rows)>old and rows[-1].get('enabled')=='1','新OFFBOARD边沿启用')
        change();wait(lambda:rows[-1].get('reason')==reason,'精确取消原因 '+reason)
        check(rows[-1].get('rc_cancel_detail','').startswith('channel=5;'),'取消冻结RC原始证据')
        time.sleep(.15);n=len(commands);time.sleep(.25);check(len(commands)==n,'取消后停止设定点 '+reason)
    try:
        spawn(['roscore','-p',str(port)],'master');wait(rosgraph.is_master_online,'私有master')
        rospy.init_node('aruco_isolated_fixture',disable_signals=True)
        cfg=yaml.safe_load((Path(__file__).parent/'test-native/fixtures/takeoff_land.snapshot.yaml').read_text())
        for k,v in cfg.items():rospy.set_param('/'+k,v)
        rospy.set_param('/aruco_landing/diagnostic_directory',str(root/'diagnostics'))
        rospy.Service('/mavros/set_mode',SetMode,mode);rospy.Service('/mavros/cmd/arming',CommandBool,arm)
        state=rospy.Publisher('/mavros/state',State,queue_size=1);rc=rospy.Publisher('/mavros/rc/in',RCIn,queue_size=1)
        ext=rospy.Publisher('/mavros/extended_state',ExtendedState,queue_size=1)
        pose=rospy.Publisher('/mavros/local_position/pose',PoseStamped,queue_size=1);vel=rospy.Publisher('/mavros/local_position/velocity_local',TwistStamped,queue_size=1)
        image=rospy.Publisher('/rtsp_cam/image_raw',Image,queue_size=1)
        rospy.Subscriber('/mavros/setpoint_raw/local',PositionTarget,lambda m:commands.append(m))
        rospy.Subscriber('/aruco_landing/status_header',String,lambda m:header.__setitem__(slice(None),next(csv.reader([m.data]))))
        rospy.Subscriber('/aruco_landing/status',String,status)
        rospy.Subscriber('/aruco_detector/metrics',String,lambda m:metrics.append(json.loads(m.data)))
        rospy.Subscriber('/aruco_imgresult',Image,lambda m:images.append((m.width,m.height)))
        K=np.array([[2042.0851788597752,0,959.0675902004489],[0,2042.8993693539358,467.50991502443503],[0,0,1]],float)
        D=np.array([-.4849208954761301,.3575460777522052,.0024881576093297698,-.0002765905391136276,0.])
        marker=cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250),231,280)
        h=.195/2;model=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],np.float32)
        source=np.array([[0,0],[279,0],[279,279],[0,279]],np.float32)
        def feed():
            try:
                last_image=-1.;last=time.monotonic()
                while run.is_set():
                    now=rospy.Time.now();tick=time.monotonic();dt=tick-last;last=tick
                    if sim['plant'] and sim['mode']=='OFFBOARD' and commands:
                        c=commands[-1];sim['x']+=c.velocity.x*dt;sim['y']+=c.velocity.y*dt;sim['z']+=c.velocity.z*dt
                    if sim['plant'] and sim['mode']=='AUTO.LAND':
                        sim['z']=max(sim['ground'],sim['z']-.3*dt)
                        if sim['z']<=sim['ground']+.001:sim['armed']=False
                    s=State(connected=True,armed=sim['armed'],mode=sim['mode']);s.header.stamp=now;state.publish(s)
                    if sim['rc']:
                        r=RCIn(rssi=sim['rssi'],channels=[1494]*sim['channels']);r.header.stamp=now
                        if len(r.channels)>4:r.channels[4]=sim['pwm']
                        if sim['rc_stamp']=='missing':r.header.stamp=rospy.Time(0)
                        elif sim['rc_stamp']=='stale':r.header.stamp=now-rospy.Duration(1)
                        elif sim['rc_stamp']=='future':r.header.stamp=now+rospy.Duration(1)
                        rc.publish(r)
                    e=ExtendedState(landed_state=1 if not sim['armed'] else 2);e.header.stamp=now;ext.publish(e)
                    p=PoseStamped();p.header.stamp=now;p.header.frame_id='map';p.pose.position.x=sim['x'];p.pose.position.y=sim['y'];p.pose.position.z=sim['z'];p.pose.orientation.w=1;pose.publish(p)
                    v=TwistStamped();v.header=p.header
                    if sim['plant'] and commands and sim['mode']=='OFFBOARD':v.twist.linear=commands[-1].velocity
                    vel.publish(v)
                    if tick-last_image>=.08:
                        frame=np.full((1080,1920),255,np.uint8)
                        if sim['marker']:
                            optical=np.array([sim['y'],.2+sim['x'],sim['z']-sim['ground']],float)
                            corners,_=cv2.projectPoints(model,np.array([np.pi,0.,0.]),optical,K,D)
                            H=cv2.getPerspectiveTransform(source,corners.reshape(4,2).astype(np.float32))
                            frame=cv2.warpPerspective(marker,H,(1920,1080),borderValue=255)
                        frame=cv2.bitwise_not(frame)
                        if sim['small']:frame=cv2.resize(frame,(640,360))
                        m=bridge.cv2_to_imgmsg(cv2.cvtColor(frame,cv2.COLOR_GRAY2BGR),'bgr8');m.header.stamp=now;m.header.frame_id='gimbal_camera_optical_frame'
                        if sim['image_stamp']=='stale':m.header.stamp=now-rospy.Duration(2)
                        elif sim['image_stamp']=='missing':m.header.stamp=rospy.Time(0)
                        image.publish(m);last_image=tick
                    time.sleep(.02)
            except Exception as error:errors.append(repr(error))
        thread=threading.Thread(target=feed,daemon=True);thread.start()
        spawn([str(binaries[1])],'detector');spawn([str(binaries[0])],'landing')
        wait(lambda:rows and metrics and images,'实际节点发布状态和识别预览',15)
        wait(lambda:any(x.get('selected_id')==231 for x in metrics),'实际检测器识别合成反色231')
        check(images[-1][0]==960,'实际预览宽度960')
        time.sleep(.8);check(not commands and not requests,'准备不发飞行输出/服务请求')
        sim['small']=True;wait(lambda:metrics[-1]['reason']=='resolution_mismatch','分辨率不符失效');sim['small']=False
        sim['image_stamp']='stale';wait(lambda:rows[-1].get('marker_reason')=='source_stamp_stale','过期图像拒绝进入控制');sim['image_stamp']='now'
        wait(lambda:rows[-1].get('marker_reason')=='accepted','恢复新源戳')
        check(float(rows[-1]['marker_pair_dt'])<=.15,'真实源戳不减0.3秒延迟')
        cases=[('rc_channel_missing',lambda:sim.update(channels=4)),('rc_pwm_invalid',lambda:sim.update(pwm=0)),
            ('rc_rssi_zero',lambda:sim.update(rssi=0)),('rc_stamp_missing',lambda:sim.update(rc_stamp='missing')),
            ('rc_stamp_future',lambda:sim.update(rc_stamp='future')),('rc_stamp_stale',lambda:sim.update(rc_stamp='stale')),
            ('rc_switch_out',lambda:sim.update(pwm=1494)),('rc_receive_timeout',lambda:sim.update(rc=False))]
        for reason,change in cases:cancel(reason,change)
        sim.update(rc=True,channels=18,rssi=255,rc_stamp='now',pwm=1494,mode='POSCTL',x=0.,y=0.,z=5.2,plant=True)
        time.sleep(.7);sim['pwm']=1913
        wait(lambda:rows[-1].get('phase')=='DESCEND','对准稳定后下降',15)
        check(not any(k=='arm' for k,v in requests),'人工模式不请求解锁或锁桨')
        sim['marker']=False;wait(lambda:rows[-1].get('phase')=='LOST_HOLD','高处丢码暂停下降',4)
        sim['marker']=True;wait(lambda:rows[-1].get('phase')=='DESCEND','重获目标后重新稳定下降',8)
        wait(lambda:rows[-1].get('phase')=='TERMINAL_DESCEND','非零原点相对标记高度进入终端',20)
        sim['marker']=False
        wait(lambda:rows[-1].get('phase') in ('LAND_HANDOFF','AUTO_LAND'),'冻结目标有界终端交接',20)
        wait(lambda:rows[-1].get('phase')=='COMPLETE','落地且锁桨才完成',10)
        check(not any(k=='arm' for k,v in requests),'全流程未直接请求锁桨')
        check(any(k=='mode' and v=='AUTO.LAND' for k,v in requests),'着陆仅调用模拟AUTO.LAND服务')
        print('PRIVATE ROS ARUCO INTEGRATION PASSED',flush=True)
    finally:
        run.clear()
        if 'thread' in locals():thread.join(3)
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for f in files:f.close()
        if args.output:
            args.output.mkdir(parents=True,exist_ok=True);shutil.copytree(root,args.output,dirs_exist_ok=True)
            (args.output/'status.json').write_text(json.dumps(rows,indent=2));(args.output/'requests.json').write_text(json.dumps(requests,indent=2))
        temp.cleanup()
if __name__=='__main__':main()
