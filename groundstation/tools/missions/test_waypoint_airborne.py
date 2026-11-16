#!/usr/bin/env python3
"""真实航点状态机 + 私有本机ROS master + 模拟MAVROS，绝不连接实机。
参数：已用假GCU接口链接的waypoint_mission可执行文件。
"""
import json, os, signal, socket, subprocess, sys, tempfile, threading, time
from pathlib import Path

def main():
    binary=str(Path(sys.argv[1]).resolve());temp=tempfile.TemporaryDirectory(prefix='gcs-map-native-')
    root=Path(temp.name);sock=socket.socket();sock.bind(('127.0.0.1',0));port=sock.getsockname()[1];sock.close()
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d'%port,ROS_IP='127.0.0.1',ROS_LOG_DIR=str(root/'ros'),GCS_TEST_GIMBAL=str(root/'gimbal.txt'))
    os.environ.pop('ROS_HOSTNAME',None)
    import rospy, rosgraph
    from mavros_msgs.msg import State,RCIn,ExtendedState,PositionTarget
    from mavros_msgs.srv import CommandBool,CommandBoolResponse,SetMode,SetModeResponse
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    processes=[];logs=[];run=threading.Event();run.set();sim={'mode':'POSCTL','armed':True,'slot':1494,'x':0.,'z':1.,'airborne':2}
    requests=[];commands=[];statuses=[];photos=[];current={'command':None}
    def spawn(args,name):
        log=open(root/(name+'.log'),'w');logs.append(log)
        p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
    def check(value,why):
        if not value:raise AssertionError(why)
        print('PASS',why,flush=True)
    def wait(predicate,why,seconds=10):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if predicate():return
            time.sleep(.03)
        raise AssertionError(why+'; recent='+repr(statuses[-3:]))
    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
    def raw(m):commands.append((time.monotonic(),m));current['command']=m
    def mode(req):requests.append(('mode',req.custom_mode));sim['mode']=req.custom_mode;return SetModeResponse(mode_sent=True)
    def arm(req):requests.append(('arm',req.value));sim['armed']=req.value;return CommandBoolResponse(success=True,result=0)
    try:
        spawn(['roscore','-p',str(port)],'master');wait(rosgraph.is_master_online,'私有ROS master')
        rospy.init_node('gcs_map_fixture',disable_signals=True)
        rospy.Service('/mavros/set_mode',SetMode,mode);rospy.Service('/mavros/cmd/arming',CommandBool,arm)
        state=rospy.Publisher('/mavros/state',State,queue_size=1);rc=rospy.Publisher('/mavros/rc/in',RCIn,queue_size=1)
        ext=rospy.Publisher('/mavros/extended_state',ExtendedState,queue_size=1);pose=rospy.Publisher('/mavros/local_position/pose',PoseStamped,queue_size=1)
        odom=rospy.Publisher('/Odometry',Odometry,queue_size=1);ack=rospy.Publisher('/waypoint_mission/photo_done',String,queue_size=1)
        lio=rospy.Publisher('/factory_fastlio/odometry',Odometry,queue_size=1)
        rospy.Subscriber('/mavros/setpoint_raw/local',PositionTarget,raw)
        rospy.Subscriber('/waypoint_mission/status',String,lambda m:statuses.append((time.monotonic(),m.data)))
        def photo(m):photos.append((time.monotonic(),m.data));ack.publish(String(data='OK '+m.data))
        rospy.Subscriber('/waypoint_mission/photo_trigger',String,photo)
        def feed():
            while run.is_set():
                now=rospy.Time.now();s=State(connected=True,armed=sim['armed'],mode=sim['mode']);s.header.stamp=now;state.publish(s)
                r=RCIn(rssi=255,channels=[1494]*18);r.channels[4]=sim['slot'];r.header.stamp=now;rc.publish(r)
                ext.publish(ExtendedState(landed_state=sim['airborne']))
                c=current['command']
                if c and sim['mode']=='OFFBOARD':
                    sim['x']+=c.velocity.x*.02;sim['z']+=c.velocity.z*.02
                p=PoseStamped();p.header.stamp=now;p.header.frame_id='map';p.pose.position.x=sim['x'];p.pose.position.z=sim['z'];p.pose.orientation.w=1;pose.publish(p)
                o=Odometry();o.header=p.header;o.pose.pose=p.pose;odom.publish(o);lio.publish(o);time.sleep(.02)
        thread=threading.Thread(target=feed,daemon=True);thread.start()
        # 实际辅助程序的只读状态链路，不调用start或任何飞控服务。
        helper=Path(__file__).with_name('task_manager.py')
        status=subprocess.run([sys.executable,str(helper),'status','--store',str(root/'manager')],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=8)
        data=json.loads(status.stdout.splitlines()[-1])
        check(data['state']['map_session']['pid']==os.getpid(),'真实ROS发布节点身份进入只读任务监视')
        def mission(manual=True, auto_land=False, return_to_start=False):
            cfg={'name':'isolated','manual_airborne':manual,'frame':'local','takeoff_z':1.,'max_z_speed':.25,'max_z_speed_up':.5,'prestream_seconds':.15,'stable_seconds':.06,'arrival_tolerance':.08,'z_tolerance':.08,'gimbal_settle':.1,'return_to_start':return_to_start,'auto_land':auto_land,'rc':{'channel':5,'offboard_slot':6,'min':1075.,'trim':1494.,'max':1913.,'ready_mode':'POSCTL'},'waypoints':[{'x':.6 if return_to_start else .12,'y':0.,'z':1.,'hold':2.,'photo':True,'gimbal_pitch':-90,'gimbal_yaw':0,'yaw':'hold'}]}
            path=root/'mission.yaml';path.write_text(json.dumps({'mission':cfg}));return path
        sim.update(armed=False,airborne=1)
        path=mission();p=spawn([binary,'_mission_file:='+str(path),'_log_dir:='+str(root/'logs'),'_photo_dir:='+str(root/'photos')],'airborne')
        wait(lambda:statuses,'空中模式节点启动');time.sleep(.7)
        check(not commands and not requests and not any('gate_ready=1' in s for _,s in statuses),'空中模式不会从锁桨地面状态自动起飞')
        sim.update(armed=True,airborne=2)
        wait(lambda:any('gate_ready=1' in s for _,s in statuses),'手动飞行时可准备遥控触发')
        time.sleep(.4);check(not commands and not requests,'切OFFBOARD前没有设定点、解锁或模式请求')
        sim['slot']=1913
        wait(lambda:any('phase=GOTO' in s for _,s in statuses),'OFFBOARD新边沿开始航点')
        check(not any(k=='arm' for k,v in requests),'空中任务没有请求解锁')
        check(not any('phase=TAKEOFF' in s for _,s in statuses),'空中任务跳过自动起飞')
        wait(lambda:photos,'航点触发拍照',12)
        hold=[t for t,s in statuses if 'phase=HOLD ' in s]
        check(hold and photos[0][0]-min(hold)>=1.8,'每点停留2秒后拍照')
        check((root/'gimbal.txt').read_text().strip()=='0 -90 0','拍照前云台朝下90度')
        wait(lambda:any('phase=HOLD_COMPLETE' in s for _,s in statuses),'末点完成后悬停')
        check(not any(v=='AUTO.LAND' for k,v in requests),'任务完成没有自动降落请求')
        sim.update(mode='POSCTL',slot=1494);wait(lambda:any('phase=ABORT' in s for _,s in statuses),'遥控切回POSCTL接管')
        time.sleep(.3);count=len(commands);time.sleep(.3);check(len(commands)==count,'接管后停止任务设定点')
        stop(p);statuses.clear();commands.clear();requests.clear();current['command']=None
        sim.update(mode='POSCTL',armed=False,airborne=1,x=0.,z=0.,slot=1494)
        path=mission(False, True, True);p=spawn([binary,'_mission_file:='+str(path),'_log_dir:='+str(root/'legacy'),'_photo_dir:='+str(root/'legacy-photos')],'legacy')
        wait(lambda:any('gate_ready=1' in s for _,s in statuses),'地面锁桨时准备遥控触发')
        time.sleep(.4)
        check(not commands and not requests,'自动起飞任务在遥控触发前无设定点、解锁或模式请求')
        sim['slot']=1913
        wait(lambda:any(k=='arm' and v for k,v in requests),'OFFBOARD边沿后请求自动解锁')
        wait(lambda:any('phase=TAKEOFF' in s for _,s in statuses),'进入原地垂直起飞阶段')
        wait(lambda:sim['z']>.4,'实际向首点高度爬升',12)
        check(abs(sim['x'])<.01,'垂直起飞期间保持起点水平位置')
        check(any(c.velocity.z>.25 for _,c in commands),'独立上升限速允许超过原0.25m/s')
        check(all(-.25-1e-9<=c.velocity.z<=.5+1e-9 for _,c in commands),'上升0.5m/s与下降0.25m/s分别限幅')
        wait(lambda:any('phase=GOTO' in s for _,s in statuses),'达到起飞高度后才开始第一航点',15)
        check(sim['z']>=.9,'开始第一航点前达到目标高度容差')
        check(any(k=='mode' and v=='OFFBOARD' for k,v in requests),'遥控触发后完成OFFBOARD模式请求')
        wait(lambda:any('phase=RETURN' in s for _,s in statuses),'末点完成后进入返航',25)
        check(sim['x']>.4,'从远离起点的末点开始返航')
        wait(lambda:any(k=='mode' and v=='AUTO.LAND' for k,v in requests),'返回起点后请求AUTO.LAND',20)
        check(abs(sim['x'])<.1,'抵达起点附近后才请求自动降落')
        wait(lambda:any('phase=WAIT_DISARM' in s for _,s in statuses),'飞控确认降落模式后等待锁桨')
        time.sleep(.3);count=len(commands);time.sleep(.3);check(len(commands)==count,'降落模式接管后停止OFFBOARD设定点')
        sim.update(armed=False,airborne=1)
        wait(lambda:any('phase=DONE' in s for _,s in statuses),'飞控锁桨后任务完成')
        stop(p)
        print('PRIVATE ROS TEST PASSED',flush=True)
    except Exception:
        print('EVIDENCE',root,flush=True)
        for p in root.glob('*.log'):print(p.name,p.read_text(errors='replace')[-2000:])
        # 保存失败证据，临时目录仍通过固定路径复制。
        import shutil
        shutil.copytree(root,'/tmp/gcs-map-native-failure',dirs_exist_ok=True)
        raise
    finally:
        run.clear()
        for p in reversed(processes):stop(p)
        for f in logs:f.close()
        temp.cleanup()
if __name__=='__main__':main()
