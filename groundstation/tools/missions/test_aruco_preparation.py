#!/usr/bin/env python3
"""精准降落准备/遥控边界：私有ROS master + 真实降落节点 + 假GCU。
参数：用test-native/gcu_stub.c链接的降落节点、现场takeoff_land.yaml。
"""
import csv, importlib.util, os, signal, socket, subprocess, sys, tempfile, threading, time
from pathlib import Path
import xml.etree.ElementTree as ET

def main():
    binary=str(Path(sys.argv[1]).resolve());config=Path(sys.argv[2]).resolve()
    if b'GCS_TEST_GIMBAL' not in Path(binary).read_bytes():
        raise RuntimeError('仅允许链接GCU桩的测试二进制，禁止运行正式机载程序')
    temp=tempfile.TemporaryDirectory(prefix='gcs-aruco-preparation-');root=Path(temp.name)
    sock=socket.socket();sock.bind(('127.0.0.1',0));port=sock.getsockname()[1];sock.close()
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d'%port,ROS_IP='127.0.0.1',ROS_LOG_DIR=str(root/'ros'),GCS_TEST_GIMBAL=str(root/'gimbal.txt'))
    os.environ.pop('ROS_HOSTNAME',None)
    import rospy,rosgraph,yaml
    from mavros_msgs.msg import State,RCIn,ExtendedState,PositionTarget
    from mavros_msgs.srv import CommandBool,CommandBoolResponse,SetMode,SetModeResponse
    from geometry_msgs.msg import PoseStamped,TwistStamped
    from std_msgs.msg import String
    processes=[];logs=[];run=threading.Event();run.set();sim={'mode':'POSCTL','pwm':1494}
    requests=[];commands=[];rows=[];header=[]
    def check(value,label):
        if not value:raise AssertionError(label)
        print('PASS',label,flush=True)
    def wait(predicate,label,seconds=8):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if predicate():return
            time.sleep(.025)
        raise AssertionError(label+' '+repr(rows[-1:]))
    def spawn(args,name):
        log=open(root/(name+'.log'),'w');logs.append(log)
        p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
    def mode(req):requests.append(('mode',req.custom_mode));sim['mode']=req.custom_mode;return SetModeResponse(mode_sent=True)
    def arm(req):requests.append(('arm',req.value));return CommandBoolResponse(success=True,result=0)
    try:
        spawn(['roscore','-p',str(port)],'master');wait(rosgraph.is_master_online,'私有ROS master启动')
        rospy.init_node('gcs_aruco_preparation_fixture',disable_signals=True)
        values=yaml.safe_load(config.read_text())
        for k,v in values.items():rospy.set_param('/'+k,v)
        # 从实际地面站辅助程序生成的launch读取准备覆盖项。
        spec=importlib.util.spec_from_file_location('task_manager',Path(__file__).with_name('task_manager.py'))
        helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
        for p in ET.fromstring(helper.launch_xml('aruco',root)).findall('param'):
            rospy.set_param(p.attrib['name'],p.attrib['value']=='true')
        rospy.set_param('/aruco_landing/diagnostic_directory',str(root/'diagnostics'))
        check(rospy.get_param('/aruco_landing/start_trigger')=='rc_offboard','保留遥控OFFBOARD触发')
        check(rospy.get_param('/aruco_landing/auto_takeoff') is False,'保留人工起飞配置')
        rospy.Service('/mavros/set_mode',SetMode,mode);rospy.Service('/mavros/cmd/arming',CommandBool,arm)
        state=rospy.Publisher('/mavros/state',State,queue_size=1);rc=rospy.Publisher('/mavros/rc/in',RCIn,queue_size=1)
        ext=rospy.Publisher('/mavros/extended_state',ExtendedState,queue_size=1)
        pose=rospy.Publisher('/mavros/local_position/pose',PoseStamped,queue_size=1);vel=rospy.Publisher('/mavros/local_position/velocity_local',TwistStamped,queue_size=1)
        rospy.Subscriber('/mavros/setpoint_raw/local',PositionTarget,lambda m:commands.append(m))
        rospy.Subscriber('/aruco_landing/status_header',String,lambda m:header.__setitem__(slice(None),next(csv.reader([m.data]))))
        rospy.Subscriber('/aruco_landing/status',String,lambda m:rows.append(dict(zip(header,next(csv.reader([m.data]))))))
        def feed():
            while run.is_set():
                now=rospy.Time.now();s=State(connected=True,armed=True,mode=sim['mode']);s.header.stamp=now;state.publish(s)
                r=RCIn(rssi=255,channels=[1494]*18);r.header.stamp=now;r.channels[4]=sim['pwm'];rc.publish(r)
                e=ExtendedState(landed_state=2);e.header.stamp=now;ext.publish(e)
                p=PoseStamped();p.header.stamp=now;p.header.frame_id='map';p.pose.position.z=1.2;p.pose.orientation.w=1;pose.publish(p)
                v=TwistStamped();v.header=p.header;vel.publish(v);time.sleep(.02)
        thread=threading.Thread(target=feed,daemon=True);thread.start()
        spawn([binary],'landing')
        wait(lambda:(root/'gimbal.txt').is_file(),'准备时发出云台命令')
        check((root/'gimbal.txt').read_text().strip()=='0 -90 0','准备时云台垂直朝下90度')
        wait(lambda:any(r.get('gimbal_status')=='startup_down_acknowledged' for r in rows),'准备云台应答回传')
        time.sleep(.6)
        check(rows[-1].get('phase')=='IDLE' and rows[-1].get('enabled')=='0','准备后降落任务保持未启用')
        check(not commands and not requests,'切OFFBOARD前不发飞行设定点、解锁或模式请求')
        sim['pwm']=1913
        wait(lambda:bool(commands) and any(r.get('enabled')=='1' for r in rows),'遥控OFFBOARD新边沿启用降落控制')
        check(not any(k=='arm' for k,v in requests),'人工起飞模式不请求解锁')
        sim.update(mode='POSCTL',pwm=1494)
        wait(lambda:rows[-1].get('enabled')=='0','切回POSCTL撤销降落任务')
        time.sleep(.25);count=len(commands);time.sleep(.25)
        check(len(commands)==count,'遥控接管后停止降落设定点')
        print('PRIVATE ARUCO PREPARATION TEST PASSED',flush=True)
    except Exception:
        for p in root.glob('*.log'):print(p.name,p.read_text(errors='replace')[-1800:])
        import shutil
        shutil.copytree(root,'/tmp/gcs-aruco-preparation-failure',dirs_exist_ok=True)
        raise
    finally:
        run.clear()
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        for f in logs:f.close()
        temp.cleanup()
if __name__=='__main__':main()
