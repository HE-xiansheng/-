#!/usr/bin/env python3
"""SSH stdin 助手：只编排既有定位/转发入口，不安装源码或改变坐标、飞控策略。"""
import contextlib
import fcntl
import hashlib
import importlib.util
import json
import math
import subprocess
import sys
import time
from pathlib import Path

BOARD_SHA='85b5a32d1d187e45ce06735c08a09d474db7f3584d26c7f39315221ff96fe541'
RELAY_SHA='60d08d55a3e56236a72ad39567e79dfb1dd9321eaaaeed7fb72cfedfa75be90d'
RELAY_TOPICS=['/mavros/vision_pose/pose','/mavros/vision_pose/pose_cov','/mavros/odometry/out','/mavros/vision_speed/speed_twist']


def operate(action, api):
    """可测试的操作顺序；存活会话复用，停止不连带结束驱动、MAVROS、录制。"""
    api.gate()
    lio,relay=api.inventory()  # 同时拒绝陌生来源/失配 PID，先核实再修改。
    if action=='start':
        if not lio or not relay:api.require_idle()
        if not lio:
            if relay:raise RuntimeError('转发仍在运行而定位已退出，请先停止定位与转发，再启动新会话')
            api.start_lio()
        api.wait_source()
        if not relay:api.start_relay()
        return {'ok':True,'action':action,'message':'定位与转发已启动，等待实时数据确认' if not(lio and relay) else '已复用当前定位与转发'}
    if action=='stop':
        api.require_idle()
        if relay:api.stop_relay()
        if lio:api.stop_lio()
        return {'ok':True,'action':action,'message':'定位与转发已停止'}
    raise ValueError('未知定位操作')


class ExistingStack:
    def __init__(self,root,ws):
        self.root,self.ws=root,ws
        for filename,digest in [('board.py',BOARD_SHA),('flight-pose-relay.py',RELAY_SHA)]:
            if hashlib.sha256((root/'management'/filename).read_bytes()).hexdigest()!=digest:
                raise RuntimeError('原定位管理源码校验不一致，未执行：'+filename)
        sys.path.insert(0,str(root/'management'))
        import board
        self.board=board
        spec=importlib.util.spec_from_file_location('original_px4_manager',ws/'px4_execution/manage.py')
        self.execution=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.execution)

    def gate(self):
        state,_=self.board.gate()
        rospy,_=self.board.ros()
        age=(rospy.Time.now()-state.header.stamp).to_sec()
        if not math.isfinite(age) or not -.1<=age<=1.5 or state.mode=='OFFBOARD':
            raise RuntimeError('需要新鲜的已连接、未解锁、非OFFBOARD飞控状态')

    def relay(self):
        try:
            data=json.loads((self.root/'active-flight-pose-relay.json').read_text());p=Path('/proc')/str(int(data['pid']))
            args=p.joinpath('cmdline').read_bytes().split(b'\0')
            wanted=(Path.home()/'drone_platform/02_drivers/sensors/lib/to_mavros/vision2mavros').resolve()
            return data if p.joinpath('exe').resolve()==wanted and b'__name:=factory_vision2mavros' in args else None
        except (OSError,ValueError,KeyError):return None

    def inventory(self):
        pubs=self.board.graph();lio=self.board.alive('localization');relay=bool(self.relay())
        if pubs.get(self.board.ODOM):
            self.board.verify_source()
            if not lio:raise RuntimeError('定位不属于当前管理会话，未接管或停止其他进程')
        for topic in RELAY_TOPICS:
            expected=['/factory_vision2mavros'] if topic=='/mavros/vision_pose/pose' else []
            if pubs.get(topic) and (not relay or pubs[topic]!=expected):
                raise RuntimeError('存在其他定位转发，未自动替换：'+topic+' '+str(pubs[topic]))
        if pubs.get('/Odometry'):raise RuntimeError('存在另一定位来源 /Odometry，未启动或替换')
        if lio:
            # 原管理器检查路径；补充验证 launch 命令和进程组，避免误用旧 PID。
            d=json.loads(self.board.active_file('localization').read_text());p=Path('/proc')/str(d['pid'])
            args=p.joinpath('cmdline').read_bytes().split(b'\0')
            launch=str(Path(d['run'])/'localization.launch').encode()
            import os
            if launch not in args or str(self.root/'runs').encode() not in launch or os.getpgid(d['pid'])!=d['pid']:
                raise RuntimeError('定位管理进程身份不匹配，未操作')
        return lio,relay

    def require_idle(self):
        if self.execution.current():raise RuntimeError('请先停止避障并等待执行器退出，再启停定位与转发')
        pubs=self.board.graph()
        if any(pubs.get(t) for t in ['/ego_planner/manual_preview/status','/drone_0_planning/trajectory']):
            raise RuntimeError('请先停止避障预览，再启停定位与转发')
        # 与 manual_session.py 相同的进程身份；持有同一把锁，防止同时启动预览。
        try:
            d=json.loads((self.ws/'.manual-preview.json').read_text());p=Path('/proc')/str(d['pid'])
            tick=p.joinpath('stat').read_text().rsplit(')',1)[1].split()[19]
            args=p.joinpath('cmdline').read_bytes()
            launch=str(self.ws/'src/ego_planner/plan_manage/launch/realflight/run_planner_board.launch').encode()
            if tick==d['start_tick'] and launch in args and b'manual_preview:=true' in args:
                raise RuntimeError('请先停止避障预览，再启停定位与转发')
        except (OSError,ValueError,KeyError,IndexError):pass

    def start_lio(self):
        self.gate();self.board.start()

    def wait_source(self):
        rospy,_=self.board.ros()
        from nav_msgs.msg import Odometry
        deadline=time.monotonic()+25;source_checked=False
        while True:
            if not self.board.alive('localization'):
                raise RuntimeError('FAST-LIO 启动后已退出，请检查原定位进程日志；未启动新转发')
            try:
                m=rospy.wait_for_message(self.board.ODOM,Odometry,timeout=min(2,max(.1,deadline-time.monotonic())))
                if not source_checked:
                    # 原 verify_source 会等待下一条 FC 状态并核验二进制；核验后重新采样，
                    # 不能把核验前的消息按核验后的时间判断为陈旧。
                    self.board.verify_source();source_checked=True
                    continue
                break
            except rospy.ROSException:
                if time.monotonic()>=deadline:
                    raise RuntimeError('定位尚无新鲜输出：请检查雷达点云和IMU，并保持机体静止；定位进程保留，未启动新转发')
        p=m.pose.pose.position;q=m.pose.pose.orientation
        xyzq=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        if (m.header.frame_id!='camera_init' or m.child_frame_id!='body' or not all(math.isfinite(x) for x in xyzq)
            or not .81<=sum(x*x for x in xyzq[3:])<=1.21 or not -.1<=(rospy.Time.now()-m.header.stamp).to_sec()<=.4):
            raise RuntimeError('定位坐标系或数据无效/过期，未启动新转发')

    def relay_command(self,action):
        self.gate()
        result=subprocess.run(['/usr/bin/python3',str(self.root/'management/flight-pose-relay.py'),action],capture_output=True,text=True,timeout=12)
        if result.returncode:raise RuntimeError((result.stdout+result.stderr).strip()[-1200:])

    def start_relay(self):self.relay_command('start')

    def stop_relay(self):
        self.relay_command('stop')
        deadline=time.monotonic()+6
        while self.relay() and time.monotonic()<deadline:time.sleep(.1)
        if self.relay():raise RuntimeError('转发尚未退出，保留定位；请检查原转发日志')

    def stop_lio(self):
        self.gate();self.board.stop_kind('localization')
        deadline=time.monotonic()+3
        while self.board.alive('localization') and time.monotonic()<deadline:time.sleep(.1)
        if self.board.alive('localization'):raise RuntimeError('定位尚未退出，请检查原定位日志')


def main():
    action=sys.argv[1]
    if action not in ('start','stop'):raise ValueError('start / stop')
    root=Path.home()/'drone_platform/01_apps/fast_lio';ws=Path.home()/'drone_platform/01_apps/ego_planner'
    # 与所有原入口共享锁；不等待其他生命周期操作，避免锁顺序死锁。
    with contextlib.ExitStack() as stack:
        for file in [root/'.manager.lock',root/'.pose-relay-lifecycle.lock',ws/'.px4-execution-management.lock',ws/'.manual-preview.lock']:
            lock=stack.enter_context(file.open('a'))
            try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:raise RuntimeError('已有定位、预览或执行器管理操作，请稍后重试')
        print(json.dumps(operate(action,ExistingStack(root,ws)),ensure_ascii=False),flush=True)


if __name__=='__main__':
    try:main()
    except Exception as exc:
        print(json.dumps({'ok':False,'message':str(exc)},ensure_ascii=False),flush=True)
        raise SystemExit(1)
