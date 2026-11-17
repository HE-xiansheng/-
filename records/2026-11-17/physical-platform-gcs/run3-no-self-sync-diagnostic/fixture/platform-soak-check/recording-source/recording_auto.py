#!/usr/bin/python3
"""可选自动录制触发器：只读 MAVROS，复用现有录制管理器，不发飞控命令。"""
import fcntl,json,os,subprocess,sys,time,hashlib
from pathlib import Path
import recording_manage as manager
from record_session import FlightTrigger,atomic
ROOT=manager.ROOT
ACTIVE='.flight-recording-auto.json'
SETTINGS='.flight-recording-auto-settings.json'
STATUS='.flight-recording-auto-status.json'


def enabled(root=ROOT):
    try:return json.loads((root/SETTINGS).read_text()).get('enabled') is True
    except (OSError,ValueError):return False


def current(root=ROOT):
    try:
        d=json.loads((root/ACTIVE).read_text());tick,args=manager.identity(d['pid'])
        if d.get('boot_id')==Path('/proc/sys/kernel/random/boot_id').read_text().strip() and tick==d['start_tick'] and os.fsencode(str(root/'recording_auto.py')) in args and b'worker' in args:return d
    except (OSError,ValueError,KeyError):pass
    return None


def configure(value,root=ROOT):
    with (root/'.flight-recording-auto-management.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        if value:manager.prepare(root)  # 提前快照、准备录包；OFFBOARD边沿再提交开始请求。
        atomic(root/SETTINGS,{'schema':1,'enabled':bool(value),'stamp':time.time()})
        info=current(root)
        if value and not info:
            logs=root/'recording-auto-logs';logs.mkdir(exist_ok=True)
            log=logs/(time.strftime('%Y%m%d-%H%M%S')+'.log')
            with log.open('ab') as out:
                proc=subprocess.Popen(['/usr/bin/python3','-B',str(root/'recording_auto.py'),'worker'],stdin=subprocess.DEVNULL,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
            tick,_=manager.identity(proc.pid)
            atomic(root/ACTIVE,{'boot_id':manager.boot(),'pid':proc.pid,'start_tick':tick,'log':str(log)})
    return manager.health(root)


class AutoPolicy:
    """模式退出保持录制；仅新鲜的落地+未解锁观测能自动结束。"""
    def __init__(self):self.last_eligible=False;self.owned_run=None;self.last_clip=None;self.ground_since=None
    def decide(self,option,flight,recording):
        if not flight.get('fresh') or not flight.get('connected'):
            self.ground_since=None;return None
        grounded=not flight.get('armed') and flight.get('landed_state')==1
        if not grounded:self.ground_since=None
        elif self.ground_since is None:self.ground_since=time.monotonic()
        eligible=bool(option and flight.get('armed') and flight.get('mode')=='OFFBOARD')
        edge=eligible and not self.last_eligible;self.last_eligible=eligible
        phase=recording.get('phase');run=recording.get('run')
        if self.owned_run and (run!=self.owned_run or phase in ('STOPPED','ERROR')):
            self.owned_run=None;self.last_clip=None
        if self.owned_run and grounded and self.ground_since is not None and time.monotonic()-self.ground_since>=.6:
            if phase in ('RECORDING','STARTING_BAG','PREPARING') and recording.get('pending_action')!='stop':return 'stop'
        if edge and not self.owned_run:
            if phase in ('RECORDING','STARTING_BAG'):
                self.owned_run=run;self.last_clip=recording.get('clip_id');return None
            if phase not in ('STOPPING',) and not recording.get('pending_action'):return 'start'
        return None
    def accepted(self,action,result):
        if action=='start':self.owned_run=result.get('run');self.last_clip=result.get('clip_id')


def worker(root=ROOT):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from mavros_msgs.msg import State,ExtendedState
    with (root/'.flight-recording-auto-worker.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        rclpy.init();node=rclpy.create_node('ego_planner_auto_record_observer',enable_rosout=False)
        observed=FlightTrigger();policy=AutoPolicy()
        subs=[node.create_subscription(State,'/mavros/state',observed.on_state,qos_profile_sensor_data),node.create_subscription(ExtendedState,'/mavros/extended_state',observed.on_landed,qos_profile_sensor_data)]
        try:
            while rclpy.ok():
                rclpy.spin_once(node,timeout_sec=.05)
                for _ in range(32):rclpy.spin_once(node,timeout_sec=0.)
                # 消耗同一批状态/落地回调，避免旧landed与新armed拼成误触发。
                for _ in range(32):rclpy.spin_once(node,timeout_sec=0.)
                _,flight=observed.check(node.get_clock().now().nanoseconds*1e-9);option=enabled(root)
                try:
                    state=manager.health(root);action=policy.decide(option,flight,state)
                    if action:
                        result=manager.request(action,root,reuse_prepared=True);policy.accepted(action,result)
                        if result.get('run'):
                            with (Path(result['run'])/'automatic-recording-events.jsonl').open('a') as log:
                                log.write(json.dumps({'time':time.time(),'action':action,'flight':flight,'request_id':result.get('request_id'),'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},ensure_ascii=False)+'\n')
                    phase='WAIT_LANDING' if policy.owned_run else 'ARMED' if option else 'DISABLED'
                    atomic(root/STATUS,{'stamp':time.time(),'phase':phase,'enabled':option,'owned_run':policy.owned_run,'flight':flight})
                except Exception as exc:atomic(root/STATUS,{'stamp':time.time(),'phase':'ERROR','enabled':option,'error':str(exc)})
                time.sleep(.2)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__=='__main__':
    if sys.argv[1:]!=['worker']:raise SystemExit('worker')
    worker()
