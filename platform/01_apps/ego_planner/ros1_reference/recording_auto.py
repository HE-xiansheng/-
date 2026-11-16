#!/usr/bin/python3
"""可选自动录制触发器：只读 MAVROS，复用现有录制管理器，不发飞控命令。"""
import fcntl,json,os,subprocess,sys,time,hashlib
from pathlib import Path
import recording_manage as manager
from record_session import FlightTrigger,atomic
ROOT=Path(__file__).resolve().parent
ACTIVE='.flight-recording-auto.json'
SETTINGS='.flight-recording-auto-settings.json'
STATUS='.flight-recording-auto-status.json'


def enabled(root=ROOT):
    try:return json.loads((root/SETTINGS).read_text()).get('enabled') is True
    except (OSError,ValueError):return False


def current(root=ROOT):
    try:
        d=json.loads((root/ACTIVE).read_text());tick,args=manager.identity(d['pid'])
        if tick==d['start_tick'] and os.fsencode(str(root/'recording_auto.py')) in args and b'worker' in args:return d
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
            atomic(root/ACTIVE,{'pid':proc.pid,'start_tick':tick,'log':str(log)})
    return manager.health(root)


class AutoPolicy:
    """模式退出保持录制；仅新鲜的落地+未解锁观测能自动结束。"""
    def __init__(self):self.last_eligible=False;self.owned_run=None;self.last_clip=None
    def decide(self,option,flight,recording):
        if not flight.get('fresh') or not flight.get('connected'):return None
        eligible=bool(option and flight.get('armed') and flight.get('mode')=='OFFBOARD')
        edge=eligible and not self.last_eligible;self.last_eligible=eligible
        phase=recording.get('phase');run=recording.get('run')
        if self.owned_run and (run!=self.owned_run or phase in ('STOPPED','ERROR')):
            self.owned_run=None;self.last_clip=None
        if self.owned_run and not flight.get('armed') and flight.get('landed_state')==1:
            if phase in ('RECORDING','STARTING_BAG','PREPARING') and recording.get('pending_action')!='stop':return 'stop'
        if edge and not self.owned_run:
            if phase in ('RECORDING','STARTING_BAG'):
                self.owned_run=run;self.last_clip=recording.get('clip_id');return None
            if phase not in ('STOPPING',) and not recording.get('pending_action'):return 'start'
        return None
    def accepted(self,action,result):
        if action=='start':self.owned_run=result.get('run');self.last_clip=result.get('clip_id')


def worker(root=ROOT):
    import rospy
    from mavros_msgs.msg import State,ExtendedState
    with (root/'.flight-recording-auto-worker.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        rospy.init_node('ego_planner_auto_record_observer',anonymous=True,disable_signals=True,disable_rosout=True)
        observed=FlightTrigger();policy=AutoPolicy()
        subs=[rospy.Subscriber('/mavros/state',State,observed.on_state,queue_size=1),rospy.Subscriber('/mavros/extended_state',ExtendedState,observed.on_landed,queue_size=1)]
        try:
            while not rospy.is_shutdown():
                _,flight=observed.check(rospy.Time.now().to_sec());option=enabled(root)
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
            for sub in subs:sub.unregister()


if __name__=='__main__':
    if sys.argv[1:]!=['worker']:raise SystemExit('worker')
    worker()
