#!/usr/bin/env python3
"""管理原record_session的显式录制请求；不操作定位/规划/飞控。"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

ROOT=Path(os.environ.get('RECORDING_ROOT', str(Path(__file__).resolve().parent))).resolve()
ACTIVE='.flight-recording.json'


def atomic(path,value):
    from record_session import atomic as write
    write(path,value)


def boot():return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def identity(pid):
    try:
        p=Path('/proc')/str(pid);fields=p.joinpath('stat').read_text().split(') ',1)[1].split()
        return fields[19] if fields[0]!='Z' else None,p.joinpath('cmdline').read_bytes().split(b'\0')
    except (OSError,ValueError,IndexError):return None,[]


def current(root=ROOT):
    try:
        data=json.loads((root/ACTIVE).read_text());tick,args=identity(data['pid'])
        if data.get('boot_id')==Path('/proc/sys/kernel/random/boot_id').read_text().strip() and tick==data['start_tick'] and os.fsencode(str(root/'record_session.py')) in args and os.fsencode(data['run']) in args:return data
    except (OSError,ValueError,KeyError):pass
    return None


def _record_health(root=ROOT):
    info=current(root)
    if info:
        try:data=json.loads((Path(info['run'])/'recording-status.json').read_text())
        except (OSError,ValueError):data={'phase':'PREPARING','policy':'manual'}
        age=time.time()-data.get('stamp',time.time())
        limit=150 if data.get('phase')=='PREPARING' else 60 if data.get('phase')=='STOPPING' else 5
        try:
            control=json.loads((Path(info['run'])/'recording-control.json').read_text())
            pending=control.get('action') if control.get('id')!=data.get('request_id') else None
        except (OSError,ValueError):pending=None
        return dict(data,running=True,worker_pid=info['pid'],run=info['run'],status_age_s=age,status_fresh=0<=age<=limit,pending_action=pending)
    try:
        old=json.loads((root/ACTIVE).read_text());data=json.loads((Path(old['run'])/'recording-status.json').read_text())
        if data.get('phase') in ('RECORDING','STARTING_BAG','STOPPING','PREPARING','WAIT_MANUAL'):
            data.update(phase='ERROR',error='录制进程已退出；请检查录包文件')
        return dict(data,running=False,status_fresh=True,bag_pid=None,active_bags=[str(x) for x in Path(old['run']).glob('*.active')],run=old['run'])
    except (OSError,ValueError,KeyError):return {'phase':'NOT_RECORDING','policy':'manual','running':False,'status_fresh':True,'duration_s':0.}


def health(root=ROOT):
    result=_record_health(root)
    try:settings=json.loads((root/'.flight-recording-auto-settings.json').read_text())
    except (OSError,ValueError):settings={}
    alive=False;fresh=False;status={}
    try:
        info=json.loads((root/'.flight-recording-auto.json').read_text());tick,args=identity(info['pid'])
        alive=info.get('boot_id')==Path('/proc/sys/kernel/random/boot_id').read_text().strip() and tick==info['start_tick'] and os.fsencode(str(root/'recording_auto.py')) in args and b'worker' in args
        status=json.loads((root/'.flight-recording-auto-status.json').read_text());fresh=alive and 0<=time.time()-status['stamp']<=5
    except (OSError,ValueError,KeyError):pass
    return dict(result,auto_enabled=settings.get('enabled') is True,auto_running=alive,auto_fresh=fresh,auto_phase=status.get('phase','UNAVAILABLE'),auto_error=status.get('error',''))


def planner_owner(root):
    runtime=Path(os.environ.get('EGO_PREVIEW_RUNTIME_DIR','/tmp/ego-manual-preview-%d'%os.getuid()))
    try:
        owner=json.loads((runtime/'active.json').read_text());tick,args=identity(owner['pid'])
        if tick==owner['start_tick'] and os.fsencode(str(root/'launch/manual_preview.launch.py')) in args:return owner
    except (OSError,ValueError,KeyError):pass
    return None


def stop_idle(info,root):
    state=health(root)
    if state.get('bag_pid') or list(Path(info['run']).glob('*.active')):raise RuntimeError('存在未收尾录包，请检查当前录制，不创建重复实例')
    if state.get('phase') not in ('WAIT_MANUAL','STOPPED','ERROR'):raise RuntimeError('录制器仍在工作，未替换其上下文')
    os.kill(info['pid'],signal.SIGTERM)
    deadline=time.monotonic()+6
    while time.monotonic()<deadline:
        tick,_=identity(info['pid'])
        if tick!=info['start_tick']:return
        time.sleep(.1)
    raise RuntimeError('闲置录制器尚未退出，未启动重复实例')


def prepare_locked(root,preferred_run=None):
    info=current(root)
    if info:return info
    old=health(root)
    if old.get('run') and (old.get('bag_pid') or list(Path(old['run']).rglob('*.active'))):
        raise RuntimeError('仍有未确认收尾的录制，不创建新实例')
    owner=planner_owner(root)
    if owner and owner.get('recorder_pid'):
        tick,args=identity(owner['recorder_pid'])
        if tick and os.fsencode(str(root/'record_session.py')) in args:
            raise RuntimeError('旧录制器仍在运行，请先迁移手动录制策略')
    run=Path(preferred_run) if preferred_run else root/'flight-records'/(time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])
    run.mkdir(parents=True,exist_ok=True)
    atomic(run/'recording-status.json',{'schema':2,'policy':'manual','phase':'PREPARING','stamp':time.time(),'run':str(run),'duration_s':0.})
    with (run/'supervisor.log').open('ab') as log:
        proc=subprocess.Popen(['/usr/bin/python3','-B',str(root/'record_session.py'),'--root',str(root),'--run',str(run)],
            stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    tick,_=identity(proc.pid)
    if tick is None:raise RuntimeError('录制准备进程启动失败，请查看supervisor.log')
    info={'boot_id':boot(),'pid':proc.pid,'start_tick':tick,'run':str(run),'policy':'manual'}
    atomic(root/ACTIVE,info);return info


def prepare(root=ROOT):
    with (root/'.flight-recording-management.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        info=current(root)
        if info and health(root).get('phase') in ('WAIT_MANUAL','STOPPED','ERROR') and not health(root).get('pending_action') and not planner_owner(root):
            stop_idle(info,root)
        return prepare_locked(root)


def request(action,root=ROOT,reuse_prepared=False):
    with (root/'.flight-recording-management.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        info=current(root);state=health(root)
        if action=='stop':
            if not info:return state
        else:
            if state.get('run') and (state.get('bag_pid') or list(Path(state['run']).glob('*.active'))) and state.get('phase') not in ('RECORDING','STARTING_BAG','STOPPING'):
                raise RuntimeError('存在未收尾录包，请先检查文件，不开始新录制')
            if info and state.get('pending_action')=='start':return state
            if info and state.get('phase') in ('RECORDING','STARTING_BAG','STOPPING'):
                if state['phase']=='STOPPING':raise RuntimeError('正在保存，请完成后再开始下一段')
                return state
            if not reuse_prepared and info and state.get('phase') in ('WAIT_MANUAL','STOPPED','ERROR') and not planner_owner(root):
                stop_idle(info,root);info=None
            info=info or prepare_locked(root)
        control={'id':uuid.uuid4().hex,'action':action,'stamp':time.time()}
        atomic(Path(info['run'])/'recording-control.json',control)
        return dict(health(root),requested=action,request_id=control['id'])


def migrate_idle(root=ROOT):
    raise RuntimeError('ROS2录制器不替换ROS1进程；请在独立ROS2环境启动')


def summary(value):
    keys=('phase','policy','running','status_fresh','status_age_s','stamp','run','clip_id','request_id','requested',
          'auto_enabled','auto_running','auto_fresh','auto_phase','auto_error','pending_action','duration_s','bag_pid','bag_bytes','free_bytes','error','recording_started_wall','storage','missing_topics')
    result={k:value[k] for k in keys if k in value}
    result['warnings']=value.get('warnings',[])[-3:]
    result['active_bags_count']=len(value.get('active_bags',[]))
    result['available']=True
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['start','stop','status','prepare','migrate-idle','auto-on','auto-off']);a=p.parse_args()
    if a.action in ('auto-on','auto-off'):
        import recording_auto
        result=recording_auto.configure(a.action=='auto-on')
    elif a.action=='status':result=health()
    elif a.action=='prepare':result=dict(prepare(),phase='PREPARING')
    elif a.action=='migrate-idle':result=migrate_idle()
    else:result=request(a.action)
    print(json.dumps(summary(result),ensure_ascii=False,allow_nan=False))


if __name__=='__main__':
    try:main()
    except Exception as exc:raise SystemExit(str(exc))
