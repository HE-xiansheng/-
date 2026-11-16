#!/usr/bin/env python3
"""Start one board-local adapter, or request a controlled stop. Never kill in flight."""
import argparse,fcntl,json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
WS=ROOT.parent
ACTIVE=WS/'.px4-execution.json'

def recording_health():
    # 只读独立手动录制状态，不改变任何执行器控制门槛。
    try:
        import importlib.util
        spec=importlib.util.spec_from_file_location('board_recording_manager',WS/'recording_manage.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module.summary(module.health(WS))
    except (OSError,ValueError,KeyError,ImportError):return {'phase':'NOT_RECORDING'}

def identity(pid):
    try:
        s=Path('/proc/%d/stat'%pid).read_text().split(') ',1)[1].split()
        cmd=Path('/proc/%d/cmdline'%pid).read_bytes().replace(b'\0',b' ').decode()
        return s[19],cmd
    except (OSError,IndexError):return None,''

def current():
    try:
        data=json.loads(ACTIVE.read_text());tick,cmd=identity(data['pid'])
        if tick==data['start_tick'] and str(ROOT/'adapter.py') in cmd and '--publish' in cmd:return data
    except (OSError,KeyError,ValueError):pass
    return None

def read_status(info):
    if not info:return {'running':False,'output_enabled':False}
    try:status=json.loads((Path(info['run'])/'status.json').read_text())
    except (OSError,ValueError):status={'phase':'STARTING','output_enabled':False}
    if 'stamp' in status:
        status['status_age_s']=time.time()-status['stamp']
        if status['phase']!='DONE' and status['status_age_s']>.6:
            status['phase']='STATUS_STALE';status['output_enabled']=False
            status['reason']='Execution status is stale; output state cannot be confirmed'
    return dict(status,running=True,pid=info['pid'],run=info['run'])

def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['enable','stop','status','check']);a=p.parse_args()
    with (WS/'.px4-execution-management.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        info=current()
        if a.action=='status':
            if info:print(json.dumps(dict(read_status(info),recording=recording_health()),ensure_ascii=False));return
            result={'running':False,'output_enabled':False,'recording':recording_health()}
            if ACTIVE.exists():
                try:
                    old=json.loads(ACTIVE.read_text());result['last_session']=json.loads((Path(old['run'])/'status.json').read_text())
                except (OSError,ValueError,KeyError):pass
            print(json.dumps(result,ensure_ascii=False));return
        if a.action=='stop':
            if info:
                (Path(info['run'])/'stop-request').touch()
                time.sleep(.3)
                print(json.dumps(read_status(current()),ensure_ascii=False))
                print('已请求停止跟踪；若仍在OFFBOARD，将保持位置，请遥控切回POSCTL后结束输出。')
            else:print('执行适配器未运行，无控制输出。')
            return
        if info:
            if a.action=='enable':raise RuntimeError('接入已启用；Position下保持待命，可更新目标后再次手动切OFFBOARD；取消勾选才结束接入。')
            raise RuntimeError('已有接入会话；使用status查看，避免混淆检查结果。')
        run=WS/'px4-execution-logs'/time.strftime('%Y%m%d-%H%M%S')
        run.mkdir(parents=True,exist_ok=False)
        cmd=['/usr/bin/python3','-B',str(ROOT/'adapter.py'),'--run',str(run),'--publish' if a.action=='enable' else '--check']
        if a.action=='check':
            result=subprocess.run(cmd,timeout=25);raise SystemExit(result.returncode)
        with (run/'console.log').open('ab') as log:
            proc=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        tick,_=identity(proc.pid)
        data={'pid':proc.pid,'start_tick':tick,'run':str(run)}
        ACTIVE.write_text(json.dumps(data))
        time.sleep(2)
        if proc.poll() is not None:
            raise RuntimeError('执行器启动失败，请查看 '+str(run/'console.log'))
        print(json.dumps(read_status(data),ensure_ascii=False))
        print('已启用等待流程。手动POSCTL起飞后可使用已有目标；遥控切OFFBOARD执行，切回POSCTL立即交回控制。')

if __name__=='__main__':main()
