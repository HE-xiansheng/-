#!/usr/bin/env python3
"""上位机管理本工具自己的 SSH 点云发送会话；机载源码通过 stdin 运行，不安装服务。"""
import argparse
try:
    import fcntl
except ImportError:  # Windows 只运行前台遥测；生命周期由 windows_bridge 管理。
    fcntl = None
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import threading
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'communication'))
from ssh_session import background_ssh

ROOT = Path(__file__).resolve().parents[3]
STATE = ROOT/'.pointcloud-link'


def identity(pid):
    try:
        p = Path('/proc')/str(pid)
        tick = p.joinpath('stat').read_text().rsplit(')',1)[1].split()[19]
        args = p.joinpath('cmdline').read_bytes().split(b'\0')
        return tick,args
    except (OSError,ValueError,IndexError):
        return None,[]


def current():
    try:
        data = json.loads((STATE/'active.json').read_text())
        tick,args = identity(data['pid'])
        if tick == data['start_tick'] and args:
            program = Path(os.fsdecode(args[0])).name
            # 兼容升级前已运行的 SSH 发送器，同时识别新的本机重连管理进程。
            legacy = program == 'ssh' and any(b'dianyun-pointcloud-only' in a for a in args)
            supervised = (program.startswith('python') and os.fsencode(str(Path(__file__).resolve())) in args
                          and b'run' in args)
            if legacy or supervised:
                return data
    except (OSError,ValueError,KeyError):
        pass
    return None


def report(data):
    result = dict(data or {},running=bool(data))
    if data:
        try:
            for line in reversed(Path(data['log']).read_text().splitlines()):
                if line.startswith('{'):
                    status = json.loads(line)
                    result['sender'] = status
                    result['status_age_s'] = time.time()-status['stamp']
                    break
        except (OSError,ValueError,KeyError):
            pass
    return result

def run(args):
    """只管理本工具的 SSH 遥测会话；断线退避重连，不触及机载控制。"""
    stopped = threading.Event()
    child = [None]

    def stop(*_):
        stopped.set()
        proc = child[0]
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
            except ProcessLookupError:
                pass

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    source = Path(__file__).with_name('cloud_sender.py').read_bytes()
    remote = ('source ~/drone_platform/env.sh && '
              'exec /usr/bin/python3 -u - '+shlex.join(['--pc-ip',args.pc_ip,'--port',str(args.port)]+
                (['--pose-port',str(args.pose_port)] if args.with_pose else []))+
              ' # dianyun-pointcloud-only')
    ssh_options, ssh_env = background_ssh()
    cmd = ['ssh','-T',*ssh_options,'-o','ConnectTimeout=6',
           '-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2',args.user+'@'+args.host,remote]
    delay = 2.0
    try:
        while not stopped.is_set():
            print(json.dumps({'phase':'CONNECTING','host':args.host,'destination':[args.pc_ip,args.port],
                              'stamp':time.time()},ensure_ascii=False),flush=True)
            started = time.monotonic()
            try:
                proc = subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=subprocess.STDOUT,env=ssh_env)
                child[0] = proc
                if stopped.is_set() and proc.poll() is None:
                    proc.terminate()
                try:
                    proc.stdin.write(source)
                    proc.stdin.close()
                except (BrokenPipeError, OSError):
                    pass
                proc.wait()
                code = proc.returncode
            except OSError as exc:
                print('点云发送进程启动失败：{}'.format(exc),flush=True)
                code = -1
            finally:
                child[0] = None
            if stopped.is_set():
                break
            if time.monotonic() - started >= 30:
                delay = 2.0
            print(json.dumps({'phase':'RECONNECTING','host':args.host,'destination':[args.pc_ip,args.port],
                              'retry_s':delay,'exit_code':code,'stamp':time.time()},ensure_ascii=False),flush=True)
            stopped.wait(delay)
            delay = min(delay * 2.0, 15.0)
    finally:
        stop()
        if child[0] is not None:
            try:
                child[0].wait(timeout=3)
            except subprocess.TimeoutExpired:
                child[0].kill()
                child[0].wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start','stop','status','run'])
    parser.add_argument('--host', default='192.168.1.224')
    parser.add_argument('--user', default='orangepi')
    parser.add_argument('--pc-ip')
    parser.add_argument('--port', type=int, default=9000)
    parser.add_argument('--with-pose', action='store_true', help='同时发送同会话FAST-LIO位置/姿态，仅遥测')
    parser.add_argument('--pose-port', type=int, default=9001)
    args = parser.parse_args()
    if args.action == 'run':
        if not args.pc_ip:
            parser.error('run 需要 --pc-ip')
        run(args)
        return
    STATE.mkdir(exist_ok=True)
    with (STATE/'manager.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        existing = current()
        if args.action == 'status':
            print(json.dumps(report(existing),ensure_ascii=False,indent=2))
            return
        if args.action == 'stop':
            if existing:
                os.kill(existing['pid'],signal.SIGTERM)
                for _ in range(30):
                    if not current(): break
                    time.sleep(.1)
            print(json.dumps({'running':bool(current()),'stopped':bool(existing)},ensure_ascii=False))
            return
        if existing:
            if (existing['host'],existing['port']) != (args.host,args.port) or (args.pc_ip and existing['pc_ip'] != args.pc_ip):
                raise RuntimeError('已有不同配置的点云连接；先停止本工具会话再更换地址或端口')
            if args.with_pose and (not existing.get('with_pose') or existing.get('pose_port') != args.pose_port):
                # 只升级本工具自己的遥测进程，定位和其他ROS节点保持运行。
                os.kill(existing['pid'],signal.SIGTERM)
                for _ in range(30):
                    if not current(): break
                    time.sleep(.1)
                if current(): raise RuntimeError('旧遥测会话尚未结束，未启动重复发送器')
            else:
                print(json.dumps(report(existing),ensure_ascii=False,indent=2))
                return
        pc_ip = args.pc_ip
        if not pc_ip:
            route = json.loads(subprocess.check_output(['ip','-j','-4','route','get',args.host],text=True))
            pc_ip = route[0]['prefsrc']
        log = STATE/('sender-'+time.strftime('%Y%m%d-%H%M%S')+'.log')
        cmd = (['/usr/bin/python3',str(Path(__file__).resolve()),'run','--host',args.host,'--user',args.user,
                '--pc-ip',pc_ip,'--port',str(args.port)] +
               (['--with-pose','--pose-port',str(args.pose_port)] if args.with_pose else []))
        _, ssh_env = background_ssh()
        with log.open('w') as output:
            proc = subprocess.Popen(cmd,stdout=output,stderr=subprocess.STDOUT,start_new_session=True,env=ssh_env)
        tick,_ = identity(proc.pid)
        data = {'pid':proc.pid,'start_tick':tick,'host':args.host,'pc_ip':pc_ip,'port':args.port,'log':str(log),
                'with_pose':args.with_pose,'pose_port':args.pose_port if args.with_pose else None}
        (STATE/'active.json').write_text(json.dumps(data,indent=2))
        time.sleep(.5)
        if proc.poll() is not None:
            raise RuntimeError('发送桥启动失败：'+log.read_text()[-1500:])
        print(json.dumps(report(data),ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
