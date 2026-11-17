#!/usr/bin/env python3
"""复用已有飞前采样/评估，SSH只读快照→本机UDP9003；不安装机载文件或调用飞控服务。"""
import argparse
import base64
import sys
try:
    import fcntl
except ImportError:  # Windows 只运行前台遥测；生命周期由 windows_bridge 管理。
    fcntl = None
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import threading
import time
import uuid
import queue
from preflight_status import evaluate,sensor_snapshot
from planner_display import snapshot
from stream_reader import packets_from_pipe,decode_grid,StatusAssembler
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'pointcloud-only'))
from cloud_sender import Lz4,packets
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'communication'))
from ssh_session import background_ssh
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2'))
from gcs_profile import load_profile, command, payload

ROOT=Path(__file__).resolve().parents[3]
STATE=ROOT/'.pointcloud-link'


def format_packet(packet,host):
    delay=packet.get('_transport_delay_s',0.)
    rows,issues=evaluate(packet,expected_frames={'cloud':'camera_init'},expected_publishers=load_profile().get('expected_publishers',{}))
    result=[]
    for key,label,status,hz,age,publishers,detail,topic in rows:
        if key=='text':continue
        raw=packet.get('topics',{}).get(key,{})
        result.append({'key':key,'label':label,'state':status,'ok':status=='正常','hz':hz,'age':age,
                       'detail':detail or raw.get('frame','') or '—','topic':topic,'publishers':publishers})
    return {'type':'PFS1','schema':1,'connected':True,'source_host':host,'sample_period_s':packet.get('sample_period_s',1.),
            'sensors':sensor_snapshot(packet),'rows':result,'issues':issues,'stamp':time.time(),'planning':snapshot(packet.get('ego_planner',{})),'recording':packet.get('recording',{}),'localization':packet.get('localization',{}),'transport_delay_s':delay,'sample_cost_s':packet.get('sample_cost_s'),'transport':packet.get('_transport','ssh')}


def identity(pid):
    try:
        p=Path('/proc')/str(pid)
        return p.joinpath('stat').read_text().rsplit(')',1)[1].split()[19],p.joinpath('cmdline').read_bytes().split(b'\0')
    except (OSError,ValueError,IndexError):return None,[]


def current():
    try:
        data=json.loads((STATE/'preflight-active.json').read_text());tick,args=identity(data['pid'])
        if tick==data['start_tick'] and os.fsencode(str(Path(__file__).resolve())) in args and b'run' in args:return data
    except (OSError,ValueError,KeyError):pass
    return None


def merge_grid(planning,cache,now):
    """仅将相同原生规划会话的栅格合并，累计排队时间用于真实过期判断。"""
    session=planning.get('status',{}).get('data',{}).get('session_id')
    if session and cache.get('session_id')==session:
        age=max(0.,now-cache['received']);row=cache['row']
        planning['grid']=dict(row,receipt_age_s=row['receipt_age_s']+age)
    return planning


def local_udp_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if sys.platform == 'linux' and 'microsoft' in os.uname().release.lower():
        # Mirrored WSL can route localhost via the Windows-facing loopback0.
        # Keep these in-WSL datagrams on Linux lo, including multi-KB snapshots.
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, b'lo\0')
        except OSError as exc:
            print(f'本机 UDP 无法绑定 lo：{exc}', file=sys.stderr, flush=True)
    return sock


def run(args):
    stream_id=uuid.uuid4().hex
    grid_stream_id=uuid.uuid4().hex
    stopped=threading.Event();children=[None,None];latest_grid={};grid_lock=threading.Lock();last_status=[0.];udp_offset=[None];udp_boot=[None]
    last_grid_packet=[0.];grid_queue=queue.Queue(maxsize=1)
    def stop(*_):
        stopped.set()
        for child in children:
            try:
                if child and child.poll() is None:child.terminate()
            except ProcessLookupError:pass
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    udp=local_udp_socket();udp.connect(('127.0.0.1',args.port))
    output_lock = threading.Lock()
    last_send_error = float('-inf')
    ssh_options, ssh_env = background_ssh()
    profile=load_profile()
    source=payload(Path(__file__).with_name('telemetry_reader.py').read_bytes())
    def connect_reader(index,grid=False):
        remote=command('exec /usr/bin/python3 -u - --cloud-topic '+shlex.quote(profile['topics']['cloud'])+
                ' --no-rosout --planner'+(' --grid-only' if grid else '')+
                ' --udp-host "${SSH_CONNECTION%% *}" --udp-port '+str(getattr(args,'raw_udp_port',9005))+
                ' --stream-id '+(grid_stream_id if grid else stream_id),profile)
        child=subprocess.Popen(['ssh','-T',*ssh_options,'-o','ConnectTimeout=6',
            '-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2',args.user+'@'+args.host,remote],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=None,env=ssh_env)
        children[index]=child;child.stdin.write(source);child.stdin.close();return child
    def close_reader(index):
        child=children[index]
        if not child:return
        try:
            if child.poll() is None:child.terminate()
        except ProcessLookupError:pass
        try:child.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:child.kill()
            except ProcessLookupError:pass
            child.wait()
        children[index]=None
    def output(value):
        nonlocal last_send_error
        raw=json.dumps(value,ensure_ascii=False,allow_nan=False).encode()
        if len(raw)>60000:raise ValueError('监视快照超过本机UDP上限')
        with output_lock:
            try:
                udp.send(raw)
            except OSError as exc:
                now = time.monotonic()
                if now - last_send_error >= 5:
                    print(f'状态 UDP 发送至 127.0.0.1:{args.port} 失败：{exc}', file=sys.stderr, flush=True)
                    last_send_error = now
            sys.stdout.write(raw.decode() + '\n')
            sys.stdout.flush()
    def queue_grid(packet):
        try:grid_queue.put_nowait(packet)
        except queue.Full:
            try:grid_queue.get_nowait()
            except queue.Empty:pass
            try:grid_queue.put_nowait(packet)
            except queue.Full:pass

    def read_grid():
        # SSH仅负责启动/管理只读订阅器；显示数据不经过SSH的大流量队列。
        while not stopped.is_set():
            try:
                child=connect_reader(1,True)
                for packet in packets_from_pipe(child,stopped,alive_hint=lambda:time.monotonic()-last_grid_packet[0]<3):
                    if packet.get('type')=='GRID1':
                        packet['_local_received']=time.monotonic();last_grid_packet[0]=packet['_local_received'];queue_grid(packet)
            except (OSError,ValueError,KeyError,TimeoutError) as exc:
                if not stopped.is_set():print('栅格显示通道：'+str(exc),file=sys.stderr,flush=True)
            finally:close_reader(1)
            stopped.wait(2.)

    def render_grid():
        sock=local_udp_socket();codec=Lz4();sequence=0;last=None;last_error=0.
        try:
            while not stopped.is_set():
                try:
                    try:packet=grid_queue.get(timeout=.25)
                    except queue.Empty:continue
                    planning=packet.get('ego_planner',{});status=planning.get('status',{}).get('data',{})
                    row=planning.get('grid',{});data=row.get('data',{});key=(status.get('session_id'),data.get('sequence'))
                    source_age=row.get('receipt_age_s',99)+packet.get('_transport_delay_s',0.)+max(0.,time.monotonic()-packet.get('_local_received',time.monotonic()))
                    if not key[0] or key==last or not 0<=source_age<=1.5:continue
                    raw=decode_grid(data)
                    sequence=(sequence+1)&0xffffffff
                    for datagram in packets(raw,codec.compress(raw),sequence) if raw else []:sock.sendto(datagram,('127.0.0.1',9004))
                    with grid_lock:
                        latest_grid.clear();latest_grid.update(session_id=key[0],received=time.monotonic(),row={'data':{k:v for k,v in data.items() if k!='xyz'},'receipt_age_s':source_age})
                    last=key
                except (OSError,ValueError,KeyError,TypeError) as exc:
                    if time.monotonic()-last_error>=5:
                        print('栅格显示帧无效：'+str(exc),file=sys.stderr,flush=True);last_error=time.monotonic()
        finally:sock.close()
    def deliver(packet):
        if not isinstance(packet.get('topics'),dict):return
        planning=packet.get('ego_planner',{})
        with grid_lock:
            merge_grid(planning,latest_grid,time.monotonic())
        if packet.get('_transport_delay_s',0.)>2.5:return
        output(format_packet(packet,args.host));last_status[0]=time.monotonic()
    def read_status_udp():
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1024*1024)
        sock.bind(('0.0.0.0',getattr(args,'raw_udp_port',9005)));sock.settimeout(.25);expected=socket.gethostbyname(args.host)
        assemblers={b'TLM1':StatusAssembler(),b'TLG1':StatusAssembler(b'TLG1')}
        previous={b'TLM1':{'boot':None,'offset':None,'remote':-1.},b'TLG1':{'boot':None,'offset':None,'remote':-1.}}
        try:
            while not stopped.is_set():
                try:raw,address=sock.recvfrom(65535)
                except socket.timeout:continue
                if address[0]!=expected:continue
                kind=raw[:4];assembler=assemblers.get(kind)
                if assembler is None:continue
                packet=assembler.accept(raw)
                expected_stream=grid_stream_id if kind==b'TLG1' else stream_id
                if packet is None or packet.get('stream_id')!=expected_stream:continue
                remote=packet.get('server_monotonic');boot=packet.get('boot_id')
                if not isinstance(remote,(int,float)):continue
                clock=previous[kind]
                if clock['boot']!=boot:clock.update(boot=boot,offset=None,remote=-1.)
                if remote<=clock['remote']:continue
                clock['remote']=remote;received=time.monotonic();delta=received-remote
                clock['offset']=delta if clock['offset'] is None else min(clock['offset'],delta)
                packet['_transport_delay_s']=max(0.,delta-clock['offset']);packet['_transport']='udp'
                if kind==b'TLG1':
                    if packet.get('type')=='GRID1':
                        packet['_local_received']=received;last_grid_packet[0]=received;queue_grid(packet)
                    continue
                try:deliver(packet)
                except (OSError,ValueError,KeyError,TypeError) as exc:print('状态包已丢弃：'+str(exc),file=sys.stderr,flush=True)
        finally:sock.close()
    udp_worker=threading.Thread(target=read_status_udp,daemon=True);udp_worker.start()
    worker=threading.Thread(target=read_grid,daemon=True);worker.start()
    grid_renderer=threading.Thread(target=render_grid,daemon=True);grid_renderer.start()
    try:
        while not stopped.is_set():
            if time.monotonic()-last_status[0]>2.5:output({'type':'PFS1','schema':1,'connected':False,'source_host':args.host,'error':'正在连接机载只读监视'})
            try:
                child=connect_reader(0)
                for packet in packets_from_pipe(child,stopped,alive_hint=lambda:time.monotonic()-last_status[0]<3):
                    deliver(packet)  # 兼容旧采样器；新版状态独立走单向UDP。
            except (OSError,ValueError,TimeoutError) as exc:
                if not stopped.is_set() and time.monotonic()-last_status[0]>2.5:output({'type':'PFS1','schema':1,'connected':False,'source_host':args.host,'error':str(exc)[:300]})
            finally:close_reader(0)
            stopped.wait(2.)
    finally:
        stop();close_reader(0);worker.join(timeout=5);grid_renderer.join(timeout=2);udp_worker.join(timeout=2);close_reader(1);udp.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['start','stop','status','run'])
    parser.add_argument('--host',default=load_profile()['host']);parser.add_argument('--user',default=load_profile()['user'])
    parser.add_argument('--port',type=int,default=9003);args=parser.parse_args()
    if not 1<=args.port<=65535:parser.error('端口必须为1..65535')
    if args.action=='run':run(args);return
    STATE.mkdir(exist_ok=True)
    with (STATE/'preflight.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX);existing=current()
        if args.action=='status':print(json.dumps(dict(existing or {},running=bool(existing)),ensure_ascii=False,indent=2));return
        if args.action=='stop':
            if existing:
                os.kill(existing['pid'],signal.SIGTERM)
                for _ in range(60):
                    if not current():break
                    time.sleep(.1)
            print(json.dumps({'running':bool(current()),'stopped':bool(existing)}));return
        if existing:
            if existing['host']!=args.host or existing['port']!=args.port:raise RuntimeError('已有其他监视配置；先停止监视再更换')
            print(json.dumps(dict(existing,running=True),ensure_ascii=False,indent=2));return
        log=STATE/('preflight-'+time.strftime('%Y%m%d-%H%M%S')+'.log')
        _, ssh_env = background_ssh()
        with log.open('w') as f:
            proc=subprocess.Popen(['/usr/bin/python3',str(Path(__file__).resolve()),'run','--host',args.host,'--user',args.user,'--port',str(args.port)],stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=ssh_env)
        tick,_=identity(proc.pid);data={'pid':proc.pid,'start_tick':tick,'host':args.host,'port':args.port,'log':str(log)}
        (STATE/'preflight-active.json').write_text(json.dumps(data,indent=2))
        time.sleep(.3)
        if proc.poll() is not None:raise RuntimeError(log.read_text()[-1500:])
        print(json.dumps(dict(data,running=True),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
