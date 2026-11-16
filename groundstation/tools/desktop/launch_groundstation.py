#!/usr/bin/env python3
"""桌面入口：沿用连接端口，重复点击唤起现有窗口，不启动机载服务。"""
import ctypes as C
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[3]
BINARY=ROOT/'build/linux/customgeometry'
STATE=ROOT/'.pointcloud-link'

def log(text):
    STATE.mkdir(exist_ok=True)
    with (STATE/'desktop-launch.log').open('a') as out:out.write(time.strftime('%F %T ')+text+'\n')

def existing_pids():
    matches=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            exe=os.readlink(proc/'exe').removesuffix(' (deleted)') if sys.version_info>=(3,9) else os.readlink(proc/'exe').replace(' (deleted)','')
            if Path(exe)==BINARY:matches.append(int(proc.name))
        except (OSError,ValueError):pass
    return matches

def raise_existing(pids):
    """仅激活属于当前程序的本机X11窗口；Wayland不支持时保留已有实例。"""
    try:
        x=C.CDLL('libX11.so.6')
        x.XOpenDisplay.argtypes=[C.c_char_p];x.XOpenDisplay.restype=C.c_void_p
        display=x.XOpenDisplay(os.environ.get('DISPLAY',':1').encode())
        if not display:return False
        x.XDefaultRootWindow.argtypes=[C.c_void_p];x.XDefaultRootWindow.restype=C.c_ulong
        x.XInternAtom.argtypes=[C.c_void_p,C.c_char_p,C.c_int];x.XInternAtom.restype=C.c_ulong
        x.XGetWindowProperty.argtypes=[C.c_void_p,C.c_ulong,C.c_ulong,C.c_long,C.c_long,C.c_int,C.c_ulong,C.POINTER(C.c_ulong),C.POINTER(C.c_int),C.POINTER(C.c_ulong),C.POINTER(C.c_ulong),C.POINTER(C.POINTER(C.c_ubyte))]
        x.XFree.argtypes=[C.c_void_p];x.XCloseDisplay.argtypes=[C.c_void_p]
        root=x.XDefaultRootWindow(display)
        atom=lambda name:x.XInternAtom(display,name.encode(),False)
        def prop(window,name):
            typ=C.c_ulong();fmt=C.c_int();count=C.c_ulong();left=C.c_ulong();data=C.POINTER(C.c_ubyte)()
            if x.XGetWindowProperty(display,window,atom(name),0,4096,False,0,C.byref(typ),C.byref(fmt),C.byref(count),C.byref(left),C.byref(data)) or not data:return []
            try:
                if fmt.value==32:return list(C.cast(data,C.POINTER(C.c_ulong))[:count.value])
                if fmt.value==8:return C.string_at(data,count.value).decode('utf8',errors='replace')
                return []
            finally:x.XFree(data)
        windows=[]
        for window in prop(root,'_NET_CLIENT_LIST'):
            pid=prop(window,'_NET_WM_PID')
            if pid and pid[0] in pids:windows.append((window,prop(window,'_NET_WM_NAME')))
        if not windows:x.XCloseDisplay(display);return False
        window=next((w for w,title in windows if title=='科研飞行平台-地面站'),windows[0][0])
        class Data(C.Union):_fields_=[('l',C.c_long*5),('b',C.c_char*20)]
        class ClientMessage(C.Structure):_fields_=[('type',C.c_int),('serial',C.c_ulong),('send_event',C.c_int),('display',C.c_void_p),('window',C.c_ulong),('message_type',C.c_ulong),('format',C.c_int),('data',Data)]
        class Event(C.Union):_fields_=[('client',ClientMessage),('pad',C.c_long*24)]
        event=Event();event.client.type=33;event.client.send_event=1;event.client.display=display;event.client.window=window;event.client.message_type=atom('_NET_ACTIVE_WINDOW');event.client.format=32;event.client.data.l[0]=1
        x.XMapRaised.argtypes=[C.c_void_p,C.c_ulong];x.XSendEvent.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_long,C.POINTER(Event)];x.XFlush.argtypes=[C.c_void_p]
        x.XMapRaised(display,window);x.XSendEvent(display,root,False,(1<<19)|(1<<20),C.byref(event));x.XFlush(display);x.XCloseDisplay(display)
        return True
    except (OSError,AttributeError,TypeError):return False

def port(value,fallback):
    try:n=int(value);return n if 1<=n<=65535 else fallback
    except (ValueError,TypeError):return fallback

def command():
    try:config=json.loads((STATE/'connection.json').read_text())
    except (OSError,ValueError):config={}
    return [str(ROOT/'start-groundstation.sh'),'--live',str(port(config.get('cloud_port'),9000)),
            '--pose-port',str(port(config.get('pose_port'),9001)),
            '--preflight-port',str(port(config.get('preflight_port'),9003))]

def main():
    if '--check' in sys.argv:
        print(json.dumps({'binary_exists':BINARY.is_file(),'command':command(),'existing_pids':existing_pids()},ensure_ascii=False));return
    STATE.mkdir(exist_ok=True)
    lock=(STATE/'desktop-launch.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        pids=existing_pids();raised=raise_existing(pids);log('existing launch lock; pids=%s raised=%s'%(pids,raised));return
    pids=existing_pids()
    if pids:
        raised=raise_existing(pids);log('activate existing pids=%s raised=%s'%(pids,raised))
        if not raised:
            try:subprocess.run(['notify-send','科研飞行平台-地面站','程序已在运行，请切换到现有窗口。'],timeout=3,check=False)
            except (OSError,subprocess.TimeoutExpired):pass
        return
    if not BINARY.is_file():
        log('binary missing')
        try:subprocess.run(['notify-send','地面站尚未构建','请先运行 build-groundstation.sh'],timeout=3,check=False)
        except (OSError,subprocess.TimeoutExpired):pass
        return
    os.set_inheritable(lock.fileno(),True)
    args=command();log('start '+json.dumps(args,ensure_ascii=False))
    os.execv(args[0],args)
if __name__=='__main__':main()
