#!/usr/bin/env python3
"""Windows 前台遥测管理：仅复用既有点云、监视采样器，不启动机载飞行服务。"""
import argparse
import ctypes
from ctypes import wintypes
import importlib.util
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

TOOLS = Path(__file__).resolve().parents[1]


def own_process_tree():
    """将本进程及后代放入 Job；主进程退出或被终止时关闭整棵遥测进程树。"""
    class BasicLimits(ctypes.Structure):
        _fields_ = [('process_time', ctypes.c_int64), ('job_time', ctypes.c_int64),
                    ('flags', wintypes.DWORD), ('min_working_set', ctypes.c_size_t),
                    ('max_working_set', ctypes.c_size_t), ('active_processes', wintypes.DWORD),
                    ('affinity', ctypes.c_size_t), ('priority', wintypes.DWORD), ('scheduling', wintypes.DWORD)]
    class IoCounters(ctypes.Structure):
        _fields_ = [(name, ctypes.c_uint64) for name in
                    ['read_ops', 'write_ops', 'other_ops', 'read_bytes', 'write_bytes', 'other_bytes']]
    class ExtendedLimits(ctypes.Structure):
        _fields_ = [('basic', BasicLimits), ('io', IoCounters),
                    ('process_memory', ctypes.c_size_t), ('job_memory', ctypes.c_size_t),
                    ('peak_process_memory', ctypes.c_size_t), ('peak_job_memory', ctypes.c_size_t)]
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.CreateJobObjectW.argtypes = [ctypes.c_void_p, wintypes.LPCWSTR]
    kernel.CreateJobObjectW.restype = wintypes.HANDLE
    kernel.SetInformationJobObject.argtypes = [wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD]
    kernel.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
    kernel.GetCurrentProcess.restype = wintypes.HANDLE
    job = kernel.CreateJobObjectW(None, None)
    limits = ExtendedLimits(); limits.basic.flags = 0x2000  # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
    if not job or not kernel.SetInformationJobObject(job, 9, ctypes.byref(limits), ctypes.sizeof(limits)):
        raise ctypes.WinError(ctypes.get_last_error())
    if not kernel.AssignProcessToJobObject(job, kernel.GetCurrentProcess()):
        raise ctypes.WinError(ctypes.get_last_error())
    return job  # 句柄保持到进程退出，不允许子进程继承。


def worker(args):
    directory = TOOLS / ('preflight' if args.worker == 'monitor' else 'pointcloud-only')
    sys.path.insert(0, str(directory))
    script = directory / ('bridge.py' if args.worker == 'monitor' else 'manage.py')
    spec = importlib.util.spec_from_file_location('gcs_windows_worker', script)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    args.port = args.preflight_port if args.worker == 'monitor' else args.cloud_port
    args.with_pose = True
    module.run(args)


def main():
    # 不依赖 Windows 系统代码页；状态消息和 Qt 接收日志统一使用 UTF-8。
    os.environ['PYTHONUTF8'] = '1'
    os.environ['PYTHONIOENCODING'] = 'utf-8'
    for stream in (sys.stdout, sys.stderr):
        if stream is not None: stream.reconfigure(encoding='utf-8', errors='replace')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--user', default='orangepi')
    parser.add_argument('--worker', choices=['cloud', 'monitor'])
    parser.add_argument('--pc-ip')
    parser.add_argument('--raw-udp-port', type=int, default=9005)
    for name, default in [('cloud-port', 9000), ('pose-port', 9001), ('preflight-port', 9003)]:
        parser.add_argument('--' + name, type=int, default=default)
    args = parser.parse_args()
    if not all(1 <= value <= 65535 for value in (args.cloud_port, args.pose_port, args.preflight_port, args.raw_udp_port)):
        parser.error('端口必须为1..65535')
    if args.worker:
        worker(args); return
    if os.name != 'nt':
        raise RuntimeError('此入口仅用于 Windows；Linux 沿用 manage_link.py')
    job = own_process_tree()
    route = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        route.connect((args.host, 22)); args.pc_ip = route.getsockname()[0]
    finally:
        route.close()
    common = ['--host', args.host, '--user', args.user, '--pc-ip', args.pc_ip,
              '--cloud-port', str(args.cloud_port), '--pose-port', str(args.pose_port),
              '--preflight-port', str(args.preflight_port)]
    common += ['--raw-udp-port', str(args.raw_udp_port)]
    children = []
    try:
        for kind in ['monitor', 'cloud']:
            child = subprocess.Popen([sys.executable, '-u', str(Path(__file__).resolve()), '--worker', kind] + common,
                                     stdout=subprocess.DEVNULL, stderr=None,
                                     creationflags=subprocess.CREATE_NO_WINDOW)
            children.append(child)
        time.sleep(.6)
        if any(child.poll() is not None for child in children):
            raise RuntimeError('Windows 遥测工作进程启动失败')
        print('GCS_BRIDGE_READY', flush=True)
        while all(child.poll() is None for child in children):
            time.sleep(.5)
        raise RuntimeError('Windows 遥测工作进程已退出')
    finally:
        for child in children:
            if child.poll() is None: child.kill()
        for child in children: child.wait(timeout=5)


if __name__ == '__main__':
    main()
