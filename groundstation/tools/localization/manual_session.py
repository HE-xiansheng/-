#!/usr/bin/env python3
"""原生ROS2前台会话；复用定位管理器，不改变话题或飞控参数。"""
import argparse
import contextlib
import fcntl
import json
import signal
import sys
import time
from pathlib import Path

from manage_stack import ExistingStack, locks as stack_locks


class StopRequested(Exception):
    pass


@contextlib.contextmanager
def lifecycle_locks():
    with stack_locks():yield


def session_token(api,kind):
    data=api.owned('relay' if kind=='relay' else 'localization')
    if not data:raise RuntimeError('未找到托管会话')
    return data['pid'],data['ticks'],Path(data['run'])


def start_session(api, kind, wait_source=None, started=None, cancelled=None):
    """终端再次执行即重启；先结束转发，避免跨定位原点继续转发。"""
    api.gate()
    lio, relay = api.inventory()
    api.require_idle()
    wait_source = wait_source or api.wait_source
    started = started or (lambda: None)

    def checkpoint():
        if cancelled and cancelled():
            raise StopRequested()

    checkpoint()
    if kind == 'localization':
        if relay:
            api.stop_relay()
        checkpoint()
        if lio:
            api.stop_lio()
        checkpoint()
        api.start_lio()
        started()
        checkpoint()
        if relay:
            wait_source()
            checkpoint()
            api.start_relay()
    else:
        if not lio:
            raise RuntimeError('请先在另一个终端启动定位，再启动转发')
        # 尚无有效定位时保留原转发；完成来源检查后才替换。
        wait_source()
        checkpoint()
        if relay:
            api.stop_relay()
        checkpoint()
        api.start_relay()
        started()


def stop_owned(api, kind, token):
    api.gate()
    lio, relay = api.inventory()
    api.require_idle()
    if not (relay if kind == 'relay' else lio):
        return
    if session_token(api, kind) != token:
        raise RuntimeError('当前会话已更换，本终端未停止新会话')
    if relay:
        api.stop_relay()
    if kind == 'localization' and lio:
        api.stop_lio()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=['localization', 'relay'])
    args, remaps = parser.parse_known_args()
    if remaps:
        parser.error('本入口使用固定ROS2接口，不接受ROS1 remap参数；请通过配置声明')
    root = Path.home()/'drone_platform/01_apps/fast_lio'
    ws = root.parent/'ego_planner'
    import rclpy
    rclpy.init(args=[])
    api = ExistingStack()
    requested = False
    waiting_source = False
    token = None

    def request_stop(signum, frame):
        nonlocal requested
        requested = True
        if waiting_source:
            raise StopRequested()

    def wait_source():
        nonlocal waiting_source
        waiting_source = True
        try:
            api.wait_source()
        finally:
            waiting_source = False

    def remember_session():
        nonlocal token
        token = session_token(api, args.kind)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, request_stop)
    try:
        with lifecycle_locks():
            start_session(api, args.kind, wait_source, remember_session, lambda: requested)
        logfile = token[2]/'console.log'
        with logfile.open(errors='replace') as log:
            while not requested:
                chunk = log.read(8192)
                if chunk:
                    print(chunk, end='', flush=True)
                try:
                    if session_token(api, args.kind) != token:
                        break
                    alive = bool(api.owned(args.kind))
                    if not alive:
                        break
                except (OSError, ValueError, KeyError, RuntimeError):
                    break
                time.sleep(.1)
    except StopRequested:
        pass
    finally:
        if requested and token is not None:
            try:
                with lifecycle_locks():
                    stop_owned(api, args.kind, token)
            except Exception as exc:
                print('为保护运行中的定位，未执行停止：'+str(exc)+
                      '。请在落地锁定并结束预览/任务后，通过地面站停止。', file=sys.stderr, flush=True)
        api.close()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('启动未完成：'+str(exc), file=sys.stderr, flush=True)
        raise SystemExit(1)
