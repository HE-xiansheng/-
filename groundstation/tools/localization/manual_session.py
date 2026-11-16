#!/usr/bin/env python3
"""旧 roslaunch 命令的前台会话；复用地面站管理器，不改变话题或飞控参数。"""
import argparse
import contextlib
import fcntl
import json
import signal
import sys
import time
from pathlib import Path

from manage_stack import ExistingStack


class StopRequested(Exception):
    pass


@contextlib.contextmanager
def lifecycle_locks(root, ws):
    with contextlib.ExitStack() as stack:
        for path in (root/'.manager.lock', root/'.pose-relay-lifecycle.lock',
                     ws/'.px4-execution-management.lock', ws/'.manual-preview.lock'):
            handle = stack.enter_context(path.open('a'))
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError('已有定位、预览或执行器管理操作，请稍后重试')
        yield


def session_token(api, kind):
    """记录 PID 及内核启动时间，避免退出时误停后来启动的新会话。"""
    data = (api.relay() if kind == 'relay' else
            json.loads(api.board.active_file('localization').read_text()))
    if not data:
        raise RuntimeError('未找到托管会话')
    pid = int(data['pid'])
    tick = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')', 1)[1].split()[19]
    return pid, tick, Path(data['run'])


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
    if any(':=' not in arg for arg in remaps):
        parser.error('只接受 ROS 名称映射参数')
    root = Path.home()/'drone_platform/01_apps/fast_lio'
    ws = root.parent/'ego_planner'
    api = ExistingStack(root, ws)
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
        with lifecycle_locks(root, ws):
            start_session(api, args.kind, wait_source, remember_session, lambda: requested)
        logfile = token[2]/('process.log' if args.kind == 'localization' else 'relay.log')
        with logfile.open(errors='replace') as log:
            while not requested:
                chunk = log.read(8192)
                if chunk:
                    print(chunk, end='', flush=True)
                try:
                    if session_token(api, args.kind) != token:
                        break
                    alive = bool(api.relay()) if args.kind == 'relay' else api.board.alive('localization')
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
                with lifecycle_locks(root, ws):
                    stop_owned(api, args.kind, token)
            except Exception as exc:
                print('为保护运行中的定位，未执行停止：'+str(exc)+
                      '。请在落地锁定并结束预览/任务后，通过地面站停止。', file=sys.stderr, flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('启动未完成：'+str(exc), file=sys.stderr, flush=True)
        raise SystemExit(1)
