#!/usr/bin/env python3
"""Own this ROS2 planner launch only; preserve the existing --start/--stop/--status CLI."""
import argparse
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
LAUNCH = ROOT / 'launch/manual_preview.launch.py'
RUNTIME = Path(os.environ.get('EGO_PREVIEW_RUNTIME_DIR', '/tmp/ego-manual-preview-%d' % os.getuid()))
STATE = RUNTIME / 'active.json'


def identity(pid):
    try:
        values = Path('/proc/%d/stat' % pid).read_text().split(') ', 1)[1].split()
        cmd = Path('/proc/%d/cmdline' % pid).read_bytes().replace(b'\0', b' ').decode()
        return (values[19], cmd) if values[0] != 'Z' else (None, '')
    except (OSError, IndexError):
        return None, ''


def owned():
    try:
        data = json.loads(STATE.read_text())
        tick, cmd = identity(data['pid'])
        if tick == data['start_tick'] and str(LAUNCH) in cmd and ' launch ' in cmd:
            return data
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return None


def check_execution_idle():
    spec = importlib.util.spec_from_file_location('px4_execution_manager', ROOT / 'px4_execution/manage.py')
    manager = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(manager)
    info = manager.current()
    if info:
        state = manager.read_status(info)
        if state.get('output_enabled') or state.get('mode') == 'OFFBOARD' or state.get('phase') == 'STATUS_STALE':
            raise RuntimeError('执行适配器仍在输出或状态未知；先请求执行停止并由遥控器接管，再停止规划器。')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', nargs='?', default='--start', choices=['--start', '--stop', '--status'])
    # Old callers supply --start as a flag, so keep that spelling.
    choices = parser.add_mutually_exclusive_group()
    choices.add_argument('--start', dest='command', action='store_const', const='--start')
    choices.add_argument('--stop', dest='command', action='store_const', const='--stop')
    choices.add_argument('--status', dest='command', action='store_const', const='--status')
    args = parser.parse_args()
    action = args.command or args.action
    RUNTIME.mkdir(parents=True, exist_ok=True)
    with (RUNTIME / 'management.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        info = owned()
        if action == '--status':
            print(json.dumps(info or {'running': False}, ensure_ascii=False))
            return
        if action == '--stop':
            if info:
                check_execution_idle()
                os.killpg(info['pid'], signal.SIGINT)
                deadline = time.monotonic() + 10
                while owned() and time.monotonic() < deadline:
                    time.sleep(.1)
                if owned():
                    raise RuntimeError('规划器尚未正常退出；未强制终止，请检查运行状态。')
                STATE.unlink(missing_ok=True)
            print(json.dumps({'running': False}, ensure_ascii=False))
            return
        if info:
            print(json.dumps(info, ensure_ascii=False))
            return
        for entry in Path('/proc').iterdir():
            if entry.name.isdigit():
                _, cmd = identity(int(entry.name))
                if '/lib/plan_manage/planner_node ' in cmd:
                    raise RuntimeError('存在其他原生规划器；请先检查其所属会话。')
        import rclpy
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
        from mavros_msgs.msg import State
        from std_msgs.msg import String
        rclpy.init()
        node = rclpy.create_node('manual_preview_start_check')
        flight, ready = [], []
        node.create_subscription(State, '/mavros/state', flight.append, qos_profile_sensor_data)
        node.create_subscription(String, '/ego_planner/manual_preview/status', ready.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        proc = None
        try:
            deadline = time.monotonic() + 5
            while not flight and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            if not flight:
                raise RuntimeError('未收到MAVROS状态；尚未启动规划器。')
            fc = flight[-1]
            stamp = fc.header.stamp.sec + fc.header.stamp.nanosec*1e-9
            age = node.get_clock().now().nanoseconds*1e-9 - stamp
            if not fc.connected or fc.armed or stamp <= 0 or not -.05 <= age <= 2:
                raise RuntimeError('需新鲜、已连接且未解锁的MAVROS状态才能启动规划预览。')
            run = RUNTIME / ('session-%d' % time.time_ns())
            run.mkdir()
            env = dict(os.environ, ROS_LOG_DIR=str(run / 'ros-logs'))
            with (run / 'console.log').open('ab') as log:
                proc = subprocess.Popen(['ros2', 'launch', str(LAUNCH)], stdin=subprocess.DEVNULL,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)
            tick, _ = identity(proc.pid)
            data = {'pid': proc.pid, 'start_tick': tick, 'run': str(run), 'running': True, 'ros_version': 2}
            temporary = STATE.with_suffix('.next')
            temporary.write_text(json.dumps(data))
            os.replace(temporary, STATE)
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and proc.poll() is None:
                rclpy.spin_once(node, timeout_sec=.05)
                for msg in ready:
                    try:
                        value = json.loads(msg.data)
                        if value.get('session_id') and value.get('goal_frame'):
                            data['session_id'] = value['session_id']
                            STATE.write_text(json.dumps(data))
                            print(json.dumps(data, ensure_ascii=False))
                            return
                    except (ValueError, TypeError):
                        pass
                ready.clear()
            raise RuntimeError('规划器启动尚未确认；请查看--status及临时目录 '+str(run))
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
