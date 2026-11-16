#!/usr/bin/env python3
"""执行器受控退出后，复用原管理入口停止预览；无解锁、切模式或参数写入。"""
import json
import math
import subprocess
from pathlib import Path


def attempt_stop(execution, flight, source_age, stop_original_preview):
    """必须先证明原执行器已退出、飞控状态新鲜且非OFFBOARD，再调用原停止入口。"""
    if execution.get('running') is not False:
        return {'status':'waiting','reason':'等待执行器受控退出；若处于OFFBOARD，请遥控切回POSCTL'}
    if (flight.get('connected') is not True or not flight.get('mode')
            or not isinstance(source_age,(int,float)) or not math.isfinite(source_age)
            or not -.1 <= source_age <= 1.5):
        return {'status':'waiting','reason':'飞控状态缺失或过期，未停止规划器'}
    if flight['mode']=='OFFBOARD':
        return {'status':'waiting','reason':'飞控仍在OFFBOARD；请遥控切回POSCTL后结束规划预览'}
    output=stop_original_preview()
    return {'status':'stopped','reason':'执行器已退出，原规划预览会话已停止','output':output}


def main():
    import fcntl
    import importlib.util
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from mavros_msgs.msg import State
    ws=Path.home()/'drone_platform/01_apps/ego_planner'
    # 使用ROS2管理器的实际运行目录和同一把锁，避免并发启动新执行器。
    spec=importlib.util.spec_from_file_location('existing_px4_manager',ws/'px4_execution/manage.py')
    manager=importlib.util.module_from_spec(spec);spec.loader.exec_module(manager)
    rclpy.init(args=[])
    node=rclpy.create_node('gcs_preview_stop_guard')
    messages=[]
    node.create_subscription(State,'/mavros/state',messages.append,qos_profile_sensor_data)
    import time
    end=time.monotonic()+3
    while not messages and time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.05)
    flight={}
    if messages:
        msg=messages[-1]
        flight={'connected':msg.connected,'mode':msg.mode}
    def source_age():
        if not messages:return math.inf
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        return node.get_clock().now().nanoseconds*1e-9-stamp if stamp>0 else math.inf
    def stop_original():
        result=subprocess.run(['bash',str(ws/'run_board.sh'),'--stop'],capture_output=True,text=True,timeout=18)
        if result.returncode:raise RuntimeError((result.stdout+result.stderr).strip()[-1500:])
        return result.stdout.strip()
    try:
        manager.RUNTIME.mkdir(parents=True,exist_ok=True)
        with (manager.RUNTIME/'management.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            execution=manager.read_status(manager.current())
            result=attempt_stop(execution,flight,source_age(),stop_original)
        print(json.dumps(result,ensure_ascii=False,allow_nan=False),flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__=='__main__':
    try:main()
    except Exception as exc:
        print(json.dumps({'status':'error','reason':str(exc)},ensure_ascii=False),flush=True)
        raise SystemExit(1)
