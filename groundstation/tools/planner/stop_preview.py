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
    import rospy
    from mavros_msgs.msg import State
    ws=Path.home()/'drone_platform/01_apps/ego_planner'
    # 使用原管理器的PID身份校验和锁，防止停止规划器时并发启动新执行器。
    spec=importlib.util.spec_from_file_location('existing_px4_manager',ws/'px4_execution/manage.py')
    manager=importlib.util.module_from_spec(spec);spec.loader.exec_module(manager)
    rospy.init_node('gcs_preview_stop_guard',anonymous=True,disable_signals=True,disable_rosout=True)
    try:
        msg=rospy.wait_for_message('/mavros/state',State,timeout=3)
        flight={'connected':msg.connected,'mode':msg.mode}
        age=rospy.Time.now().to_sec()-msg.header.stamp.to_sec()
    except rospy.ROSException:
        flight={};age=math.inf
    def stop_original():
        result=subprocess.run(['bash',str(ws/'run_board.sh'),'--stop'],capture_output=True,text=True,timeout=18)
        if result.returncode:raise RuntimeError((result.stdout+result.stderr).strip()[-1500:])
        return result.stdout.strip()
    with (ws/'.px4-execution-management.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        # 锁可能被其他管理请求占用；在实际决策时重算源龄期。
        if flight:age=rospy.Time.now().to_sec()-msg.header.stamp.to_sec()
        execution=manager.read_status(manager.current())
        result=attempt_stop(execution,flight,age,stop_original)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False),flush=True)


if __name__=='__main__':
    try:main()
    except Exception as exc:
        print(json.dumps({'status':'error','reason':str(exc)},ensure_ascii=False),flush=True)
        raise SystemExit(1)
