"""版本化ROS2管理接口分发。只调用配置中已迁移的API，不覆盖机载管理器。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from gcs_ros2_runtime import configuration


def unavailable(role,reason):
    result={'ok':False,'available':False,'message':reason,'reason':reason,'status':'unavailable','role':role}
    if role=='mission':result['state']={}
    return result


def dispatch(role,arguments,input_data=b''):
    if role=='recording' and not configuration().get('modules',{}).get('recording'):
        import gcs_ros2_recording
        try:return gcs_ros2_recording.request(arguments[0])
        except Exception as exc:return unavailable(role,str(exc))
    config=configuration()
    spec=config.get('modules',{}).get(role,{})
    api=spec.get('api')
    if spec.get('api_version')!=1 or not isinstance(api,list) or not api or not all(isinstance(v,str) and v and '\x00' not in v for v in api):
        return unavailable(role,'机载ROS2'+role+'管理接口尚未部署或未配置（需要api_version=1）')
    # 身份由显式API契约确认，不能把存在同名目录/ROS节点当成管理器可用。
    try:
        check=subprocess.run([*api,'capabilities'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=8)
        contract=json.loads(check.stdout)
        if check.returncode or contract.get('backend')!='ros2' or contract.get('api_version')!=1 or role not in contract.get('roles',[]):
            return unavailable(role,'机载管理器ROS2版本/角色契约不匹配，未执行操作')
        if arguments[0] not in contract.get('actions',[]):
            return unavailable(role,'机载ROS2管理器不支持操作: '+arguments[0])
    except (OSError,ValueError,subprocess.TimeoutExpired) as exc:
        return unavailable(role,'无法核实机载ROS2管理器: '+str(exc))
    if arguments[0]=='watch':
        child=subprocess.Popen([*api,*arguments],stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=sys.stderr,start_new_session=True)
        def stop(*_):
            if child.poll() is None:
                try:os.killpg(child.pid,signal.SIGTERM)
                except ProcessLookupError:pass
        for sig in (signal.SIGHUP,signal.SIGTERM,signal.SIGINT):signal.signal(sig,stop)
        try:
            for line in child.stdout:
                # 不让管理器非JSON日志推进Qt任务状态。
                try:value=json.loads(line)
                except ValueError:continue
                if isinstance(value,dict):print(json.dumps(value,ensure_ascii=False,allow_nan=False),flush=True)
            if child.wait():return unavailable(role,'ROS2任务监视接口已退出')
        finally:
            stop()
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        return None
    try:
        result=subprocess.run([*api,*arguments],input=input_data,stdout=subprocess.PIPE,stderr=sys.stderr,timeout=110)
        rows=[]
        for line in result.stdout.splitlines():
            try:value=json.loads(line)
            except ValueError:continue
            if isinstance(value,dict):rows.append(value)
        if not rows:return unavailable(role,'机载ROS2管理器未返回有效JSON；实际结果未知')
        value=rows[-1]
        if result.returncode:value.update(ok=False,message=value.get('message','机载操作失败；实际结果未知'))
        return value
    except (OSError,subprocess.TimeoutExpired) as exc:
        return unavailable(role,'机载ROS2管理请求未完成；实际结果未知: '+str(exc))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('role',choices=['localization','planner','mission','recording'])
    parser.add_argument('arguments',nargs=argparse.REMAINDER)
    args=parser.parse_args()
    if not args.arguments:parser.error('缺少管理操作')
    data=sys.stdin.buffer.read(1024*1024+1) if args.arguments[0]=='start-map' else b''
    if len(data)>1024*1024:raise ValueError('地图任务请求过大')
    result=dispatch(args.role,args.arguments,data)
    if result is not None:print(json.dumps(result,ensure_ascii=False,allow_nan=False),flush=True)

if __name__=='__main__':main()
