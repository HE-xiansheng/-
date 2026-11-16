#!/usr/bin/env python3
"""只管理现有点云/话题桥接进程；地址已由上位机验证，不操作网络或飞行服务。"""
import argparse
from pathlib import Path
import subprocess


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--host',required=True)
    for name,default in [('cloud-port',9000),('pose-port',9001),('preflight-port',9003)]:
        parser.add_argument('--'+name,type=int,default=default)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    monitor=root/'tools/preflight/bridge.py'
    cloud=root/'tools/pointcloud-only/manage.py'
    def run(script,*values):
        result=subprocess.run(['/usr/bin/python3',str(script),*map(str,values)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=18)
        if result.returncode:raise RuntimeError('本地数据桥接操作失败：'+script.parent.name)
    run(cloud,'stop')
    run(monitor,'stop')
    run(monitor,'start','--host',args.host,'--port',args.preflight_port)
    run(cloud,'start','--host',args.host,'--port',args.cloud_port,'--with-pose','--pose-port',args.pose_port)
    print('只读数据桥接已启动',flush=True)

if __name__=='__main__':main()
