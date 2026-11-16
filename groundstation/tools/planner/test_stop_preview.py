#!/usr/bin/python3
"""验证停止顺序与OFFBOARD等待，不连接ROS或启动任何真实进程。"""
from stop_preview import attempt_stop
calls=[]
def stopped():calls.append('original_preview_stop');return 'TEST ONLY'
manual={'connected':True,'mode':'POSCTL'}
for execution,flight,age in [
    ({'running':True},manual,0),
    ({},manual,0),
    ({'running':False},{'connected':True,'mode':'OFFBOARD'},0),
    ({'running':False},{'connected':False,'mode':'POSCTL'},0),
    ({'running':False},manual,4),
    ({'running':False},manual,-1),
    ({'running':False},manual,float('nan')),
    ({'running':False},{},0),
]:
    assert attempt_stop(execution,flight,age,stopped)['status']=='waiting'
assert calls==[],'unsafe/unknown state must never stop original planner'
result=attempt_stop({'running':False},manual,.2,stopped)
assert result['status']=='stopped' and calls==['original_preview_stop']
# 飞行模式由遥控切回POSCTL以后，再次检查才停止；不自动切模式。
calls.clear()
assert attempt_stop({'running':True},{'connected':True,'mode':'OFFBOARD'},0,stopped)['status']=='waiting'
assert attempt_stop({'running':False},{'connected':True,'mode':'OFFBOARD'},0,stopped)['status']=='waiting'
assert calls==[]
assert attempt_stop({'running':False},manual,0,stopped)['status']=='stopped'
assert len(calls)==1
try:
    attempt_stop({'running':False},manual,0,lambda:(_ for _ in ()).throw(RuntimeError('failed stop')))
except RuntimeError:pass
else:raise AssertionError('stop errors must not become successful completion')
print('PASS: running/unknown executor, OFFBOARD, source freshness, RC handover and failed shutdown; no real ROS/control')
