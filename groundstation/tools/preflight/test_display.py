#!/usr/bin/env python3
"""无ROS/飞控连接的显示有效性回归。"""
import copy,json
from planner_display import snapshot
from bridge import format_packet

now=100.
status={'session_id':'s','mode':'manual_preview','goal_frame':'world','height_reference':'fastlio_session_origin','phase':'ACTIVE','trajectory_id':7,'stamp':now,'planning_mode':'measured_preview'}
trajectory={'id':7,'start_time':now-.1,'order':5,'duration':[1.],'x':[0,0,0,0,1,0],'y':[0]*6,'z':[0,0,0,0,0,1]}
packet={'now':now,'status':{'data':status,'receipt_age_s':.1},'trajectory':{'data':trajectory,'receipt_age_s':.1},
        'grid':{'data':{'frame':'world','stamp':now,'session_id':'s','count':10,'source_count':20,'resolution':.1,'sequence':1},'receipt_age_s':.1}}
s=snapshot(packet);assert s['path_valid'] and s['grid_valid'] and s['path'][0]==[0,0,1] and s['path'][-1]==[1,0,1]
p=copy.deepcopy(packet);p['trajectory']['data']['id']=8;assert not snapshot(p)['path_valid']
p=copy.deepcopy(packet);p['status']['data']['phase']='GOAL_IN_COLLISION';assert not snapshot(p)['path_valid']
p=copy.deepcopy(packet);p['status']['receipt_age_s']=1;assert not snapshot(p)['path_valid'] and not snapshot(p)['grid_valid']
p=copy.deepcopy(packet);p['now']=103;assert not snapshot(p)['path_valid']
p=copy.deepcopy(packet);p['now']=103;p['status']['data'].update(stamp=103,planning_mode='upstream_fsm');assert snapshot(p)['path_valid']
p=copy.deepcopy(packet);p['grid']['receipt_age_s']=2;assert not snapshot(p)['grid_valid'] and snapshot(p)['grid_reason']=='GRID_STALE'
p=copy.deepcopy(packet);p['grid']['data']['count']=0;assert snapshot(p)['grid_valid'] and snapshot(p)['grid_reason']=='EMPTY_GRID'
p=copy.deepcopy(packet);p['status']['data']['phase']='CLOUD_STALE';assert not snapshot(p)['grid_valid'] and snapshot(p)['grid_reason']=='PLANNER_INPUT_STALE'
p=copy.deepcopy(packet);p['grid']['data']['resolution']=None;assert not snapshot(p)['grid_valid']
p=copy.deepcopy(packet);p['trajectory']['data']['x']=[float('nan')]*6;assert not snapshot(p)['path_valid']
topic={'source_age_s':.1,'receipt_age_s':.1,'hz':10.,'limit_s':.4,'frame':'camera_init','publishers':['/factory_laserMapping']}
r=format_packet({'topics':{'cloud':topic}},'test-only')
assert next(x for x in r['rows'] if x['key']=='cloud')['ok']
topic['frame']='body'
r=format_packet({'topics':{'cloud':topic}},'test-only')
assert next(x for x in r['rows'] if x['key']=='cloud')['state']=='坐标系错误'
result=format_packet({'topics':{}},'test-only');assert len(result['rows'])==11
fc=next(r for r in result['rows'] if r['key']=='fc');assert '未知' in fc['detail'] and '未解锁' not in fc['detail']
from telemetry_reader import cache_planner_status
cache={'status':{'data':{'session_id':'old'}},'trajectory':{'data':trajectory},'candidate_trajectory':{'data':trajectory},'grid':{'data':{}}}
cache_planner_status(cache,'status',{'session_id':'new'},1.)
assert 'trajectory' not in cache and 'candidate_trajectory' not in cache and 'grid' not in cache
p=copy.deepcopy(packet);p['status']['data']['goal_frame']='body';assert not snapshot(p)['path_valid']
# 机内候选是独立预览；仅匹配当前会话、目标、轨迹且可确认时显示。
p=copy.deepcopy(packet);p['status']['data'].update(session_id='s',phase='WAIT_TARGET',planning_mode='upstream_fsm',candidate={'available':True,'phase':'READY','id':9,'trajectory_id':9,'confirm_token':'s:candidate:9'})
p['candidate_trajectory']={'data':dict(trajectory,id=9),'receipt_age_s':.1}
assert snapshot(p)['candidate_path_valid'] and not snapshot(p)['path_valid']
for key,value in [('confirm_token','other:candidate:9'),('trajectory_id',10),('available',False),('phase','INVALID')]:
 q=copy.deepcopy(p);q['status']['data']['candidate'][key]=value;assert not snapshot(q)['candidate_path_valid']
q=copy.deepcopy(p);q['status']['receipt_age_s']=1;assert not snapshot(q)['candidate_path_valid']
print(json.dumps({'passed':True,'checks':21,'scope':'native-display-only, no ROS publishers or flight commands'}))

# 实时期限不放宽；仅同会话旧栅格短暂保留，空帧/过期/输入故障不残留。
for age in [1.2,1.6,1.3,1.8,1.4]:
 p=copy.deepcopy(packet);p['grid']['receipt_age_s']=age;r=snapshot(p)
 assert r['grid_display_valid']
 assert r['grid_valid']==(age<=1.5)
p=copy.deepcopy(packet);p['grid']['receipt_age_s']=3.1;assert not snapshot(p)['grid_display_valid']
p=copy.deepcopy(packet);p['status']['data']['phase']='CLOUD_STALE';assert not snapshot(p)['grid_display_valid']
p=copy.deepcopy(packet);p['grid']['data']['count']=0;assert snapshot(p)['grid_display_valid'] and snapshot(p)['grid_count']==0
print('栅格临界期限显示保持测试通过；实时有效期限仍为1.5秒')

# ROS2使用实际camera_init，不能将其误判为旧world或接受跨会话栅格。
p=copy.deepcopy(packet);p["status"]["data"]["goal_frame"]="camera_init";p["grid"]["data"]["frame"]="camera_init";assert snapshot(p)["grid_valid"]
p["grid"]["data"]["session_id"]="old";assert not snapshot(p)["grid_display_valid"]
p=copy.deepcopy(packet);p["grid"]["data"]["stamp"]=now-4;assert not snapshot(p)["grid_display_valid"]
p=copy.deepcopy(packet);p["grid"]["data"]["stamp"]=now+1;assert not snapshot(p)["grid_valid"]
