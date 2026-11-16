"""纯显示适配：复用原PolyTraj采样，不生成或改变控制轨迹。"""
import math
import numpy as np
from planner_geometry import sample_poly,is_active


def snapshot(packet):
    now=packet.get('now',0.)
    def get(key,limit):
        row=packet.get(key,{})
        return row.get('data',{}) if 0<=row.get('receipt_age_s',math.inf)<=limit else {}
    status=get('status',.75);execution=get('execution',.75);trajectory=get('trajectory',math.inf)
    if (not (-.05<=now-status.get('stamp',0)<=.75) or status.get('mode')!='manual_preview'
        or status.get('goal_frame') not in ('camera_init','world') or status.get('height_reference')!='fastlio_session_origin'
        or status.get('planning_mode') not in ('measured_preview','upstream_fsm')):status={}
    if not (-.05<=now-execution.get('stamp',0)<=.75):execution={}
    result={'status':status,'execution':execution,'path':[],'path_valid':False,'candidate_path':[],'candidate_path_valid':False,'grid_valid':False,'grid_display_valid':False,
            'grid_reason':'WAIT_PLANNER' if not status else 'WAIT_GRID'}
    if status and trajectory and is_active(status,trajectory,now):
        try:
            xyz=sample_poly(trajectory)
            if not np.isfinite(xyz).all():raise ValueError('Nonfinite display samples')
            stride=max(1,int(math.ceil(len(xyz)/600)))
            points=xyz[::stride].tolist()
            if len(xyz)>1 and points[-1]!=xyz[-1].tolist():points.append(xyz[-1].tolist())
            result.update(path=points,path_valid=True,trajectory_id=trajectory['id'],
                          planning_mode=status.get('planning_mode'),start_time=trajectory['start_time'])
        except (KeyError,ValueError,TypeError,OverflowError):pass
    candidate=status.get('candidate',{}) if status else {}
    preview=get('candidate_trajectory',math.inf)
    expected_token=str(status.get('session_id',''))+':candidate:'+str(candidate.get('id',''))
    if (status.get('session_id') and status.get('planning_mode')=='upstream_fsm' and candidate.get('available') is True
        and candidate.get('phase')=='READY' and candidate.get('confirm_token')==expected_token
        and preview and preview.get('id')==candidate.get('trajectory_id')==candidate.get('id')):
        try:
            xyz=sample_poly(preview)
            if not np.isfinite(xyz).all():raise ValueError('Nonfinite candidate samples')
            stride=max(1,int(math.ceil(len(xyz)/600)));points=xyz[::stride].tolist()
            if len(xyz)>1 and points[-1]!=xyz[-1].tolist():points.append(xyz[-1].tolist())
            result.update(candidate_path=points,candidate_path_valid=True,candidate_trajectory_id=preview['id'])
        except (KeyError,ValueError,TypeError,OverflowError):pass
    grid_record=packet.get('grid',{})
    grid_age=grid_record.get('receipt_age_s')
    if isinstance(grid_age,(int,float)) and math.isfinite(grid_age) and grid_age>=0:
        result['grid_age_s']=grid_age
        if grid_age>1.5:result['grid_reason']='GRID_STALE'
    grid=get('grid',3.)
    if grid and status and status.get('phase') not in ('ODOM_STALE','CLOUD_STALE','INPUT_STALE_AFTER_PLAN'):
        # 源时刻、坐标和定位会话都匹配，接收新包不能刷新过期栅格。
        valid_source=(grid.get('frame')==status.get('goal_frame') and grid.get('session_id')==status.get('session_id') and bool(status.get('session_id')) and -.05<=now-grid.get('stamp',0)<=3.)
        # 1.5秒仍是实时期限；显示允许保留同会话旧帧至3秒，防止临界点整层闪灭。
        # 此字段仅供绘图，绝不用于目标确认、碰撞判断或执行器输入有效性。
        resolution=grid.get('resolution')
        valid=valid_source and isinstance(resolution,(float,int)) and math.isfinite(resolution) and resolution>0
        fresh=valid and grid_age<=1.5 and now-grid['stamp']<=1.5 and now-grid.get('stamp',0)<=1.5
        result.update(grid_valid=fresh,grid_display_valid=valid,
                      grid_reason=('EMPTY_GRID' if grid.get('count')==0 else 'READY') if fresh else 'GRID_STALE' if valid else 'INVALID_GRID',
                      grid_count=grid.get('count',0),grid_source_count=grid.get('source_count',0),
                      grid_sequence=grid.get('sequence'),grid_resolution=resolution)
    elif status and status.get('phase') in ('ODOM_STALE','CLOUD_STALE','INPUT_STALE_AFTER_PLAN'):
        result['grid_reason']='PLANNER_INPUT_STALE'

    return result
