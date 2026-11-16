"""Last on-board snapshot status, with a separate host monitor expiry.

The ~1Hz transport is not the source message frequency. Source-age limits apply
at the snapshot; losing the monitor expires all rows after 2.5s. Control gates
use their independent, strict live-source checks, never this display policy.
"""
import math
LABELS={'lidar':'MID360点云','imu':'雷达IMU','lio':'FAST-LIO定位','cloud':'规划点云','relay':'外部Pose转发','fc_odom':'飞控里程计','fc':'飞控连接/模式','landed':'着地状态','ekf':'EKF估计','rc':'遥控输入','battery':'电池','text':'飞控提示'}
EXPECTED={'lio':['/factory_laserMapping'],'cloud':['/factory_laserMapping'],'relay':['/factory_vision2mavros']}
FLAGS=['attitude_status_flag','velocity_horiz_status_flag','velocity_vert_status_flag','pos_horiz_rel_status_flag','pos_vert_abs_status_flag']
def evaluate(packet,elapsed=0,expected_frames=None):
    rows=[];blocking=[];topics=packet.get('topics',{})
    frames={'lio':'camera_init','cloud':'body'}
    if expected_frames:frames.update(expected_frames)
    for key,label in LABELS.items():
        r=topics.get(key,{});age=r.get('source_age_s');receipt=r.get('receipt_age_s');hz=r.get('hz');values=r.get('values',{})
        fresh=(0<=elapsed<=2.5 and isinstance(age,(int,float)) and math.isfinite(age) and -.1<=age<=r.get('limit_s',.4) and isinstance(receipt,(int,float)) and 0<=receipt<=r.get('limit_s',.4))
        status='正常' if fresh else '缺失/过期'
        if fresh and key in EXPECTED and r.get('publishers')!=EXPECTED[key]:status='来源冲突'
        if fresh and key in EXPECTED and r.get('graph_fresh') is False:status='来源待核实'
        if fresh and key in frames and r.get('frame')!=frames[key]:status='坐标系错误'
        if fresh and key in ['lio','relay','fc_odom']:
            p=values.get('position',[]);q=values.get('quaternion',[])
            if (len(p)!=3 or len(q)!=4 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in p+q)
                or not .81<=sum(x*x for x in q)<=1.21):status='位姿无效'
            if key=='lio' and values.get('child_frame_id')!='body':status='坐标系错误'
        detail=''
        if key=='fc':
            detail='%s / %s'%(values.get('mode','未知'),'已解锁' if values.get('armed') else '未解锁')
            if not values.get('connected'):status='飞控未连接'
            if values.get('system_status') in [5,6,8]:status='系统异常('+str(values['system_status'])+')'
            if not fresh:status='缺失/过期';detail='连接/模式/解锁状态未知'
        if key=='ekf' and fresh:
            bad=[k for k in FLAGS if not values.get(k)]+[k for k in ['gps_glitch_status_flag','accel_error_status_flag'] if values.get(k)]
            detail='位置/速度有效' if not bad else ','.join(bad);status='正常' if not bad else '估计无效'
        if key=='battery':
            voltage=values.get('voltage');pct=values.get('percentage')
            detail=('%.2fV'%voltage if isinstance(voltage,(int,float)) else '电压未知')+(' / %.0f%%'%(pct*100) if isinstance(pct,(int,float)) and 0<=pct<=1 else ' / 电量未知')
            if fresh and (not isinstance(pct,(int,float)) or not 0<=pct<=1):status='电量未知'
            elif fresh and pct<.25:status='电量偏低'
        if key=='lio' and fresh:
            p=values.get('position',[])
            if len(p)==3:detail='Z %.3fm（定位原点，非离地）'%p[2]
        if key=='rc' and fresh:
            detail='%d通道 / RSSI %s'%(len(values.get('channels',[])),values.get('rssi','未知'))
            if len(values.get('channels',[]))<4 or values.get('rssi')==0:status='遥控输入异常'
        if key=='landed' and fresh:detail={0:'未知',1:'在地面',2:'在空中',3:'起飞中',4:'降落中'}.get(values.get('landed_state'),'未知')
        if key=='text':status='提示' if fresh else '无新提示';detail='新启动监视器不含旧提示历史'
        if key!='text' and status!='正常':blocking.append(label+':'+status)
        hz_text='%.1f'%hz if isinstance(hz,(int,float)) and math.isfinite(hz) and hz>=0 else '—'
        rows.append((key,label,status,hz_text,('%.2f'%age if isinstance(age,(int,float)) else '—'),','.join(r.get('publishers',[])) or '—',detail,r.get('topic','')))
    if packet.get('control_publishers'):blocking.append('存在真实控制发布者')
    return rows,blocking


def sensor_snapshot(packet):
    """数值与源时间一起传输；无数据、无效数字、过期均不能显示为实时。"""
    result={}
    delay=packet.get('_transport_delay_s',0.)
    for key in ('imu','fc_imu','lidar','pressure'):
        row=packet.get('topics',{}).get(key,{})
        ages=[row.get('receipt_age_s'),row.get('source_age_s')]
        limit=row.get('limit_s',1.)
        valid_age=all(isinstance(v,(int,float)) and math.isfinite(v) and -.2<=v<=limit-delay for v in ages)
        values=row.get('values',{})
        result[key]={'ok':bool(values) and valid_age,'values':values,'topic':row.get('topic',''),
                     'frame':row.get('frame',''),'hz':row.get('hz'),'age':max(ages)+delay if valid_age else None}
    return result
