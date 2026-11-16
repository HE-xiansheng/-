"""Unknown, stale, conflicting or invalid data never counts as a passed preflight link."""
import json
from preflight_status import LABELS,EXPECTED,FLAGS,evaluate
packet={'topics':{}}
for k in LABELS:
 v={}
 if k in ['lio','relay','fc_odom']:v={'position':[0,0,.8],'quaternion':[0,0,0,1],'child_frame_id':'body'}
 if k=='fc':v={'connected':True,'armed':False,'mode':'POSCTL','system_status':3}
 if k=='ekf':v={f:True for f in FLAGS}
 if k=='battery':v={'voltage':24.,'percentage':.8,'current':2.}
 if k=='rc':v={'channels':[1500]*8,'rssi':255}
 frame='body' if k=='cloud' else 'camera_init'
 packet['topics'][k]={'source_age_s':.02,'receipt_age_s':.01,'limit_s':.4 if k not in ['fc','battery','ekf','landed','rc'] else 3,'hz':10.,'publishers':EXPECTED.get(k,['/mavros']),'values':v,'frame':frame,'topic':'/'+k}
assert not evaluate(packet)[1]
# A 1Hz snapshot must not flicker stale between packets just because the
# high-rate source has a 0.4s limit at the actual sampling time.
for phase in [0,.2,.4,.6,.8,1.2,2.5]:assert not evaluate(packet,elapsed=phase)[1]
assert evaluate(packet,elapsed=2.501)[1]
assert evaluate(packet,elapsed=.8)[0][0][4]=='0.02' # explicitly snapshot age
# 电池初始样本不足显示未知；频率统计窗口不改变源数据过期判断。
p=json.loads(json.dumps(packet));p['topics']['battery'].update(hz=None,limit_s=5.)
battery=next(r for r in evaluate(p)[0] if r[0]=='battery');assert battery[2:4]==('正常','—')
p['topics']['battery'].update(hz=.5,source_age_s=5.01,receipt_age_s=5.01)
battery=next(r for r in evaluate(p)[0] if r[0]=='battery');assert battery[2:4]==('缺失/过期','0.5')
for key,changes in [('lio',{'source_age_s':2}),('lio',{'receipt_age_s':-.01}),('lio',{'source_age_s':-.2}),('relay',{'publishers':['/other']}),('cloud',{'frame':'camera_init'}),('lio',{'values':{'position':[0,0,float('nan')],'quaternion':[0,0,0,1]}}),('battery',{'values':{'voltage':24.,'percentage':-1}}),('fc',{'values':{'connected':True,'mode':'POSCTL','system_status':8}}),('ekf',{'values':{f:False for f in FLAGS}})]:
 p=json.loads(json.dumps(packet));p['topics'][key].update(changes);assert evaluate(p)[1],(key,changes)
assert evaluate(packet,elapsed=8)[1]
assert evaluate({'topics':{}})[1]
assert evaluate(dict(packet,control_publishers={'/mavros/setpoint_raw/local':['/demo']}))[1]
print(json.dumps({'passed':True,'checks':['all-links-fresh','unknown-values-block','raw-LIO-not-cached-relay','source-identity','frames','system-state','EKF','battery','stale-monitor','competing-control'],'scope':'status interpretation only, not arming approval'}))
