#!/usr/bin/env python3
"""保留并检查真实Qt桥接与canonical只读采样；不修改健康门槛。"""
import argparse
import json
from pathlib import Path
import statistics


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('gui_log',type=Path)
    args=parser.parse_args()
    repo=Path(__file__).resolve().parents[1]
    target=repo/'records/2026-11-17/base-autostart/gcs-sampler-check'
    rows=[]
    for line in args.gui_log.read_text().splitlines():
        try:value=json.loads(line)
        except ValueError:continue
        if value.get('type')=='PFS1' and value.get('connected'):rows.append(value)
    rows=rows[-180:]
    (target/'used-gcs-after.jsonl').write_text('\n'.join(json.dumps(v,ensure_ascii=False) for v in rows)+'\n')
    checks={'at least 30 seconds of GUI bridge snapshots':len(rows)>100 and rows[-1]['stamp']-rows[0]['stamp']>=30,
            'all GUI bridge samples connected':all(v['connected'] for v in rows),
            'no armed state in GUI snapshots':all('未解锁' in next(r for r in v['rows'] if r['key']=='fc')['detail'] for v in rows)}
    summary={}
    for key,low,high in [('lidar',8,12),('imu',150,240),('fc_imu',40,60),('fc_odom',25,35)]:
        # 飞控IMU是传感器面板条目；不假定每个传感器也在话题表里。
        series=[v['sensors'][key] if key=='fc_imu' else next(r for r in v['rows'] if r['key']==key) for v in rows]
        hz=[float(v['hz']) for v in series];age=[float(v['age']) for v in series]
        checks[key+' healthy at true rate throughout GUI observation']=all(v['ok'] for v in series) and min(hz)>low and max(hz)<high
        summary[key]={'hz_min':min(hz),'hz_median':statistics.median(hz),'hz_max':max(hz),'age_max_s':max(age)}
    checks['LIO and vision relay remain manually stopped']=all(not v['localization']['lio_running'] and not v['localization']['relay_running'] for v in rows)
    canonical=[json.loads(line) for line in (target/'canonical-live.jsonl').read_text().splitlines() if line.startswith('{')][5:]
    checks['canonical actual live sampler produced at least ten settled snapshots']=len(canonical)>=10
    for key,low,high in [('lidar',8,12),('imu',150,240),('fc_imu',40,60),('fc_odom',25,35)]:
        series=[v['topics'][key] for v in canonical]
        checks[key+' canonical original freshness and real frequency']=all(low<v['hz']<high and 0<=v['source_age_s']<v['limit_s'] and v['receipt_age_s']<v['limit_s'] and 'error' not in v for v in series)
        summary['canonical_'+key]={'hz_min':min(v['hz'] for v in series),'hz_max':max(v['hz'] for v in series),'source_age_max_s':max(v['source_age_s'] for v in series)}
    checks['canonical no external control publishers']=all(not v['control_publishers'] for v in canonical)
    checks['canonical PX4 connected and never armed']=all(v['topics']['fc']['values']['connected'] and not v['topics']['fc']['values']['armed'] for v in canonical)
    checks['no sampled FAST-LIO/relay source in canonical']=all(v['topics'][k]['samples']==0 for v in canonical for k in ('lio','relay'))
    value={'passed':all(checks.values()),'checks':checks,'used_gui_log':str(args.gui_log),'snapshot_count':len(rows),
           'duration_s':rows[-1]['stamp']-rows[0]['stamp'],'streams':summary,
           'scope':'existing Qt GUI bridge and independent canonical sampler; read-only base streams; no localization or task start'}
    (target/'live-results.json').write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(value,ensure_ascii=False))
    raise SystemExit(0 if value['passed'] else 1)


if __name__=='__main__':
    main()
