#!/usr/bin/env python3
"""从电脑已取回的原始记录提取摘要；不修改原始验收结果。"""
import argparse
import collections
import csv
import json
import re
from pathlib import Path


def distribution_ms(values):
    values = sorted(values)
    if not values:
        return None
    return {
        name: round(values[int((len(values) - 1) * quantile)] / 1e6, 6)
        for name, quantile in [('min', 0), ('p50', .5), ('p95', .95),
                               ('p99', .99), ('max', 1)]
    }


def analyze(run):
    root = run / 'fixture/platform-soak-check'
    result = json.loads((root / 'result.json').read_text())
    qt = json.loads((run / 'qt-result.json').read_text())
    summary = {
        'overall_passed': json.loads((run / 'result.json').read_text())['passed'],
        'qt_passed': sum(qt['checks'].values()),
        'qt_total': len(qt['checks']),
        'onboard_passed': sum(result['checks'].values()),
        'onboard_total': len(result['checks']),
        'failed': [key for key, value in result['checks'].items() if not value],
        'diagnostic_overrides': result.get('diagnostic_overrides', {}),
        'streams': result['stream_results'],
        'coverage': result['coverage'],
        'planner_phases': dict(collections.Counter(
            row['phase'] for row in result['planner_status'])),
        'cleanup': json.loads((run / 'fixture/runner-result.json').read_text()),
    }
    summary['lio_self_sync_logged_s'] = [float(value) for value in re.findall(
        r'Self sync IMU and LiDAR, time diff is ([-\d.]+)',
        (root / 'lio.log').read_text(errors='replace'))]
    received = collections.defaultdict(list)
    with (root / 'gimbal-headers.csv').open() as source:
        for row in csv.DictReader(source):
            received[row['qos'], row['key']].append(int(row['stamp_ns']))
    published = collections.defaultdict(list)
    for kind, stamp in re.findall(r'published_(raw|color) stamp_ns=(\d+)',
                                 (root / 'mpp.log').read_text(errors='replace')):
        published[kind].append(int(stamp))
    summary['mpp_delivery'] = {}
    for kind, stamps in published.items():
        for qos in ['reliable', 'best_effort']:
            seen = received[qos, kind]
            summary['mpp_delivery'][qos + '_' + kind] = {
                'published': len(stamps), 'received': len(seen),
                'duplicate_received': len(seen) - len(set(seen)),
                'missing_source_stamps': sorted(set(stamps) - set(seen)),
                'unexpected_stamps': sorted(set(seen) - set(stamps)),
            }
    resources = result['resources']
    summary['resources'] = {
        'peak_c': max(value for sample in resources
                      for value in sample['temperature_c'].values()),
        'min_available_gib': min(sample['memory']['MemAvailable']
                                 for sample in resources) / 2**30,
        'oom_kill_counter_delta': (resources[-1]['vmstat']['oom_kill'] -
                                   resources[0]['vmstat']['oom_kill']),
    }
    timing = root / 'source-timing.csv'
    if timing.exists():
        grouped = collections.defaultdict(list)
        with timing.open() as source:
            for row in csv.DictReader(source):
                values = {key: int(value) for key, value in row.items()
                          if key != 'kind'}
                grouped['all', row['kind']].append(values)
                if result['begin_ns'] <= values['dds_source_ns'] <= result['end_ns']:
                    grouped['stable_by_dds_source_time', row['kind']].append(values)
        summary['source_timing_ms'] = {}
        for (window, kind), rows in sorted(grouped.items()):
            summary['source_timing_ms'][window + '/' + kind] = {
                'messages': len(rows),
                **{a + '_minus_' + b: distribution_ms([row[a] - row[b] for row in rows])
                   for a, b in [('dds_source_ns', 'stamp_ns'),
                                ('dds_received_ns', 'dds_source_ns'),
                                ('callback_wall_ns', 'dds_received_ns'),
                                ('callback_wall_ns', 'stamp_ns')]},
            }
    summary['limitations'] = (
        '诊断覆盖不是正式参数验收；元数据来自独立观察器，不能代替LIO内部回调排队时间。'
        '此轮没有运行真实MAVROS/飞控或任务控制器；不证明运动精度、安装外参或整平台完成。')
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    target = args.run / 'analysis.json'
    target.write_text(json.dumps(analyze(args.run), ensure_ascii=False, indent=2) + '\n')
    print(target)
