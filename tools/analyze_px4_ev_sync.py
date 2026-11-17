#!/usr/bin/env python3
"""Verify every reported EKF instance and measurement-clock mapping from raw NSH."""
import json
from pathlib import Path
import re
import sys


def parse(text):
    output = {}
    for key, value in re.findall(r'^\s*([a-zA-Z_][a-zA-Z_0-9]*):\s*([^\n]+)', text, re.M):
        value = value.strip()
        if value.lower() in ('true', 'false'):
            output[key] = value.lower() == 'true'
        elif re.match(r'^-?\d+(?:\.\d+)?', value):
            output[key] = float(re.match(r'^-?\d+(?:\.\d+)?', value).group())
        else:
            output[key] = value
    return output


base = Path(sys.argv[1])
wire = json.loads((base/'wire.json').read_text())
queries = []
for raw in wire['queries']:
    text = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', raw['text']).replace('\r', '\n')
    blocks = re.split(r'^Instance (\d+):\s*$', text, flags=re.M)
    instances = {int(blocks[i]): parse(blocks[i+1]) for i in range(1, len(blocks), 2)} if len(blocks) > 1 else {0: parse(text)}
    queries.append({'topic': raw['topic'], 'wall_ns': raw['wall_ns'], 'instances': instances})


def values(topic):
    return [q for q in queries if q['topic'] == topic]


checks = {}
flags = values('estimator_status_flags')
checks['all_reported_EKF_instances_fuse_EV_pose'] = len(flags) >= 2 and all(
    q['instances'] and all(all(v.get(k) is True for k in ('cs_ev_pos', 'cs_ev_hgt', 'cs_ev_yaw'))
                          and v.get('cs_ev_yaw_fault') is False for v in q['instances'].values()) for q in flags)
for topic in ('estimator_aid_src_ev_pos', 'estimator_aid_src_ev_hgt', 'estimator_aid_src_ev_yaw'):
    snapshots = values(topic)
    checks[topic+'_all_instances_actually_fused'] = len(snapshots) >= 2 and all(
        q['instances'] and all(v.get('fused') is True and v.get('innovation_rejected') is False
                              and 0 <= v.get('timestamp', 0)-v.get('time_last_fuse', 1e18) < 500000
                              for v in q['instances'].values()) for q in snapshots)
vision = [p for p in wire['outgoing'] if p['mavpackettype'] == 'VISION_POSITION_ESTIMATE']
sync = values('timesync_status')
mappings = []
for q in values('vehicle_visual_odometry'):
    state = q['instances'][0]
    nearest_sync = min(sync, key=lambda s: abs(s['wall_ns']-q['wall_ns'])) if sync else None
    clock = nearest_sync['instances'][0] if nearest_sync else {}
    sample = state.get('timestamp_sample', 0)
    remote_sample = sample-clock.get('estimated_offset', 0)
    nearest = min(vision, key=lambda p: abs(p['usec']-remote_sample)) if vision else {}
    position_text = state.get('position', '')
    try:
        position = json.loads(position_text)
    except (ValueError, TypeError):
        position = []
    position_error = max((abs(a-nearest.get(k, 1e18)) for a,k in zip(position, ('x','y','z'))), default=1e18)
    mappings.append({'snapshot_wall_ns': q['wall_ns'], 'sample_boot_usec': sample,
        'PX4_receive_minus_sample_usec': state.get('timestamp', 0)-sample,
        'offset_usec': clock.get('estimated_offset'), 'round_trip_usec': clock.get('round_trip_time'),
        'predicted_source_usec': remote_sample, 'matched_wire_usec': nearest.get('usec'),
        'wire_stamp_error_usec': abs(nearest.get('usec', 1e18)-remote_sample),
        'matched_wire_position_error_m': position_error})
checks['PX4_samples_map_to_actual_host_source_clock'] = len(mappings) >= 2 and all(
    m['wire_stamp_error_usec'] < 5000 and m['matched_wire_position_error_m'] < 1e-4
    and 0 < m['round_trip_usec'] < 10000 for m in mappings)
checks['PX4_preserves_measurement_age_after_sync'] = len(mappings) >= 2 and all(
    1000 < m['PX4_receive_minus_sample_usec'] < 500000 for m in mappings)
report = {'checks': checks, 'passed': all(checks.values()), 'clock_mappings': mappings,
          'EKF_snapshots': [q for q in queries if q['topic'].startswith('estimator_')],
          'scope': 'reported EKF instances, actual aid-source fusion and host source-clock mapping; static ground snapshots'}
print(json.dumps(report, indent=2))
raise SystemExit(0 if report['passed'] else 1)
