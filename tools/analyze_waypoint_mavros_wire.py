#!/usr/bin/env python3
"""Compare actual native waypoint ROS commands against PX4-side MAVLink/ULog."""
import json
import argparse
import math
from pathlib import Path
import sys
from pyulog import ULog


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('base', type=Path)
    parser.add_argument('--fixture-subdir', default='waypoint-mavros-wire',
                        choices=['waypoint-mavros-wire','ego-mavros-wire'])
    args = parser.parse_args()
    base = args.base
    fixture = json.loads((base/args.fixture_subdir/'result.json').read_text())
    transport = json.loads((base/'px4-runtime/transport.json').read_text())
    runner = json.loads((base/'runner-result.json').read_text())
    source = fixture['commands']
    wire = [p for p in transport['packets'] if p['mavpackettype']=='SET_POSITION_TARGET_LOCAL_NED']
    checks = dict(fixture['checks'])
    checks['runner_clean_exit'] = runner['passed'] and runner['remote_removed']
    checks['px4_loopback_only'] = transport['interfaces']==['lo']
    checks['px4_normal_exit'] = transport['process_returncode']==0
    def boot_ms(ns):
        return (ns//1000000) & 0xffffffff
    by_stamp = {boot_ms(c['stamp_ns']):c for c in source}
    checks['unique_source_millisecond_stamps'] = len(by_stamp)==len(source)
    checks['wire_source_counts_equal'] = len(source)==len(wire) and len(wire)>100
    checks['all_wire_source_stamps_match'] = all(p['time_boot_ms'] in by_stamp for p in wire)
    checks['all_source_stamps_reach_wire'] = {p['time_boot_ms'] for p in wire}==set(by_stamp)
    errors = []
    for p in wire:
        c = by_stamp.get(p['time_boot_ms'])
        if c is None:
            continue
        expected = {}
        for source_key, fields in [('position',['x','y','z']),('velocity',['vx','vy','vz']),('acceleration',['afx','afy','afz'])]:
            x,y,z = c[source_key]
            expected.update(zip(fields,[y,x,-z]))
        expected['yaw'] = math.atan2(math.sin(math.pi/2-c['yaw']),math.cos(math.pi/2-c['yaw']))
        expected['yaw_rate'] = -c['yaw_rate']
        for field,value in expected.items():
            error = abs(p[field]-value)
            if error>1e-5:
                errors.append(dict(stamp=p['time_boot_ms'],field=field,expected=value,actual=p[field]))
    checks['enu_ned_position_velocity_acceleration_yaw_match'] = bool(wire) and not errors
    checks['masks_and_frames_match'] = bool(wire) and all(p['coordinate_frame']==1 and p['type_mask']==by_stamp[p['time_boot_ms']]['type_mask'] for p in wire if p['time_boot_ms'] in by_stamp)
    checks['target_system_component_one'] = bool(wire) and all(p['target_system']==1 and p['target_component']==1 for p in wire)
    ulog_path = list((base/'px4-runtime/log').rglob('*.ulg'))
    assert len(ulog_path)==1,ulog_path
    log = ULog(str(ulog_path[0]))
    offboard = log.get_dataset('offboard_control_mode').data
    status = log.get_dataset('vehicle_status').data
    checks['px4_decoded_position_and_velocity_inputs'] = bool(any(offboard['position']) and any(offboard['velocity']))
    if args.fixture_subdir=='ego-mavros-wire':
        checks['px4_decoded_acceleration_input'] = bool(any(offboard['acceleration']))
    # PX4 v1.16 vehicle_status arming_state=2 is armed, nav_state=14 is OFFBOARD.
    checks['ulog_never_armed'] = bool(len(status['arming_state'])) and all(int(x)!=2 for x in status['arming_state'])
    checks['ulog_never_offboard'] = all(int(x)!=14 for x in status['nav_state'])
    wire_stamps={p['time_boot_ms'] for p in wire}
    missing=[i for i,c in enumerate(source) if boot_ms(c['stamp_ns']) not in wire_stamps]
    first_wire_source=next((i for i,c in enumerate(source) if boot_ms(c['stamp_ns']) in wire_stamps),None)
    prefix_only=first_wire_source is not None and missing==list(range(first_wire_source))
    result = dict(passed=all(checks.values()),checks=checks,source_count=len(source),wire_count=len(wire),
        unmatched_source_indices=missing,unmatched_is_initial_prefix_only=prefix_only,
        initial_unmatched_span_seconds=(source[first_wire_source]['stamp_ns']-source[0]['stamp_ns'])/1e9 if first_wire_source is not None else None,
        # 全流严格比较保留失败；这些诊断字段不豁免缺失或改变passed。
        geometry_errors=errors,ulog=str(ulog_path[0].relative_to(base)),
        px4_offboard_control_mode_samples=len(offboard['timestamp']),
        scope=args.fixture_subdir+': synthetic inputs -> actual native controller -> MAVROS -> PX4 decode; no armed flight dynamics or real hardware mission')
    (base/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
