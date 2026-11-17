#!/usr/bin/env python3
"""Match actual captured LIO/relay poses by source stamp and report geometry."""
import argparse
import json
import math
from pathlib import Path


def rpy(pose):
    x, y, z, w = pose[3:]
    norm = x*x+y*y+z*z+w*w
    return [math.atan2(2*(w*x+y*z), norm-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)/norm))),
            math.atan2(2*(w*z+x*y), norm-2*(y*y+z*z))]


def analyze(root):
    odom = [json.loads(line) for line in (root/'odom-headers.jsonl').read_text().splitlines()]
    relay = [json.loads(line) for line in (root/'vision-headers.jsonl').read_text().splitlines()]
    source = {row['stamp_ns']: row for row in odom}
    unmatched = [r['stamp_ns'] for r in relay if r['stamp_ns'] not in source]
    if not odom or not relay or unmatched:
        raise ValueError('missing or unmatched physical pose samples')
    offsets, angles, delays = [], [], []
    for row in relay:
        original = source[row['stamp_ns']]
        offsets.append([original['pose'][i]-row['pose'][i] for i in range(3)])
        angles.append([a-b for a, b in zip(rpy(original['pose']), rpy(row['pose']))])
        delays.append((row['receipt_monotonic_ns']-original['receipt_monotonic_ns'])/1e9)
    translation_residual = max(math.dist(p, offsets[0]) for p in offsets)
    angular_residual = max(math.dist(a, angles[0]) for a in angles)
    return {
        'scope': 'actual recorded source-stamp pairs; legacy position/RPY zeroing consistency; no external accuracy reference',
        'matched_pairs': len(relay), 'unmatched': unmatched,
        'position_zero_m': offsets[0], 'position_zero_residual_max_m': translation_residual,
        'rpy_zero_rad': angles[0], 'rpy_zero_residual_max_rad': angular_residual,
        'source_axis_span_m': [max(r['pose'][i] for r in odom)-min(r['pose'][i] for r in odom) for i in range(3)],
        'source_first_to_last_distance_m': math.dist(odom[0]['pose'][:3], odom[-1]['pose'][:3]),
        'observer_receipt_difference_s': {'min': min(delays), 'max': max(delays)},
        'passed': translation_residual < 1e-9 and angular_residual < 1e-9,
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('capture', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    report = analyze(args.capture)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))
    raise SystemExit(0 if report['passed'] else 1)
