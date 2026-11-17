#!/usr/bin/env python3
"""Independently compare recorded real ROS poses with physical-wire NED poses."""
import json
import math
from pathlib import Path
import sys


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def rpy(q):
    x, y, z, w = q
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1, min(1, 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


base = Path(sys.argv[1])
wire = json.loads((base/'wire.json').read_text())
poses = json.loads((base/'poses.json').read_text())
sources = {p['stamp_ns']//1000: p for p in poses['vision']}
packets = [p for p in wire['outgoing'] if p['mavpackettype'] == 'VISION_POSITION_ESTIMATE']
errors = []
position_errors = []
missing = []
for packet in packets:
    p = sources.get(packet['usec'])
    if p is None:
        missing.append(packet['usec'])
        continue
    x, y, z = p['position']
    position_errors.append(max(abs(a-b) for a, b in zip((y, x, -z), (packet['x'], packet['y'], packet['z']))))
    # q(NED<-ENU) * q(ENU<-FLU) * q(FLU<-FRD).
    expected = rpy(multiply(multiply((math.sqrt(.5), math.sqrt(.5), 0, 0), p['q_xyzw']), (1, 0, 0, 0)))
    actual = (packet['roll'], packet['pitch'], packet['yaw'])
    errors.append(max(abs(math.atan2(math.sin(a-b), math.cos(a-b))) for a, b in zip(expected, actual)))
checks = {
    'sustained_real_wire_samples': len(packets) >= 400,
    'every_wire_stamp_matches_recorded_relay': bool(packets) and not missing,
    'ENU_NED_positions_match': bool(position_errors) and max(position_errors) < 1e-5,
    'ENU_FLU_to_NED_FRD_orientation_matches': bool(errors) and max(errors) < 1e-5,
    'source_quaternions_unit': bool(poses['vision']) and all(abs(sum(v*v for v in p['q_xyzw'])-1) < 1e-6 for p in poses['vision']),
    'no_rejected_packets_reached_wire': not wire['forbidden'],
}
result = {'checks': checks, 'passed': all(checks.values()), 'packets': len(packets),
          'unmatched_stamps': missing, 'max_position_error_m': max(position_errors, default=None),
          'max_orientation_error_rad': max(errors, default=None),
          'scope': 'actual relay ROS ENU/FLU to wire NED/FRD; does not establish mounting extrinsics'}
print(json.dumps(result, indent=2))
raise SystemExit(0 if result['passed'] else 1)
