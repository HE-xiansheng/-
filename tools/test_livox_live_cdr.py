#!/usr/bin/env python3
"""Compare fast conversion with independent ROS object decoding on captured data."""
import base64
import json
from pathlib import Path
import struct
import sys

from livox_live_protocol import read_frame, ros1_sensor_to_cdr, TOPICS
from validate_fastlio_replay import decode
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu
from rclpy.serialization import deserialize_message, serialize_message

samples = {}
with Path(sys.argv[1]).open('rb') as stream:
    while len(samples) < 2:
        item = read_frame(stream)
        if item is None:
            raise ValueError('missing sensor sample')
        samples.setdefault(item[0], item[1])
checks = {}
for kind, raw in samples.items():
    for frame in [b'', b'a', b'abc', b'abcd', b'1234567', b'12345678', b'123456789', '雷达'.encode()]:
        length = struct.unpack_from('<I', raw, 12)[0]
        changed = raw[:12]+struct.pack('<I', len(frame))+frame+raw[16+length:]
        serialized, stamp, actual_frame = ros1_sensor_to_cdr(kind, changed)
        _, _, reference = decode({'topic': TOPICS[kind], 'receipt_offset': 0,
                                  'serialized': base64.b64encode(changed).decode()})
        typ = Imu if kind == 1 else CustomMsg
        actual = deserialize_message(serialized, typ)
        checks[str(kind)+'_frame_'+frame.hex()] = (actual == reference and
            stamp == actual.header.stamp.sec*10**9+actual.header.stamp.nanosec and
            actual_frame == actual.header.frame_id and
            deserialize_message(serialize_message(actual), typ) == reference)
    try:
        ros1_sensor_to_cdr(kind, raw[:-1])
    except ValueError:
        checks[str(kind)+'_truncated_schema_rejected'] = True
    else:
        checks[str(kind)+'_truncated_schema_rejected'] = False
raw = samples[2]
length = struct.unpack_from('<I', raw, 12)[0]
offset = 16+length
empty = bytearray(raw[:offset+20])
struct.pack_into('<I', empty, offset+8, 0)
struct.pack_into('<I', empty, offset+16, 0)
serialized, _, _ = ros1_sensor_to_cdr(2, empty)
checks['empty_point_sequence'] = not deserialize_message(serialized, CustomMsg).points
bad = bytearray(samples[1])
struct.pack_into('<I', bad, 8, 10**9)
try:
    ros1_sensor_to_cdr(1, bad)
except ValueError:
    checks['invalid_nanoseconds_rejected'] = True
else:
    checks['invalid_nanoseconds_rejected'] = False
print(json.dumps({'checks': checks, 'passed': all(checks.values()),
                  'scope': 'all fields of real samples, CDR alignment variants, empty points and malformed inputs'}, indent=2))
raise SystemExit(0 if all(checks.values()) else 1)
