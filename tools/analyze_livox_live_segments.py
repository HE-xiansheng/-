#!/usr/bin/env python3
"""Join source/forwarder/receiver timing without changing acceptance thresholds."""
import bisect
import csv
import json
from pathlib import Path
import statistics
import sys
from livox_live_protocol import HEADER

root = Path(sys.argv[1])
result = json.loads((root/'livox-live-check/result.json').read_text())
source = {(r[0], r[1]): r for r in result['source']['transport_rows']}
direct = 'temporary authenticated TCP' in result['scope']
chunks = list(csv.DictReader((root/'transport-chunks.csv').open()))
chunk_ends = [int(r['offset_end']) for r in chunks]
local = {}
with (root/'source.stream').open('rb') as stream:
    while True:
        header = stream.read(HEADER.size)
        if not header:
            break
        if len(header) != HEADER.size:
            raise ValueError('truncated trace frame')
        kind, size, _, receipt = HEADER.unpack(header)
        stream.seek(size, 1)
        index = bisect.bisect_left(chunk_ends, stream.tell())
        if kind in (1, 2):
            local[(kind, receipt)] = chunks[index]
rows = []
for r in csv.DictReader((root/'livox-live-check/input-headers.csv').open()):
    key = int(r['kind']), int(r['source_receipt_ns'])
    s = source[key]
    received = int(r['receive_wall_ns'])
    published = int(r['publish_wall_ns'])
    rows.append({'kind': key[0], 'receipt_ns': key[1],
        'source_queue_s': (s[2]-s[1])/1e9,
        'source_write_s': (s[3]-s[2])/1e9,
        'target_queue_and_publish_s': (published-received)/1e9,
        'source_age_s': float(r['source_age_s']),
        'target_decode_ms': float(r['decode_publish_ms'])})
    if direct:
        rows[-1]['source_begin_to_target_read_s'] = (received-s[2])/1e9
    else:
        c = local[key]
        local_read = int(c['read_end_wall_ns'])
        local_forward = int(c['forward_end_wall_ns'])
        rows[-1].update(source_begin_to_local_read_s=(local_read-s[2])/1e9,
            local_forward_s=(local_forward-int(c['read_begin_wall_ns']))/1e9,
            local_read_to_target_read_s=(received-local_read)/1e9)
    if len(s) >= 10:
        rows[-1].update(source_encode_wall_s=s[6]/1e9, source_encode_cpu_s=s[7]/1e9,
                        source_pipe_write_s=s[8]/1e9, source_encoded_bytes=s[9])


def stats(values):
    ordered = sorted(values)
    return {'min': ordered[0], 'median': statistics.median(ordered),
            'p95': ordered[int((len(ordered)-1)*.95)], 'max': ordered[-1]}


report = {'scope': 'diagnostic only; original strict acceptance remains unchanged',
    'path': 'direct authenticated TCP' if direct else 'SSH through computer',
    'joined_inputs': len(rows), 'source_rows': len(source), 'local_frames': len(local),
    'clock_note': 'Cross-host wall intervals require clock-offset-bounds.json; write/read overlap and clock uncertainty can give small negative values.',
    'streams': {}, 'worst_age_inputs': sorted(rows, key=lambda r:r['source_age_s'], reverse=True)[:10]}
for kind in (1, 2):
    selected = [r for r in rows if r['kind'] == kind]
    report['streams'][kind] = {key: stats([r[key] for r in selected]) for key in rows[0]
                              if key not in ('kind', 'receipt_ns')}
(root/'segment-analysis.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
