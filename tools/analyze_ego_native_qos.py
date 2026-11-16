#!/usr/bin/env python3
"""Describe equal-QoS native observation without treating missing frames as success."""
import json
from pathlib import Path
import sys

base=Path(sys.argv[1]);remote=base/'ego-mavros-wire'
trace=[json.loads(line) for line in (remote/'publisher-trace.jsonl').read_text().splitlines()]
records=[json.loads(line) for line in (remote/'native-observer.log').read_text().splitlines() if line.startswith('{')]
stamp=lambda ns:(ns//1000000)&0xffffffff
source={stamp(t['stamp_ns']):t for t in trace}
transport=json.loads((base/'px4-runtime/transport.json').read_text())
wire={p['time_boot_ms'] for p in transport['packets'] if p['mavpackettype']=='SET_POSITION_TARGET_LOCAL_NED'}
groups={kind:{(r['time_boot_ms'] if kind=='sink' else stamp(r['stamp_ns']))
              for r in records if r['kind']==kind} for kind in ['best_effort','reliable','sink']}
missing={kind:sorted(set(source)-group) for kind,group in groups.items()}
missing['wire']=sorted(set(source)-wire)
indices={kind:[i for i,t in enumerate(trace) if stamp(t['stamp_ns']) in absent] for kind,absent in missing.items()}
result=dict(scope='diagnostic comparison only; actual all-message acceptance remains analysis.json',
    emitted_count=len(trace),unique_emission_stamps=len(source),
    native_counts={kind:len(group) for kind,group in groups.items()},wire_count=len(wire),
    rmw=[r['rmw'] for r in records if r['kind']=='ready'],
    missing_source_indices=indices,
    native_best_effort_missing_equals_wire_missing=missing['best_effort']==missing['wire'],
    native_sink_equals_wire=groups['sink']==wire,
    native_reliable_full_coverage=not missing['reliable'],
    wire_is_actual_source_subset=wire<=set(source),
    match_counts_for_wire_missing=[trace[i]['matched_subscriptions'] for i in indices['wire']],
    match_transitions=[t for t in trace if 'discovered_subscriptions' in t],
    limitation='Native observers share one executor, but are independent of MAVROS. Observations change load/scheduling. Matching delivery patterns alone do not identify a specific middleware bug.')
(base/'native-qos-analysis.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ['missing_source_indices','match_transitions']},indent=2))
