#!/usr/bin/env python3
"""Describe startup delivery against actual publication-time DDS match counts."""
import json
from pathlib import Path
import sys


base=Path(sys.argv[1])
remote=base/'ego-mavros-wire'
trace=[json.loads(line) for line in (remote/'publisher-trace.jsonl').read_text().splitlines()]
fixture=json.loads((remote/'result.json').read_text())
transport=json.loads((base/'px4-runtime/transport.json').read_text())
wire=[p for p in transport['packets'] if p['mavpackettype']=='SET_POSITION_TARGET_LOCAL_NED']
stamp=lambda ns:(ns//1000000)&0xffffffff
source_by_stamp={stamp(t['stamp_ns']):t for t in trace}
wire_stamps={p['time_boot_ms'] for p in wire}
observer_stamps={stamp(c['stamp_ns']) for c in fixture['commands']}
sink=fixture.get('mavros_sink_commands',[])
sink_stamps={p['time_boot_ms'] for p in sink}
missing=[dict(index=i,stamp_ns=t['stamp_ns'],matched_subscriptions=t['matched_subscriptions'])
         for i,t in enumerate(trace) if stamp(t['stamp_ns']) not in wire_stamps]
transitions=[t for t in trace if 'discovered_subscriptions' in t]
result=dict(scope='diagnostic only; does not waive strict full-stream wire checks',
    emitted_count=len(trace),observer_count=len(observer_stamps),wire_count=len(wire),
    all_emissions_reached_wire=len(trace)==len(wire) and set(source_by_stamp)==wire_stamps,
    all_emissions_seen_by_fixture=set(source_by_stamp)==observer_stamps,
    all_wire_stamps_are_actual_emissions=wire_stamps<=set(source_by_stamp),
    wire_only_after_second_subscription=bool(wire) and all(source_by_stamp[p['time_boot_ms']]['matched_subscriptions']>=2 for p in wire if p['time_boot_ms'] in source_by_stamp),
    missing_only_with_fixture_subscription=bool(missing) and all(t['matched_subscriptions']==1 for t in missing),
    no_missing_after_second_subscription=all(stamp(t['stamp_ns']) in wire_stamps for t in trace if t['matched_subscriptions']>=2),
    unique_emission_millisecond_stamps=len(source_by_stamp)==len(trace),
    unmatched_emissions=missing,match_transitions=transitions,
    internal_sink_observed='mavros_sink_commands' in fixture,
    internal_sink_count=len(sink),
    internal_sink_source_coverage=len(sink_stamps & set(source_by_stamp)),
    internal_sink_contains_wire_missing_stamps=sorted((set(source_by_stamp)-wire_stamps)&sink_stamps),
    internal_sink_missing_source_stamps=sorted(set(source_by_stamp)-sink_stamps) if sink else None,
    interpretation='First endpoint is identified as the fixture. Second endpoint has unknown ROS node name; only MAVROS is the other command consumer in this isolated fixture. Counts prove match timing in this run, not the historical 9-message loss.')
(base/'discovery-analysis.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
