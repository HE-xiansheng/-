#!/usr/bin/env python3
"""Test-only publication tracing; executes the adjacent unchanged EGO adapter."""
import json
from pathlib import Path
import runpy
import sys
import time

from rclpy.node import Node
from rclpy.publisher import Publisher

base=Path(__file__).resolve().parent
nodes={}
create=Node.create_publisher
publish=Publisher.publish
trace=(base/'publisher-trace.jsonl').open('w',buffering=1)
last_count={}


def create_traced(node,*args,**kwargs):
    result=create(node,*args,**kwargs)
    nodes[id(result)]=node
    return result


def publish_traced(pub,message):
    if pub.topic_name=='/mavros/setpoint_raw/local':
        count=pub.get_subscription_count()
        record=dict(stamp_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                    before_publish_monotonic=time.monotonic(),matched_subscriptions=count)
        if last_count.get(id(pub))!=count:
            node=nodes[id(pub)]
            record['discovered_subscriptions']=[dict(node_name=s.node_name,node_namespace=s.node_namespace,
                reliability=int(s.qos_profile.reliability),durability=int(s.qos_profile.durability))
                for s in node.get_subscriptions_info_by_topic(pub.topic_name)]
            last_count[id(pub)]=count
        trace.write(json.dumps(record)+'\n')
    return publish(pub,message)


Node.create_publisher=create_traced
Publisher.publish=publish_traced
sys.argv[0]=str(base/'adapter.py')
try:
    runpy.run_path(sys.argv[0],run_name='__main__')
finally:
    trace.close()
