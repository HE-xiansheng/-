#!/usr/bin/env python3
"""Query existing PX4 listeners over the live MAVROS router, without reconfiguration.

Packets must be encoded by the recorded official MAVLink generator. The only
permitted payloads are the fixed telemetry listeners below and closing our shell.
"""
import json
import math
import struct
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import Mavlink, State
from rcl_interfaces.srv import GetParameters

ALLOWED = {'listener estimator_status_flags 1\n',
           'listener estimator_aid_src_ev_hgt 1\n',
           'listener vehicle_visual_odometry 1\n'}


def validate(packet):
    assert packet['msgid'] == 126 and packet['sysid'] == 245 and packet['compid'] == 190
    assert packet['magic'] == 253 and packet['incompat_flags'] == packet['compat_flags'] == 0
    assert 1 <= packet['len'] <= 81
    raw = b''.join(struct.pack('<Q', v) for v in packet['payload64'])[:packet['len']].ljust(81, b'\0')
    baud, timeout, device, flags, count = struct.unpack_from('<IHBBB', raw)
    assert baud == timeout == 0 and device == 10 and raw[79:] == b'\0\0'
    assert count <= 70 and not any(raw[9+count:79])
    command = raw[9:9+count].decode('ascii')
    assert (flags == 2 and command in ALLOWED) or (flags == 0 and count == 0)
    return command


def main(packets):
    for packet in packets:
        validate(packet)
    assert len(packets) == 7 and validate(packets[-1]) == ''
    assert all(validate(p) in ALLOWED for p in packets[:-1])
    rclpy.init()
    node = rclpy.create_node('px4_height_readonly_query', enable_rosout=False)
    incoming, states, requests = [], [], []
    parameters = {}
    current = [None]

    def receive(message):
        if message.msgid != 126 or message.sysid != 1 or message.compid != 1:
            return
        raw = b''.join(struct.pack('<Q', v) for v in message.payload64)[:message.len].ljust(81,b'\0')
        count = raw[8]
        if count > 70:
            return
        text = raw[9:9+count].decode('utf8', errors='replace')
        incoming.append(dict(wall=time.time(),query=current[0],text=text))

    source = node.create_subscription(Mavlink, '/uas1/mavlink_source', receive, qos_profile_sensor_data)
    state = node.create_subscription(State, '/mavros/state', lambda m: states.append(dict(at=time.monotonic(),connected=m.connected,armed=m.armed)), qos_profile_sensor_data)
    sink = node.create_publisher(Mavlink, '/uas1/mavlink_sink', 10)

    def disarmed():
        return bool(states and states[-1]['connected'] and not states[-1]['armed'] and time.monotonic()-states[-1]['at']<2.5)

    def send(packet):
        validate(packet)
        if packet['msgid'] != 126:
            raise RuntimeError('only fixed read-only SERIAL_CONTROL queries allowed')
        message = Mavlink()
        for key,value in packet.items():
            setattr(message,key,value)
        message.header.stamp = node.get_clock().now().to_msg()
        sink.publish(message)

    try:
        deadline = time.monotonic()+7
        while time.monotonic()<deadline and not (disarmed() and sink.get_subscription_count()):
            rclpy.spin_once(node,timeout_sec=.05)
        if not disarmed() or not sink.get_subscription_count():
            raise RuntimeError('connected/disarmed FCU and existing router required')
        client = node.create_client(GetParameters,'/mavros/param/get_parameters')
        request = GetParameters.Request();request.names=['EKF2_EV_CTRL','EKF2_HGT_REF','EKF2_BARO_CTRL','EKF2_GPS_CTRL','EKF2_RNG_CTRL']
        if client.wait_for_service(timeout_sec=1):
            future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=2)
            if future.done() and future.result():
                parameters={name:dict(type=v.type,integer=v.integer_value,double=v.double_value) for name,v in zip(request.names,future.result().values)}
        for packet in packets[:-1]:
            if not disarmed():
                raise RuntimeError('FCU status not fresh/disarmed; ending read-only query')
            current[0]=validate(packet);requests.append(dict(wall=time.time(),command=current[0]));send(packet)
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.02)
        print(json.dumps(dict(parameters=parameters,requests=requests,incoming=incoming,states=states),ensure_ascii=False),flush=True)
    finally:
        if requests:
            send(packets[-1])
            deadline=time.monotonic()+.2
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.02)
        node.destroy_node();rclpy.try_shutdown()


# The workstation appends the validated official-generator packet literals and
# calls main. Executing this file alone does not publish any message.
