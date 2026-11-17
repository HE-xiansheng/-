#!/usr/bin/env python3
"""Read-only SDK2 search on one NIC; never send configuration commands.

Packet layout/CRC from Livox SDK2 sdk_protocol.cpp and comm/define.h.
Run with a temporary address already assigned to the sensor interface.
"""
import binascii
import json
import select
import socket
import struct
import time
import zlib

interface = 'enP3p49s0'
host = '192.168.1.51'
index = socket.if_nametoindex(interface)
sockets = []
for address in (host, '255.255.255.255'):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.IPPROTO_IP, 8, 1)  # Linux IP_PKTINFO
    sock.bind((address, 56000))
    sockets.append(sock)
started = time.monotonic()
last_send = -10
sequence = 0
devices = {}
received = []
while time.monotonic() - started < 20:
    elapsed = time.monotonic() - started
    if elapsed - last_send >= 1:
        sequence += 1
        header = struct.pack('<BBHIHBB6s', 0xaa, 0, 24, sequence, 0, 0, 0, b'\0'*6)
        packet = header + struct.pack('<HI', binascii.crc_hqx(header, 0xffff), 0)
        pktinfo = struct.pack('I4s4s', index, socket.inet_aton(host), b'\0'*4)
        sockets[0].sendmsg([packet], [(socket.IPPROTO_IP, 8, pktinfo)], 0,
                           ('255.255.255.255', 56000))
        last_send = elapsed
    ready, _, _ = select.select(sockets, [], [], .2)
    for sock in ready:
        data, ancillary, flags, source = sock.recvmsg(65535, 256)
        incoming_index = None
        for level, kind, value in ancillary:
            if level == socket.IPPROTO_IP and kind == 8:
                incoming_index = struct.unpack('I', value[:4])[0]
        if incoming_index != index or source[0] == host:
            continue
        item = {'source': source, 'bytes': len(data), 'hex': data.hex(),
                'elapsed': elapsed, 'interface_index': incoming_index}
        received.append(item)
        if len(data) < 48 or data[0:2] != b'\xaa\x00':
            continue
        length, = struct.unpack_from('<H', data, 2)
        command, = struct.unpack_from('<H', data, 8)
        crc16, crc32 = struct.unpack_from('<HI', data, 18)
        if (length != len(data) or command != 0 or data[10] != 1 or
                crc16 != binascii.crc_hqx(data[:18], 0xffff) or
                crc32 != zlib.crc32(data[24:])):
            continue
        status, kind, serial, ip, port = struct.unpack_from('<BB16s4sH', data, 24)
        device = {'return_code': status, 'device_type': kind,
                  'serial': serial.rstrip(b'\0').decode('ascii', 'replace'),
                  'ip': socket.inet_ntoa(ip), 'command_port': port, 'source': source}
        devices[device['serial']] = device
        print(json.dumps({'detection': device}), flush=True)
for sock in sockets:
    sock.close()
print(json.dumps({'interface': interface, 'host': host,
                  'devices': list(devices.values()), 'received': received}), flush=True)
