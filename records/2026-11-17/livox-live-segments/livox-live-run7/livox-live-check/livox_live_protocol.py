#!/usr/bin/env python3
"""Bounded, checksummed transport for unmodified ROS1 Livox/IMU serialization."""
import struct
import time
import zlib
HEADER = struct.Struct('!BIIQ')
TOPICS = {1: '/livox/imu', 2: '/livox/lidar'}
MAX_BYTES = 16 * 1024 * 1024


def ros1_sensor_to_cdr(kind, raw):
    """Encode the locked Imu/CustomMsg schemas without per-point Python objects."""
    if kind not in TOPICS or len(raw) < 16:
        raise ValueError('invalid sensor schema')
    _, sec, ns, size = struct.unpack_from('<IIII', raw)
    offset = 16+size
    if sec > 0x7fffffff or ns >= 10**9 or offset > len(raw):
        raise ValueError('invalid sensor header')
    frame = raw[16:offset]
    frame.decode('utf-8')
    cdr = bytearray(b'\x00\x01\x00\x00')
    cdr.extend(struct.pack('<iII', sec, ns, size+1))
    cdr.extend(frame+b'\x00')
    cdr.extend(b'\x00' * (-(len(cdr)-4) % 8))
    if kind == 1:
        if len(raw)-offset != 37*8:
            raise ValueError('invalid Imu schema length')
        cdr.extend(raw[offset:])
    else:
        if len(raw)-offset < 20:
            raise ValueError('invalid CustomMsg schema length')
        count = struct.unpack_from('<I', raw, offset+16)[0]
        if len(raw)-offset-20 != count*19 or struct.unpack_from('<I', raw, offset+8)[0] != count:
            raise ValueError('invalid CustomMsg point count')
        cdr.extend(raw[offset:offset+20])
        # CDR aligns each CustomPoint to four bytes; its last element has no
        # trailing padding. ROS1 packs each element into exactly 19 bytes.
        points = raw[offset+20:]
        padded = bytearray(count*20)
        for column in range(19):
            padded[column::20] = points[column::19]
        cdr.extend(padded[:-1] if count else padded)
    return bytes(cdr), sec*10**9+ns, frame.decode('utf-8')


def write_frame(stream, kind, payload, receipt_ns):
    if kind not in (1, 2, 255) or len(payload) > MAX_BYTES:
        raise ValueError('invalid frame type/size')
    begin = time.monotonic_ns()
    cpu_begin = time.thread_time_ns()
    encoded = zlib.compress(payload, 1)
    encoded_at = time.monotonic_ns()
    encode_cpu_ns = time.thread_time_ns()-cpu_begin
    if len(encoded) > MAX_BYTES:
        raise ValueError('compressed frame too large')
    stream.write(HEADER.pack(kind, len(encoded), zlib.crc32(payload), receipt_ns))
    stream.write(encoded)
    stream.flush()
    return {'encode_wall_ns': encoded_at-begin, 'encode_cpu_ns': encode_cpu_ns,
            'write_wall_ns': time.monotonic_ns()-encoded_at, 'encoded_bytes': len(encoded)}


def read_exact(stream, count, allow_eof=False):
    chunks = bytearray()
    while len(chunks) < count:
        chunk = stream.read(count-len(chunks))
        if not chunk:
            if allow_eof and not chunks:
                return None
            raise ValueError('truncated live input')
        chunks.extend(chunk)
    return bytes(chunks)


def read_frame(stream):
    data = read_exact(stream, HEADER.size, allow_eof=True)
    if data is None:
        return None
    kind, size, crc, receipt_ns = HEADER.unpack(data)
    if kind not in (1, 2, 255) or not 0 < size <= MAX_BYTES:
        raise ValueError('invalid frame type/size')
    decoder = zlib.decompressobj()
    payload = decoder.decompress(read_exact(stream, size), MAX_BYTES+1)
    if len(payload) > MAX_BYTES or not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:
        raise ValueError('invalid compressed payload')
    if zlib.crc32(payload) != crc:
        raise ValueError('live input checksum mismatch')
    return kind, payload, receipt_ns
