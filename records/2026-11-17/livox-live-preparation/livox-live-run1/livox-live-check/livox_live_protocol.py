#!/usr/bin/env python3
"""Bounded, checksummed transport for unmodified ROS1 Livox/IMU serialization."""
import struct
import zlib
HEADER = struct.Struct('!BIIQ')
TOPICS = {1: '/livox/imu', 2: '/livox/lidar'}
MAX_BYTES = 16 * 1024 * 1024


def write_frame(stream, kind, payload, receipt_ns):
    if kind not in (1, 2, 255) or len(payload) > MAX_BYTES:
        raise ValueError('invalid frame type/size')
    encoded = zlib.compress(payload, 1)
    if len(encoded) > MAX_BYTES:
        raise ValueError('compressed frame too large')
    stream.write(HEADER.pack(kind, len(encoded), zlib.crc32(payload), receipt_ns))
    stream.write(encoded)
    stream.flush()


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
