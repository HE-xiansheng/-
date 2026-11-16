#!/usr/bin/env python3
"""FAST-LIO 世界点云与可选位姿 → 既有 PCD1/LZ4、POS1；只发送遥测，无 ROS 发布或控制接口。"""
import argparse
import ctypes
import ctypes.util
try:
    import fcntl
except ImportError:  # Windows 只运行前台遥测；生命周期由 windows_bridge 管理。
    fcntl = None
import json
import math
import os
import socket
import struct
import threading
import time
import uuid
import numpy as np

HEADER = struct.Struct('<IIHHIIH')
MAGIC = 0x50434431
POSE = struct.Struct('<IId7f')


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose_packet(msg, sequence, cloud_frame):
    """保留同一定位原点下的 IMU/body 位置和四元数，不增加坐标变换。"""
    if msg.header.frame_id not in ('camera_init','world') or msg.header.frame_id != cloud_frame or msg.child_frame_id != 'body':
        raise ValueError('位姿与点云frame不一致，或子frame不是body')
    p,q = msg.pose.pose.position,msg.pose.pose.orientation
    stamp = stamp_seconds(msg.header.stamp)
    values = [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    if stamp <= 0 or not math.isfinite(stamp) or not all(math.isfinite(v) for v in values):
        raise ValueError('位姿或时间戳无效')
    if abs(math.sqrt(sum(v*v for v in values[3:]))-1.) > .05:
        raise ValueError('姿态四元数无效')
    return POSE.pack(0x504F5331,sequence,stamp,*values),values


def xyz_bytes(msg, max_points):
    """按 PointCloud2 字段、行跨度和字节序读取；只做显示抽样，不改变坐标。"""
    if msg.width <= 0 or msg.height <= 0:
        return b'', 0
    if msg.point_step <= 0 or msg.row_step < msg.width*msg.point_step or len(msg.data) < msg.height*msg.row_step:
        raise ValueError('PointCloud2 行/点跨度或数据长度错误')
    fields = {f.name:f for f in msg.fields}
    arrays = []
    endian = '>' if msg.is_bigendian else '<'
    for name in ('x', 'y', 'z'):
        f = fields[name]
        size = {7:4, 8:8}.get(f.datatype)
        if size is None or f.count != 1 or f.offset < 0 or f.offset+size > msg.point_step:
            raise ValueError('XYZ 字段必须为有效 FLOAT32/FLOAT64')
        arrays.append(np.ndarray((msg.height,msg.width), dtype=endian+'f'+str(size),
                                 buffer=msg.data, offset=f.offset,
                                 strides=(msg.row_step,msg.point_step)).reshape(-1))
    stride = max(1, int(math.ceil(msg.width*msg.height/max_points)))
    xyz = np.column_stack([a[::stride] for a in arrays]).astype('<f4')
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    return xyz.tobytes(), len(xyz)


class Lz4:
    def __init__(self):
        if os.name == 'nt':
            from lz4.block import compress
            self.windows_compress = compress
            return
        self.lib = ctypes.CDLL(ctypes.util.find_library('lz4') or 'liblz4.so.1')
        self.lib.LZ4_compressBound.argtypes = [ctypes.c_int]
        self.lib.LZ4_compressBound.restype = ctypes.c_int
        self.lib.LZ4_compress_default.argtypes = [ctypes.c_char_p,ctypes.c_void_p,ctypes.c_int,ctypes.c_int]
        self.lib.LZ4_compress_default.restype = ctypes.c_int

    def compress(self, raw):
        if os.name == 'nt':
            return self.windows_compress(raw, store_size=False)
        capacity = self.lib.LZ4_compressBound(len(raw))
        if capacity <= 0:
            raise ValueError('LZ4 输入长度错误')
        dest = ctypes.create_string_buffer(capacity)
        size = self.lib.LZ4_compress_default(raw, dest, len(raw), capacity)
        if size <= 0:
            raise ValueError('LZ4 压缩失败')
        return dest.raw[:size]


def acquire_destination_lock(pc_ip,port):
    """同一上位机目的地只允许一个本工具点云发送器。"""
    lock_path = '/tmp/dianyun-gcs-cloud-{}-{}.lock'.format(pc_ip,port)
    lock_fd = os.open(lock_path,os.O_CREAT | os.O_RDWR | getattr(os,'O_NOFOLLOW',0),0o600)
    lock_file = os.fdopen(lock_fd,'r+')
    try:
        fcntl.flock(lock_file,fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_file.close()
        raise RuntimeError('同一目标的点云发送器仍在退出，稍后重试')
    return lock_file


def packets(raw, compressed, sequence):
    room = 1400-HEADER.size
    count = (len(compressed)+room-1)//room
    if not 0 < count <= 65535:
        raise ValueError('分片数量超出原协议范围')
    for i in range(count):
        data = compressed[i*room:(i+1)*room]
        yield HEADER.pack(MAGIC, sequence, i, count, len(raw), len(compressed), len(data))+data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pc-ip', required=True)
    parser.add_argument('--port', type=int, default=9000)
    parser.add_argument('--topic', default='/factory_fastlio/cloud_world')
    parser.add_argument('--max-hz', type=float, default=2.)
    parser.add_argument('--max-points', type=int, default=4000)
    parser.add_argument('--pose-port', type=int, default=0, help='0保持只传点云；9001为既有POS1接口')
    parser.add_argument('--pose-topic', default='/factory_fastlio/odometry')
    args = parser.parse_args()
    socket.inet_pton(socket.AF_INET, args.pc_ip)
    if not 1 <= args.port <= 65535 or not 0 <= args.pose_port <= 65535 or args.pose_port == args.port or not math.isfinite(args.max_hz) or not 0 < args.max_hz <= 30 or not 1 <= args.max_points <= 1000000:
        parser.error('端口、显示频率或显示点数上限无效')
    # 防止断线重连时旧 SSH 会话尚未退出，导致相同目标出现重复发送器。
    lock_file = acquire_destination_lock(args.pc_ip,args.port)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = rclpy.create_node('dianyun_gcs_cloud_sender_' + uuid.uuid4().hex[:8], enable_rosout=False)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass
    spin_thread = threading.Thread(target=spin, daemon=True)
    lock = threading.Lock()
    latest = [None, 0.]

    def receive(msg):
        with lock:
            latest[:] = [msg, time.monotonic()]

    # 大点云使用Reliable/depth1；回调只替换最新缓存，压缩与UDP发送不阻塞DDS回调。
    sub = node.create_subscription(PointCloud2, args.topic, receive, QoSProfile(depth=1))
    codec = Lz4()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # connect() 为 UDP 固定发送目的地，不监听端口，也不调用 recv()。
    sock.connect((args.pc_ip,args.port))
    pose_sock = None
    pose_sub = None
    pose_sequence = 0
    pose_status = {'pose_phase':'DISABLED','poses_sent':0}
    if args.pose_port:
        pose_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        pose_sock.connect((args.pc_ip,args.pose_port))
        pose_status['pose_phase'] = 'WAIT_ODOMETRY'

        def odometry(msg):
            nonlocal pose_sequence
            try:
                age = node.get_clock().now().nanoseconds * 1e-9-stamp_seconds(msg.header.stamp)
                if not -.05 <= age <= .6:
                    raise ValueError('位姿源时间戳过期')
                with lock:
                    cloud, cloud_received = latest
                if cloud is None or time.monotonic() - cloud_received > .6:
                    raise ValueError('等待同会话点云frame')
                pose_sequence = (pose_sequence+1)&0xffffffff
                packet,values = pose_packet(msg,pose_sequence,cloud.header.frame_id)
                if pose_sock.send(packet) != len(packet):
                    raise OSError('POS1 未完整发送')
                with lock:
                    pose_status.update(pose_phase='STREAMING',poses_sent=pose_status['poses_sent']+1,
                                       pose_source_stamp=stamp_seconds(msg.header.stamp),pose_received=time.monotonic(),
                                       pose_frame=msg.header.frame_id,pose_child_frame=msg.child_frame_id,
                                       position=values[:3],quaternion_xyzw=values[3:])
                    pose_status.pop('pose_error',None)
            except (OSError,ValueError,OverflowError,struct.error) as exc:
                with lock:
                    pose_status.update(pose_phase='INVALID_OR_UNAVAILABLE',pose_error=str(exc))

        pose_sub = node.create_subscription(Odometry, args.pose_topic, odometry, qos_profile_sensor_data)
    sequence = int(time.time()) & 0x7fffffff
    frames = datagrams = errors = 0
    consumed = None
    last_log = 0.
    state = {'phase':'WAIT_CLOUD', 'topic':args.topic, 'destination':[args.pc_ip,args.port],
             'protocol':'PCD1/LZ4', 'display_max_hz':args.max_hz, 'display_max_points':args.max_points,
             'ros_publishers':[], 'node':node.get_fully_qualified_name(),
             'pose_destination':[args.pc_ip,args.pose_port] if args.pose_port else None,
             'pose_topic':args.pose_topic if args.pose_port else None}
    spin_thread.start()
    try:
        while rclpy.ok():
            tick = time.monotonic()
            with lock:
                msg, received = latest
            state['phase'] = 'WAIT_CLOUD' if msg is None else 'STREAMING'
            if msg is not None:
                age = node.get_clock().now().nanoseconds * 1e-9-stamp_seconds(msg.header.stamp)
                state.update(source_age_s=age, frame_id=msg.header.frame_id, source_stamp=stamp_seconds(msg.header.stamp))
                if tick-received > .6 or not -.05 <= age <= .6:
                    state['phase'] = 'CLOUD_STALE'
                elif msg.header.frame_id not in ('camera_init','world'):
                    state['phase'] = 'INVALID_FRAME'
                elif msg is not consumed:
                    consumed = msg
                    try:
                        raw, count = xyz_bytes(msg, args.max_points)
                        if count:
                            compressed = codec.compress(raw)
                            sequence = (sequence+1) & 0xffffffff
                            frame_packets = 0
                            for i, data in enumerate(packets(raw,compressed,sequence)):
                                if sock.send(data) != len(data):
                                    raise OSError('UDP 未完整发送')
                                datagrams += 1
                                frame_packets += 1
                                if i%4 == 3:
                                    time.sleep(.001)
                            frames += 1
                            state.update(points=count, source_points=msg.width*msg.height,
                                         compressed_bytes=len(compressed), packets_per_frame=frame_packets)
                        else:
                            state['phase'] = 'EMPTY_CLOUD'
                    except (OSError,ValueError,KeyError,TypeError) as exc:
                        errors += 1
                        state.update(phase='SEND_ERROR', error=str(exc))
            if tick-last_log >= 2.:
                with lock:
                    state.update(pose_status)
                if args.pose_port and tick-state.get('pose_received',tick) > .6:
                    state['pose_phase'] = 'ODOMETRY_STALE'
                state.update(stamp=time.time(), frames_sent=frames, datagrams_sent=datagrams, errors=errors)
                # SSH 关闭后 stdout 管道断开，主循环退出，避免遗留后台发送器。
                print(json.dumps(state,ensure_ascii=False,allow_nan=False),flush=True)
                last_log = tick
            time.sleep(max(0.,1./args.max_hz-(time.monotonic()-tick)))
    except (BrokenPipeError,KeyboardInterrupt):
        pass
    finally:
        executor.shutdown(timeout_sec=3.)
        spin_thread.join(timeout=3.)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        lock_file.close()
        if pose_sock is not None: pose_sock.close()
        sock.close()


if __name__ == '__main__':
    main()
