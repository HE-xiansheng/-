#!/usr/bin/env python3
"""订阅一种图像并通过 SSH 输出 JPEG；识别模式可管理只监视检测节点。"""
import argparse
import fcntl
import math
import os
import signal
import struct
import subprocess
import sys
import threading
import time

out = sys.stdout.buffer
DISPLAY_MAX_WIDTH = 640
DISPLAY_JPEG_QUALITY = 70
DISPLAY_INTERVAL_S = 0.125

def aruco_source_fresh(msg,now,timeout=1.0):
    """发送时再次检查识别帧源戳；排队或反复重发的旧帧不能保持画面实时。"""
    stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    age=now-stamp
    return math.isfinite(age) and stamp>0 and -.1<=age<=timeout

class LatestFrameSender:
    """接收与发送分离：网络卡住时仅保留最新待发送帧。"""
    def __init__(self, output, interval=DISPLAY_INTERVAL_S, shutdown=lambda reason: None):
        self.shutdown = shutdown
        self.output = output
        self.interval = interval
        self.condition = threading.Condition()
        self.pending = None
        self.stopping = False
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def submit(self, get_jpeg):
        with self.condition:
            if not self.stopping:
                self.pending = get_jpeg
                self.condition.notify()

    def run(self):
        last = 0.0
        while True:
            with self.condition:
                while not self.stopping and self.pending is None:
                    self.condition.wait()
                if self.stopping:
                    return
                delay = last + self.interval - time.monotonic()
                if delay > 0:
                    self.condition.wait(timeout=delay)
                    continue
                get_jpeg, self.pending = self.pending, None
            jpeg = get_jpeg()
            if not jpeg or len(jpeg) > 8 * 1024 * 1024:
                continue
            last = time.monotonic()
            try:
                self.output.write(b'VID1' + struct.pack('>I', len(jpeg)) + jpeg)
                self.output.flush()
            except (BrokenPipeError, ConnectionResetError):
                self.shutdown('视频连接已关闭')
                return

    def stop(self):
        with self.condition:
            self.stopping = True
            self.pending = None
            self.condition.notify_all()
        self.thread.join(timeout=0.5)

sender = None

def send_frame(get_jpeg):
    if sender is not None:
        sender.submit(get_jpeg)

# 子进程独立会话避免与 SSH 同时收到 SIGHUP；父进程异常退出时由 Linux 内核发
# SIGINT，使 ROS 节点退出。脚本内容固定，工作区及 ROS 参数均以独立 argv 传入。
DETECTOR_EXEC = r'''
import ctypes, os, signal, sys
expected = int(sys.argv[1])
if os.getppid() != expected:
    sys.exit(1)
libc = ctypes.CDLL(None, use_errno=True)
if libc.prctl(1, signal.SIGINT, 0, 0, 0) != 0:
    raise OSError(ctypes.get_errno(), 'PR_SET_PDEATHSIG failed')
if os.getppid() != expected:
    sys.exit(1)
os.execv(sys.argv[2], sys.argv[2:])
'''

class PersonMonitor:
    """仅管理本桥接启动的检测节点；其他画面发布者只订阅、不停止。"""
    def __init__(self, node, shutdown, stopping):
        self.node = node
        self.shutdown = shutdown
        self.stopping = stopping
        self.stop_event = threading.Event()
        self.thread = None
        self.process = None
        self.lock_file = None
        self.failed = False

    def start(self):
        self.thread = threading.Thread(target=self.run, name='gcs-person-monitor', daemon=True)
        self.thread.start()

    def acquire_lock(self):
        path = '/tmp/gcs-person-monitor-{}-{}.lock'.format(os.getuid(), int(os.environ.get('ROS_DOMAIN_ID', '0')))
        flags = os.O_CREAT | os.O_RDWR | getattr(os, 'O_NOFOLLOW', 0)
        fd = os.open(path, flags, 0o600)
        lock_file = os.fdopen(fd, 'r+')
        try:
            fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lock_file.close()
            return False
        self.lock_file = lock_file
        return True

    def launch(self):
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        prefix = get_package_prefix('yolo_ultra_test')
        binary = os.environ.get('GCS_PERSON_BINARY', os.path.join(prefix, 'lib', 'yolo_ultra_test', 'person_preview'))
        model = os.environ.get('GCS_PERSON_MODEL', os.path.join(get_package_share_directory('yolo_ultra_test'), 'weights', 'yolov11_s_det_int.rknn'))
        if not os.access(binary, os.X_OK) or not os.path.isfile(model):
            raise RuntimeError('机载识别程序或模型不可用')
        ident = str(os.getpid())
        args = [binary, '--ros-args', '-r', '__node:=gcs_person_monitor_' + ident,
                '-p', 'model_file:=' + model]
        if self.stop_event.is_set() or self.stopping():
            return
        self.process = subprocess.Popen([sys.executable, '-c', DETECTOR_EXEC, ident] + args,
                                        stdout=subprocess.DEVNULL, start_new_session=True)
        print('人物识别：已启动仅监视检测节点', file=sys.stderr, flush=True)

    def cleanup(self):
        try:
            if self.process is not None and self.process.poll() is None:
                try:
                    self.process.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    self.process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait(timeout=1.0)
        finally:
            if self.lock_file is not None:
                self.lock_file.close()
                self.lock_file = None

    def run(self):
        try:
            # DDS发现先留出窗口；图查询失败会退出，不冒险重复拉起。
            self.stop_event.wait(2.0)
            while not self.stop_event.is_set() and not self.stopping():
                if self.process is not None:
                    if self.process.poll() is not None:
                        raise RuntimeError('机载识别进程已退出（代码 {}）'.format(self.process.returncode))
                elif self.node.count_publishers('/manual_person_follow/image') == 0 and self.acquire_lock():
                    # 加锁后再次检查，避免两个地面站同时启动检测节点。
                    if self.node.count_publishers('/manual_person_follow/image') == 0:
                        self.launch()
                    elif self.lock_file is not None:
                        self.lock_file.close()
                        self.lock_file = None
                self.stop_event.wait(1.0)
        except Exception as exc:
            self.failed = True
            print('GCS_PERSON_ERROR: 人物识别不可用：{}'.format(exc), file=sys.stderr, flush=True)
            self.shutdown('人物识别节点不可用')
        finally:
            self.cleanup()

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=6.0)

def main():
    parser = argparse.ArgumentParser(description='只读视频帧转发')
    parser.add_argument('--source', choices=('raw', 'person', 'aruco', 'tracking'), default='raw')
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage, Image
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('gcs_video_view_' + str(os.getpid()))
    stopped = threading.Event()
    def shutdown(reason):
        stopped.set()
    parent_pid = os.getppid()
    if parent_pid == 1:
        node.destroy_node()
        rclpy.shutdown()
        return 1
    def watch_parent():
        while not stopped.wait(.5):
            if os.getppid() != parent_pid:
                shutdown('SSH管理进程已退出')
    threading.Thread(target=watch_parent, daemon=True).start()
    for signame in ('SIGHUP', 'SIGINT', 'SIGTERM'):
        signal.signal(getattr(signal, signame), lambda signum, frame: shutdown('退出信号'))
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    import cv2
    import numpy as np
    # 避免显示转码挤占定位和检测使用的 CPU 线程。
    cv2.setNumThreads(1)
    error_at = [0.0]

    def encode_image(image):
        if image is None or image.size == 0:
            raise ValueError('图像解码失败')
        height, width = image.shape[:2]
        if width > DISPLAY_MAX_WIDTH:
            scaled_height = max(1, round(height * DISPLAY_MAX_WIDTH / width))
            image = cv2.resize(image, (DISPLAY_MAX_WIDTH, scaled_height),
                               interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode('.jpg', image,
                                [int(cv2.IMWRITE_JPEG_QUALITY), DISPLAY_JPEG_QUALITY])
        if not ok:
            raise ValueError('JPEG 编码失败')
        return jpeg.tobytes()

    def encode_safely(make_image):
        try:
            return encode_image(make_image())
        except Exception as exc:
            now = time.monotonic()
            if now - error_at[0] >= 2.0:
                # stdout 是二进制帧协议，诊断只能写 stderr。
                print('视频画面编码失败: {}'.format(exc), file=sys.stderr, flush=True)
                error_at[0] = now
            return None

    global sender
    sender = LatestFrameSender(out, shutdown=shutdown)
    monitor = None
    from cv_bridge import CvBridge
    bridge = CvBridge()
    last_stamp = [0]
    def receive(msg):
        receipt = time.monotonic()
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        def fresh():
            return time.monotonic() - receipt <= 1.0 and aruco_source_fresh(
                msg, node.get_clock().now().nanoseconds * 1e-9)
        if stamp <= last_stamp[0] or not fresh():
            return
        last_stamp[0] = stamp
        def encode_latest():
            if not fresh():
                return None
            if args.source == 'raw':
                result = encode_safely(lambda: cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR))
            else:
                result = encode_safely(lambda: bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
            return result if fresh() else None
        send_frame(encode_latest)
    topic = {'raw':'/camera/color/image_raw/compressed', 'aruco':'/aruco_imgresult',
             'tracking':'/yoloUltra_imgresult', 'person':'/manual_person_follow/image'}[args.source]
    node.create_subscription(CompressedImage if args.source == 'raw' else Image, topic, receive, qos)
    if args.source == 'person':
        monitor = PersonMonitor(node, shutdown, stopped.is_set)
        monitor.start()
    try:
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        stopped.set()
        if monitor is not None:
            monitor.stop()
        sender.stop()
        sender = None
        node.destroy_node()
        rclpy.shutdown()
    return 1 if monitor is not None and monitor.failed else 0

if __name__ == '__main__':
    sys.exit(main())
