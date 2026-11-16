#!/usr/bin/env python3
"""订阅一种图像并通过 SSH 输出 JPEG；识别模式可管理只监视检测节点。"""
import argparse
import errno
import fcntl
import math
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import xmlrpc.client
from urllib.parse import urlsplit
import rospy
from sensor_msgs.msg import CompressedImage, Image

out = sys.stdout.buffer
DISPLAY_MAX_WIDTH = 640
DISPLAY_JPEG_QUALITY = 70
DISPLAY_INTERVAL_S = 0.125

def aruco_source_fresh(msg,now,timeout=1.0):
    """发送时再次检查识别帧源戳；排队或反复重发的旧帧不能保持画面实时。"""
    stamp=msg.header.stamp.to_sec()
    age=now-stamp
    return math.isfinite(age) and stamp>0 and -.1<=age<=timeout

class LatestFrameSender:
    """接收与发送分离：网络卡住时仅保留最新待发送帧。"""
    def __init__(self, output, interval=DISPLAY_INTERVAL_S):
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
                rospy.signal_shutdown('视频连接已关闭')
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

class TimeoutTransport(xmlrpc.client.Transport):
    """限制 ROS 图查询耗时，无法确认已有发布者时不启动重复节点。"""
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.0
        return connection

def person_image_published():
    uri = os.environ.get('ROS_MASTER_URI', 'http://127.0.0.1:11311')
    with xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport()) as master:
        code, message, state = master.getSystemState(rospy.get_name())
        if code != 1:
            raise RuntimeError('无法查询识别画面发布者：{}'.format(message))
        nodes = next((names for topic, names in state[0]
                      if topic == '/manual_person_follow/image'), [])
        for node in nodes:
            code, _, node_uri = master.lookupNode(rospy.get_name(), node)
            if code != 1:
                continue
            address = urlsplit(node_uri)
            if not address.hostname or not address.port:
                raise RuntimeError('无法确认已有识别节点的地址')
            try:
                # 只探测节点端口，不调用节点控制服务。
                with socket.create_connection((address.hostname, address.port), timeout=1.0):
                    return True
            except OSError as exc:
                if exc.errno != errno.ECONNREFUSED:
                    raise RuntimeError('无法确认已有识别节点是否在线：{}'.format(node)) from exc
    return False

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
os.execvp('bash', ['bash', '-c', 'source "$1/devel/setup.bash" || exit 1; shift; exec "$@"',
                   'gcs-person-monitor', sys.argv[2]] + sys.argv[3:])
'''

class PersonMonitor:
    """仅管理本桥接启动的检测节点；其他画面发布者只订阅、不停止。"""
    def __init__(self):
        self.stop_event = threading.Event()
        self.thread = None
        self.process = None
        self.lock_file = None
        self.failed = False

    def start(self):
        self.thread = threading.Thread(target=self.run, name='gcs-person-monitor', daemon=True)
        self.thread.start()

    def acquire_lock(self):
        path = '/tmp/gcs-person-monitor-{}.lock'.format(os.getuid())
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
        workspace = os.environ.get('GCS_PERSON_WORKSPACE',
                                   '/home/orangepi/manual_person_follow_20261027_Y8q9Nn')
        setup = os.path.join(workspace, 'devel', 'setup.bash')
        binary = os.path.join(workspace, 'devel', 'lib', 'manual_follow_yolo',
                              'manual_follow_yolo_node')
        model = os.path.join(workspace, 'src', 'manual_follow_yolo', 'weights',
                             'yolov11_s_det_int.rknn')
        if not os.path.isfile(setup) or not os.access(binary, os.X_OK) or not os.path.isfile(model):
            raise RuntimeError('机载识别程序或模型不可用')
        ident = str(os.getpid())
        namespace = '/gcs_person_monitor_' + ident
        args = [binary, '__name:=gcs_person_monitor_' + ident, '_model_file:=' + model,
                '/manual_person_follow/track:=' + namespace + '/track',
                '/manual_person_follow/target:=' + namespace + '/target']
        if self.stop_event.is_set() or rospy.is_shutdown():
            return
        self.process = subprocess.Popen([sys.executable, '-c', DETECTOR_EXEC, ident, workspace] + args,
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
            while not self.stop_event.is_set() and not rospy.is_shutdown():
                if self.process is not None:
                    if self.process.poll() is not None:
                        raise RuntimeError('机载识别进程已退出（代码 {}）'.format(self.process.returncode))
                elif not person_image_published() and self.acquire_lock():
                    # 加锁后再次检查，避免两个地面站同时启动检测节点。
                    if not person_image_published():
                        self.launch()
                    elif self.lock_file is not None:
                        self.lock_file.close()
                        self.lock_file = None
                self.stop_event.wait(1.0)
        except Exception as exc:
            self.failed = True
            print('GCS_PERSON_ERROR: 人物识别不可用：{}'.format(exc), file=sys.stderr, flush=True)
            rospy.signal_shutdown('人物识别节点不可用')
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

    rospy.init_node('gcs_video_view', anonymous=True, disable_signals=True)
    # SSH命令使用exec，连接退出后直接父进程消失；即使无视频可写也能回收订阅。
    parent_pid = os.getppid()
    if parent_pid == 1:
        return 1
    def watch_parent():
        while not rospy.is_shutdown():
            if os.getppid() != parent_pid:
                rospy.signal_shutdown('SSH管理进程已退出')
                return
            time.sleep(0.5)
    threading.Thread(target=watch_parent, daemon=True).start()

    # SSH 退出或切换来源时优雅注销 ROS 订阅，避免留下失效节点。
    def stop_on_signal(signum, _frame):
        rospy.signal_shutdown('视频进程收到退出信号 {}'.format(signum))
    for signame in ('SIGHUP', 'SIGINT', 'SIGTERM'):
        if hasattr(signal, signame):
            signal.signal(getattr(signal, signame), stop_on_signal)
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
    sender = LatestFrameSender(out)
    monitor = None
    if args.source == 'raw':
        # 原压缩帧仅在当前显示通道内缩放重编码，不改变机载相机话题。
        def compressed_frame(msg):
            send_frame(lambda: encode_safely(lambda: cv2.imdecode(
                np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)))

        rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage,
                         compressed_frame, queue_size=1, buff_size=8388608)
    else:
        from cv_bridge import CvBridge
        bridge = CvBridge()

        def person_frame(msg):
            def encode_latest():
                if args.source == 'aruco' and not aruco_source_fresh(msg,rospy.Time.now().to_sec()):
                    return None
                return encode_safely(lambda: bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
            send_frame(encode_latest)

        topic = {'aruco':'/aruco_imgresult', 'tracking':'/yoloUltra_imgresult', 'person':'/manual_person_follow/image'}[args.source]
        rospy.Subscriber(topic, Image, person_frame, queue_size=1, buff_size=8388608)
        if args.source == 'person':
            monitor = PersonMonitor()
            monitor.start()
    try:
        rospy.spin()
    finally:
        sender.stop()
        if monitor is not None:
            monitor.stop()
    return 1 if monitor is not None and monitor.failed else 0

if __name__ == '__main__':
    sys.exit(main())
