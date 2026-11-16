#!/usr/bin/env python3
"""Foreground vision session: reuse producers and stop only owned children."""
import argparse
import fcntl
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Session:
    def __init__(self, node, timeout):
        self.node = node
        self.timeout = timeout
        self.children = []

    def start(self, command):
        child = subprocess.Popen(command, start_new_session=True)
        self.children.append(child)
        print('Started owned process {}'.format(child.pid), flush=True)

    def check_children(self):
        for child in self.children:
            if child.poll() is not None:
                raise RuntimeError('Owned process {} exited ({})'.format(child.pid, child.returncode))

    def discover(self):
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.check_children()

    def has_publisher(self, topic):
        return bool(self.node.get_publishers_info_by_topic(topic))

    def wait_image(self, topic):
        ready = []
        def receive(msg):
            stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
            if stamp > 0 and -0.5 <= age <= 2.0 and msg.width > 0 and msg.height > 0 and msg.data:
                ready.append((msg.width, msg.height, msg.encoding))
        sub = self.node.create_subscription(Image, topic, receive, qos_profile_sensor_data)
        try:
            end = time.monotonic() + self.timeout
            while not ready and time.monotonic() < end:
                self.check_children()
                rclpy.spin_once(self.node, timeout_sec=0.05)
            if not ready:
                raise RuntimeError('No fresh image on {} within {}s; existing producers are left running'.format(topic, self.timeout))
            print('{}: {}x{} {}'.format(topic, *ready[-1]), flush=True)
        finally:
            self.node.destroy_subscription(sub)

    def close(self):
        # Each owned child has its own group, including ros2 run / viewer children.
        for sig, timeout in [(signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 1)]:
            for child in self.children:
                try:
                    os.killpg(child.pid, sig)
                except ProcessLookupError:
                    pass
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                alive = False
                for child in self.children:
                    child.poll()
                    try:
                        os.killpg(child.pid, 0)
                        alive = True
                    except ProcessLookupError:
                        pass
                if not alive:
                    break
                time.sleep(0.05)
        for child in self.children:
            child.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-view', action='store_true')
    parser.add_argument('--rtsp-url', default='rtsp://192.168.144.108')
    parser.add_argument('--timeout', type=float, default=15.0)
    # Preserve the previous ROS2 launch argument spelling as well.
    argv = []
    for value in sys.argv[1:]:
        if value.startswith('rtsp_url:='):
            argv.extend(['--rtsp-url', value.split(':=', 1)[1]])
        else:
            argv.append(value)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    lock_dir = Path('/tmp') / ('aruco-vision-' + str(os.getuid()))
    lock_dir.mkdir(mode=0o700, exist_ok=True)
    lock = (lock_dir / ('domain-' + os.environ.get('ROS_DOMAIN_ID', '0') + '.lock')).open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('Another foreground ArUco vision launcher owns this DDS domain', file=sys.stderr)
        return 1
    rclpy.init(args=[])
    node = rclpy.create_node('aruco_vision_launcher_' + str(os.getpid()))
    session = Session(node, args.timeout)
    def stop(signum, _):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        session.discover()
        if not session.has_publisher('/rtsp_cam/image_raw'):
            session.start(['ros2', 'run', 'rtsp_test', 'rtsp_hw_pub_ultra', '--ros-args', '-p', 'rtsp_url:=' + args.rtsp_url])
        else:
            print('Reusing RTSP publisher', flush=True)
        session.wait_image('/rtsp_cam/image_raw')
        if not session.has_publisher('/aruco_imgresult'):
            config = Path(get_package_share_directory('aruco_test')) / 'config/aruco.yaml'
            session.start(['ros2', 'run', 'aruco_test', 'Safina_aruco_test', '--ros-args', '--params-file', str(config)])
        else:
            print('Reusing ArUco publisher', flush=True)
        session.wait_image('/aruco_imgresult')
        if os.environ.get('DISPLAY') and not args.no_view:
            session.start(['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/aruco_imgresult'])
        print('Vision is ready. Ctrl+C stops only owned processes. No flight controller is started.', flush=True)
        while session.children:
            session.check_children()
            rclpy.spin_once(node, timeout_sec=0.1)
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(str(exc), file=sys.stderr, flush=True)
        return 1
    finally:
        # Repeated Ctrl+C must not interrupt cleanup and orphan an owned producer.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        session.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()


if __name__ == '__main__':
    sys.exit(main())
