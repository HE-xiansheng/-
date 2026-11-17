#!/usr/bin/env python3
"""Subscribe only on .235 and stream untouched serialized bytes; no publishers."""
import json
import os
import queue
import signal
import sys
import tempfile
import threading
import time
from livox_live_protocol import write_frame


def main(seconds=60):
    binary = sys.stdout.buffer
    sys.stdout = sys.stderr
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())
    with tempfile.TemporaryDirectory(prefix='ros2-live-observer-', dir='/dev/shm') as logs:
        os.environ['ROS_LOG_DIR'] = logs
        import rospy
        rospy.init_node('ros2_live_livox_observer', anonymous=True, disable_signals=True,
                        disable_rosout=True)
        pending = queue.Queue(maxsize=2048)
        counts = {'1': 0, '2': 0}
        sent = {'1': 0, '2': 0}
        dropped = {'1': 0, '2': 0}
        errors = []
        types = {}
        transport = {str(k): {'messages': 0, 'max_queue_age_s': 0., 'max_write_s': 0., 'max_queue_depth': 0} for k in (1, 2)}
        lock = threading.Lock()
        recording = True
        subscribers = []
        begin = time.monotonic()

        def transmit(kind, payload, receipt_ns):
            start = time.monotonic()
            metrics = transport[str(kind)]
            metrics['max_queue_age_s'] = max(metrics['max_queue_age_s'], (time.time_ns()-receipt_ns)/1e9)
            metrics['max_queue_depth'] = max(metrics['max_queue_depth'], pending.qsize())
            write_frame(binary, kind, payload, receipt_ns)
            metrics['max_write_s'] = max(metrics['max_write_s'], time.monotonic()-start)
            metrics['messages'] += 1
            sent[str(kind)] += 1

        def receive(message, kind):
            with lock:
                if not recording:
                    return
                message_type = message._connection_header.get('type')
                expected = {1: 'sensor_msgs/Imu', 2: 'livox_ros_driver2/CustomMsg'}[kind]
                if message_type != expected:
                    errors.append('Unexpected type on '+str(kind)+': '+str(message_type))
                    stop.set()
                    return
                types[str(kind)] = message_type
                counts[str(kind)] += 1
                try:
                    pending.put_nowait((kind, bytes(message._buff), time.time_ns()))
                except queue.Full:
                    dropped[str(kind)] += 1
        try:
            subscribers = [rospy.Subscriber(topic, rospy.AnyMsg, receive,
                callback_args=kind, queue_size=200, buff_size=16*1024*1024)
                for kind, topic in [(1, '/livox/imu'), (2, '/livox/lidar')]]
            while not stop.is_set() and time.monotonic()-begin < seconds:
                try:
                    kind, payload, receipt_ns = pending.get(timeout=.1)
                except queue.Empty:
                    continue
                transmit(kind, payload, receipt_ns)
            for subscriber in subscribers:
                subscriber.unregister()
            subscribers.clear()
            # A callback already in progress may finish after unregister.
            with lock:
                recording = False
                while not pending.empty():
                    kind, payload, receipt_ns = pending.get_nowait()
                    transmit(kind, payload, receipt_ns)
            result = {'duration_s': time.monotonic()-begin, 'received': counts,
                      'sent': sent, 'dropped': dropped, 'publishers_created': 0,
                      'types': types, 'errors': errors, 'transport': transport, 'flight_commands_sent': False}
            write_frame(binary, 255, json.dumps(result).encode(), time.time_ns())
        finally:
            for subscriber in subscribers:
                subscriber.unregister()
            rospy.signal_shutdown('read-only live sample finished')


if __name__ == '__main__':
    main(float(sys.argv[1]) if len(sys.argv) > 1 else 60)
