#!/usr/bin/env python3
"""Read ROS1 Livox inputs and stream compressed serialized messages to stdout."""
import base64
import gzip
import io
import json
import os
import tempfile
import threading
import time
import sys


with tempfile.TemporaryDirectory(prefix='livox-observer-', dir='/dev/shm') as temporary:
    os.environ['ROS_LOG_DIR'] = temporary
    import rospy
    from livox_ros_driver2.msg import CustomMsg
    from sensor_msgs.msg import Imu

    rospy.init_node('ros2_migration_input_observer', anonymous=True, disable_signals=True)
    lock = threading.Lock()
    counts = {}
    recording = True
    start = time.monotonic()
    with gzip.GzipFile(fileobj=sys.stdout.buffer, mode='wb') as output:
        def callback(message, topic):
            buffer = io.BytesIO()
            message.serialize(buffer)
            entry = {'topic': topic, 'receipt_offset': time.monotonic() - start,
                     'serialized': base64.b64encode(buffer.getvalue()).decode('ascii')}
            with lock:
                if recording:
                    output.write((json.dumps(entry) + '\n').encode())
                    counts[topic] = counts.get(topic, 0) + 1
        subscribers = [rospy.Subscriber('/livox/lidar', CustomMsg, callback, callback_args='/livox/lidar', queue_size=10),
                       rospy.Subscriber('/livox/imu', Imu, callback, callback_args='/livox/imu', queue_size=200)]
        rospy.sleep(10)
        for subscriber in subscribers:
            subscriber.unregister()
        with lock:
            recording = False
    rospy.signal_shutdown('Read-only sample finished')
    print(json.dumps({'received': counts, 'flight_commands_sent': False}), file=sys.stderr)
