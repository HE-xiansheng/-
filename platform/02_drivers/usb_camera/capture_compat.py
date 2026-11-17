#!/usr/bin/env python3
"""Preserve Empty capture service names over the native ROS2 SetBool API."""
import json
import signal
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool


class CaptureCompat(Node):
    def __init__(self):
        super().__init__('capture_compat', namespace='usb_cam')
        self.client = self.create_client(SetBool, 'set_capture', callback_group=ReentrantCallbackGroup())
        self.status = self.create_publisher(String, 'capture_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.start = self.create_service(Empty, 'start_capture', self.callback(True))
        self.stop = self.create_service(Empty, 'stop_capture', self.callback(False))

    def callback(self, enabled):
        def invoke(request, response):
            success = False
            reason = 'native capture service unavailable'
            try:
                if self.client.wait_for_service(timeout_sec=1.0):
                    done = threading.Event()
                    future = self.client.call_async(SetBool.Request(data=enabled))
                    future.add_done_callback(lambda _: done.set())
                    if done.wait(2.0):
                        reply = future.result()
                        success, reason = reply.success, reply.message
                    else:
                        self.client.remove_pending_request(future)
                        future.cancel()
                        reason = 'native capture service timed out'
            except Exception as exc:
                reason = str(exc)
            # Empty cannot encode rejection, so retain a separate result for operators.
            result = {'requested_capture': enabled, 'success': success, 'message': reason,
                'stamp_ns': self.get_clock().now().nanoseconds}
            self.status.publish(String(data=json.dumps(result)))
            if not success:
                self.get_logger().error(reason)
            return response
        return invoke


def main():
    # ros2 launch and a group interrupt can both signal this child. Keep the
    # context alive until callbacks finish, and make repeated stops idempotent.
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CaptureCompat()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        while rclpy.ok() and not stopping.is_set():
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
