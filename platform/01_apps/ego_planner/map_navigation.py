#!/usr/bin/env python3
"""ROS2 direct map goal client; coordinates use the planner status goal_frame."""
import argparse
import json
import math
import time


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('preview').add_argument('xyz', nargs=3, type=float)
    sub.add_parser('status')
    args = parser.parse_args()
    if args.action == 'preview' and not all(math.isfinite(v) for v in args.xyz):
        parser.error('目标坐标必须为有限数值')
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    rclpy.init()
    node = rclpy.create_node('ego_planner_navigation_client')
    updates = []

    def receive(msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                updates.append(value)
        except (ValueError, TypeError):
            pass

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    reader = node.create_subscription(String, '/ego_planner/manual_preview/status', receive, qos)

    def wait(predicate, timeout=8):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            while updates:
                value = updates.pop(0)
                if predicate(value):
                    return value
        raise RuntimeError('未收到目标更新回执，请查看机载规划器状态')

    try:
        status = wait(lambda s: True)
        if args.action == 'status':
            print(json.dumps(status, ensure_ascii=False))
            return
        frame = status.get('goal_frame')
        if not isinstance(frame, str) or not frame:
            raise RuntimeError('规划器未声明目标坐标系')
        publisher = node.create_publisher(PoseStamped, '/ego_planner/navigation/preview_goal', 1)
        end = time.monotonic() + 3
        while publisher.get_subscription_count() == 0 and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError('原生规划器目标入口未运行')
        msg = PoseStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.pose.orientation.w = 1.
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = args.xyz
        publisher.publish(msg)
        result = wait(lambda s: s.get('session_id') == status.get('session_id')
                      and s.get('goal_id', 0) > status.get('goal_id', 0)
                      and s.get('has_goal') and s.get('goal') == args.xyz)
        print(json.dumps(result, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
