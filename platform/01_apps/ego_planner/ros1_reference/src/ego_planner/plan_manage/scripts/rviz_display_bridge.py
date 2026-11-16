#!/usr/bin/python3
"""Adapt existing planner visuals for this viewer's private frame tree only."""
import time
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker


def main():
    rospy.init_node('display_bridge')
    base = '/drone_0_ego_planner_node/'
    cloud_interval = 1.0 / float(rospy.get_param('~cloud_hz', 2.0))
    last_cloud = {}
    outputs = []
    inputs = []
    for source, target, kind, depth in [
        ('grid_map/occupancy_inflate', 'occupancy_inflate', PointCloud2, 1),
        ('grid_map/occupancy', 'occupancy', PointCloud2, 1),
        ('optimal_list', 'optimal_list', Marker, 10),
    ]:
        publisher = rospy.Publisher(rospy.get_name() + '/' + target, kind, queue_size=depth)
        outputs.append(publisher)

        def forward(message, output=publisher, key=target, cloud=(kind is PointCloud2)):
            if output.get_num_connections() == 0:
                return
            if message.header.frame_id.lstrip('/') != 'world':
                rospy.logwarn_throttle(10, 'Ignoring visualization outside planner world frame')
                return
            if cloud:
                now = time.monotonic()
                if now - last_cloud.get(key, -cloud_interval) < cloud_interval:
                    return
                last_cloud[key] = now
            # Original planner coordinates are unchanged. The private display
            # origin is identity-relative to world; no flight TF is published.
            message.header.frame_id = 'ego_planner_rviz_origin'
            if message.header.stamp.is_zero():
                message.header.stamp = rospy.Time.now()
            output.publish(message)

        inputs.append(rospy.Subscriber(base + source, kind, forward,
                                       queue_size=depth, buff_size=4 * 1024 * 1024,
                                       tcp_nodelay=True))
    rospy.spin()


if __name__ == '__main__':
    main()
