#!/usr/bin/env python3
"""Real ROS2 FSM process with synthetic inputs, isolated DDS and no FC interfaces."""
import json
import math
import os
import signal
import struct
import subprocess
import tempfile
import time

os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_LOG_DIR'] = '/tmp/ego-fsm-ros-logs'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty, String
from traj_utils.msg import PolyTraj


def main():
    rclpy.init()
    node = rclpy.create_node('ego_fsm_bench')
    root = '/home/orangepi/drone_platform/01_apps/ego_planner'
    checks = {}
    statuses, paths, executable = [], [], []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/ego_planner/manual_preview/status', lambda m: statuses.append(json.loads(m.data)), qos)
    node.create_subscription(PolyTraj, '/ego_planner/manual_preview/trajectory', paths.append, qos)
    node.create_subscription(PolyTraj, '/ego_planner/flight_execution/trajectory', executable.append, 10)
    odom_pub = node.create_publisher(Odometry, '/bench/odom', 10)
    cloud_pub = node.create_publisher(PointCloud2, '/bench/cloud', 10)
    goal_pub = node.create_publisher(PoseStamped, '/ego_planner/navigation/preview_goal', 10)
    stop_pub = node.create_publisher(Empty, '/mandatory_stop', 10)
    mode_pub = node.create_publisher(Bool, '/ego_planner/flight_execution', qos)
    position, speed, stamp_offset = [-2., 0., 0.], 0., 0.
    feed_odom = feed_cloud = False
    odom_frame = 'camera_init'
    cloud = PointCloud2()
    cloud.header.frame_id = 'camera_init'
    cloud.height = 1
    cloud.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(('x', 'y', 'z'))]
    points = [(1.05, y*.1, z*.1) for y in range(-5, 6) for z in range(-5, 6)]
    cloud.width = len(points)
    cloud.point_step = 12
    cloud.row_step = 12*len(points)
    cloud.is_dense = True
    cloud.data = b''.join(struct.pack('<fff', *p) for p in points)
    last_feed = 0.

    def pump(seconds, predicate=None):
        nonlocal last_feed
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last_feed >= .05:
                last_feed = now
                stamp = node.get_clock().now()
                if feed_odom:
                    odom = Odometry()
                    odom.header.frame_id = odom_frame
                    odom.header.stamp = (stamp + rclpy.duration.Duration(seconds=stamp_offset)).to_msg()
                    odom.pose.pose.orientation.w = 1.
                    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
                    odom.twist.twist.linear.x = speed
                    odom_pub.publish(odom)
                if feed_cloud:
                    cloud.header.stamp = stamp.to_msg()
                    cloud_pub.publish(cloud)
            rclpy.spin_once(node, timeout_sec=.005)
            if predicate and predicate():
                return True
        return bool(predicate and predicate())

    def phase(name, timeout=4):
        return pump(timeout, lambda: statuses and statuses[-1]['phase'] == name)

    def goal(x=-.5, frame='camera_init'):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.orientation.w = 1.
        msg.pose.position.x = x
        goal_pub.publish(msg)

    log = tempfile.TemporaryFile()
    proc = subprocess.Popen([root+'/workspace/install/plan_manage/lib/plan_manage/planner_node', '--ros-args',
        '--params-file', root+'/config/planner.yaml', '--params-file', root+'/config/manual_preview.yaml',
        '-p', 'fsm.manual_arrival_hold:=0.3', '-r', 'odom_world:=/bench/odom',
        '-r', 'grid_map/odom:=/bench/odom', '-r', 'grid_map/cloud:=/bench/cloud'], stdout=log, stderr=subprocess.STDOUT)
    try:
        checks['waits_for_inputs'] = phase('WAIT_INPUT', 8)
        feed_odom = feed_cloud = True
        checks['waits_for_goal'] = phase('WAIT_GOAL')
        goal()
        checks['goal_plans_preview'] = phase('ACTIVE', 8) and bool(paths)
        count = statuses[-1]['goal_id'] if statuses else -1
        goal(2., 'map')
        checks['wrong_frame_goal_rejected'] = pump(2, lambda: statuses and statuses[-1]['navigation_message'] == 'GOAL_FRAME_MISMATCH') and statuses[-1]['goal_id'] == count
        goal(float('nan'))
        checks['nonfinite_goal_rejected'] = pump(2, lambda: statuses and statuses[-1]['navigation_message'] == 'INVALID_GOAL') and statuses[-1]['goal_id'] == count
        checks['status_frame_correct'] = statuses[-1]['goal_frame'] == 'camera_init'
        checks['direct_goal_no_confirmation'] = not statuses[-1]['confirmation_required'] and statuses[-1]['goal_confirmed']
        checks['preview_coefficients_finite'] = bool(paths) and all(math.isfinite(v) for p in paths for v in list(p.coef_x)+list(p.coef_y)+list(p.coef_z)+list(p.duration))
        checks['preview_unique_ids'] = len({p.traj_id for p in paths}) == len(paths)
        position = [-.5, 0., 0.]
        speed = .5
        checks['moving_near_goal_not_arrived'] = phase('WAIT_STOP') and not statuses[-1]['measured_arrived']
        speed = 0.
        checks['arrival_requires_measured_hold'] = phase('SETTLING') and phase('ARRIVED') and statuses[-1]['measured_arrived']
        feed_cloud = False
        checks['stale_cloud_clears_arrival'] = phase('STALE_INPUT', 2) and not statuses[-1]['measured_arrived']
        feed_cloud = True
        position = [-2., 0., 0.]
        checks['fresh_inputs_recover'] = phase('ACTIVE')
        feed_odom = False
        checks['stale_odometry_blocks_planning'] = phase('STALE_INPUT', 2)
        before = len(paths)
        pump(.35)
        checks['no_new_preview_on_stale_data'] = len(paths) == before
        feed_odom = True
        stamp_offset = 2.
        checks['future_odometry_rejected'] = phase('STALE_INPUT')
        stamp_offset = 0.
        checks['fresh_odometry_recovers'] = phase('ACTIVE')
        stop_pub.publish(Empty())
        checks['stop_acknowledged'] = phase('STOPPED')
        before = len(paths)
        pump(.8)
        checks['stop_prevents_new_preview'] = len(paths) == before
        goal()
        checks['new_goal_after_stop'] = phase('ACTIVE')
        checks['preview_sends_no_execution_trajectory'] = len(executable) == 0
        cli = subprocess.Popen(['python3', root+'/map_navigation.py', 'preview', '-0.7', '0', '0'],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        checks['map_cli_acknowledges_goal'] = pump(10, lambda: cli.poll() is not None) and cli.returncode == 0
        out, err = cli.communicate(timeout=2)
        if cli.returncode:
            print('CLI_ERROR', err.decode(errors='replace'))
        else:
            checks['map_cli_uses_status_frame'] = json.loads(out)['goal_frame'] == 'camera_init'
        before = len(executable)
        mode_pub.publish(Bool(data=True))
        checks['isolated_execution_mode_received'] = pump(4, lambda: statuses and statuses[-1]['planning_mode'] == 'upstream_fsm')
        goal()
        checks['isolated_native_fsm_produces_trajectory'] = pump(8, lambda: len(executable) > before)
        feed_cloud = False
        checks['execution_stale_cloud_detected'] = phase('STALE_INPUT', 2)
        before = len(executable)
        pump(.35)
        checks['execution_stale_blocks_new_trajectories'] = len(executable) == before
        position = [-.5, 0., 0.]
        pump(.5)
        checks['execution_stale_cloud_cannot_report_arrived'] = statuses and not statuses[-1]['measured_arrived']
        feed_cloud = True
        position = [-2., 0., 0.]
        mode_pub.publish(Bool(data=False))
        checks['return_to_preview'] = pump(4, lambda: statuses and statuses[-1]['planning_mode'] == 'measured_preview')
        checks['planner_still_running'] = proc.poll() is None
    finally:
        proc.send_signal(signal.SIGINT) if proc.poll() is None else None
        try:
            proc.wait(timeout=6)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        checks['clean_shutdown'] = proc.returncode == 0
        if not all(checks.values()):
            log.seek(0)
            print(log.read().decode(errors='replace')[-10000:])
            print('LAST_STATUSES', json.dumps(statuses[-6:]))
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    result = {'passed': all(checks.values()), 'checks': checks, 'preview_messages': len(paths),
              'isolated_execution_trajectories': len(executable), 'flight_commands_sent': False,
              'isolation': {'ROS_DOMAIN_ID': 87, 'ROS_LOCALHOST_ONLY': 1}}
    print('FSM_RESULT '+json.dumps(result, sort_keys=True))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
