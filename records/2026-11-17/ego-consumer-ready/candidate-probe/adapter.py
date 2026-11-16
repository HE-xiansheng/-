#!/usr/bin/env python3
"""ROS2 EGO -> MAVROS PVA adapter. Pilot owns mode and arming; no service writes."""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
import argparse
from collections import deque
import fcntl
import json
import math
from pathlib import Path
import signal
import sys
import time
import numpy as np
from execution import Alignment, Trajectory, Session, rotation


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def atomic(path, data):
    temporary = path.with_suffix('.next')
    temporary.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False))
    os.replace(temporary, path)


def pose(msg):
    value = msg.pose.pose if hasattr(msg.pose, 'pose') else msg.pose
    p = np.array([value.position.x, value.position.y, value.position.z])
    q = np.array([value.orientation.x, value.orientation.y, value.orientation.z, value.orientation.w])
    if not np.isfinite(p).all():
        raise ValueError('nonfinite position')
    matrix = rotation(q)
    return p, q, math.atan2(matrix[1, 0], matrix[0, 0])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    flags = parser.add_mutually_exclusive_group()
    flags.add_argument('--publish', action='store_true')
    flags.add_argument('--check', action='store_true')
    parser.add_argument('--isolated-test', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--run', type=Path, required=True)
    import rclpy
    from rclpy.utilities import remove_ros_args
    # argparse只处理本工具选项，标准ROS2重映射由rclpy.init读取原始argv。
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    if args.isolated_test and (os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        parser.error('isolated test requires localhost DDS domain 87')
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from mavros_msgs.msg import State, ExtendedState, PositionTarget
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String, Bool
    from traj_utils.msg import PolyTraj

    args.run.mkdir(parents=True, exist_ok=True)
    if args.publish:
        singleton = (args.run.parent / 'execution-publisher.lock').open('a')
        fcntl.flock(singleton, fcntl.LOCK_EX | fcntl.LOCK_NB)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('ego_planner_px4_execution')
    consumer_node = node.declare_parameter('command_consumer_node', '/mavros/setpoint_raw').value
    if not isinstance(consumer_node, str) or not consumer_node.startswith('/') or consumer_node.endswith('/'):
        raise ValueError('command_consumer_node must be an absolute ROS node name')
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cache, polys = {}, deque(maxlen=10)
    planner_session = None
    session = Session()
    alignment = active = activekey = frozen_yaw = anchor = None
    execution_mode = None
    stopped = False
    lastpacket = {}
    lastlog = 0.
    lastloop = time.monotonic()
    check_deadline = lastloop + 6.
    check_exit = 1
    # Limits are transport freshness thresholds, not extra planner dynamics limits.
    topics = {
        'state': ('/mavros/state', State, 2.),
        'landed': ('/mavros/extended_state', ExtendedState, 2.),
        'fcpose': ('/mavros/local_position/pose', PoseStamped, .4),
        'lio': ('/factory_fastlio/odometry', Odometry, .4),
        'cloud': ('/factory_fastlio/cloud_world', PointCloud2, .5),
        'status': ('/ego_planner/manual_preview/status', String, .6),
    }

    def receive(msg, key):
        nonlocal planner_session, active, activekey
        receipt = time.monotonic()
        if key == 'status':
            try:
                value = json.loads(msg.data)
                incoming = value['session_id']
                if not isinstance(value, dict) or not isinstance(incoming, str) or not incoming:
                    raise ValueError('missing planner session')
                if incoming != planner_session:
                    polys.clear()
                    active = activekey = None
                    planner_session = incoming
            except (ValueError, TypeError, KeyError):
                cache.pop(key, None)
                return
        cache[key] = (msg, receipt)

    for key, (topic, typ, _) in topics.items():
        node.create_subscription(typ, topic, lambda m, k=key: receive(m, k),
                                 retained if key == 'status' else qos_profile_sensor_data)
    node.create_subscription(PolyTraj, '/ego_planner/flight_execution/trajectory',
                             lambda m: polys.append((m, time.monotonic(), planner_session)), 10)
    modepub = node.create_publisher(Bool, '/ego_planner/flight_execution', retained) if args.publish else None
    statuspub = node.create_publisher(String, '/ego_planner/px4_execution/' +
                                      ('status' if args.publish else 'check_status'), retained)
    pub = node.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 1) if args.publish else None
    from rcl_interfaces.srv import GetParameters
    consumer_client = node.create_client(GetParameters, consumer_node + '/get_parameters') if pub is not None else None
    consumer_probe = None
    consumer_ready = pub is None
    next_consumer_check = 0.

    def request_stop(*_):
        nonlocal stopped
        stopped = True

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, request_stop)

    def tick():
        nonlocal lastloop, execution_mode, alignment, active, activekey, frozen_yaw
        nonlocal lastpacket, lastlog, check_exit, anchor
        nonlocal consumer_ready, next_consumer_check, consumer_probe
        now = time.monotonic()
        rosnow = node.get_clock().now().nanoseconds * 1e-9
        gap = now - lastloop
        lastloop = now
        errors, ages = [], {}
        # 只在首次输出前等待真实MAVROS读者；记录器等其他订阅者不能解除等待。
        # 一旦开始输出不依赖图缓存反复开关，已有状态/输入时效保护负责运行时停发。
        if not consumer_ready and now >= next_consumer_check:
            next_consumer_check = now + .1
            for endpoint in node.get_subscriptions_info_by_topic(pub.topic_name):
                name = endpoint.node_namespace.rstrip('/') + '/' + endpoint.node_name
                if name == consumer_node and endpoint.topic_type == 'mavros_msgs/msg/PositionTarget' and pub.get_subscription_count() > 0:
                    if consumer_probe is None and consumer_client.service_is_ready():
                        request = GetParameters.Request()
                        request.names = ['thrust_scaling']
                        consumer_probe = consumer_client.call_async(request)
                    break
        # 端点构造早于插件执行器启动；只读参数响应证明插件已经能处理回调。
        if consumer_probe is not None and consumer_probe.done() and not consumer_ready:
            try:
                reply = consumer_probe.result()
                consumer_ready = bool(reply and len(reply.values) == 1 and reply.values[0].type == 3)
            except Exception:
                consumer_ready = False
            if not consumer_ready:
                consumer_probe = None
        if not consumer_ready:
            errors.append('command_consumer:waiting')
        status = {}
        if 'status' in cache:
            try:
                status = json.loads(cache['status'][0].data)
            except (ValueError, TypeError):
                pass
        for key, (_, _, timeout) in topics.items():
            if key not in cache:
                errors.append(key + ':missing')
                continue
            msg, receipt = cache[key]
            stamp = status.get('stamp') if key == 'status' else seconds(msg.header.stamp)
            if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                errors.append(key + ':invalid_stamp')
                continue
            age = rosnow - stamp
            ages[key] = {'receive': now - receipt, 'source': age}
            if stamp <= 0 or not -.05 <= age <= timeout or now - receipt > timeout:
                errors.append(key + ':stale')
        fc = cache.get('state', (State(),))[0]
        landed = cache.get('landed', (ExtendedState(),))[0].landed_state
        if not fc.connected:
            errors.append('fc:disconnected')
        fp = fq = lp = lq = current = None
        try:
            fp, fq, yaw = pose(cache['fcpose'][0])
            lp, lq, _ = pose(cache['lio'][0])
            current = (*fp, yaw)
            if not cache['fcpose'][0].header.frame_id:
                errors.append('fcpose:missing_frame')
            frame = cache['lio'][0].header.frame_id
            if not frame or frame != status.get('goal_frame') or cache['cloud'][0].header.frame_id != frame:
                errors.append('localization:frame_mismatch')
        except (KeyError, ValueError, TypeError):
            errors.append('pose:invalid')
        if gap > .2:
            errors.append('control_loop:late')
        healthy = not errors
        stop = stopped or (args.run / 'stop-request').exists()
        requested = bool(args.publish and healthy and fc.mode == 'OFFBOARD' and fc.armed and landed == 2 and not stop)
        if requested != execution_mode:
            if modepub:
                modepub.publish(Bool(data=requested))
            execution_mode = requested
            polys.clear()
            active = activekey = None
            if requested:
                alignment = None
        if not session.entered and healthy and fc.armed and landed == 2 and fc.mode in ('POSCTL', 'OFFBOARD'):
            # Keep the deployed policy: freeze rotation, update translation from latest measured poses.
            alignment = Alignment(lp, lq, fp, fq)
            frozen_yaw = current[3]
            anchor = fp.copy()
        if requested and alignment is None and healthy:
            alignment = Alignment(lp, lq, fp, fq)
            frozen_yaw = current[3]
            anchor = fp.copy()
        prepared = alignment is not None and current is not None
        reference = frame_offset = None
        native = status.get('planning_mode') == 'upstream_fsm'
        planning_errors = []
        if healthy and prepared and fc.mode == 'OFFBOARD' and requested and native:
            matches = [m for m, _, sid in polys if sid == planner_session and m.traj_id == status.get('trajectory_id')]
            if matches and status.get('phase') in ('ACTIVE', 'WAIT_TARGET', 'EMERGENCY_STOP'):
                msg = matches[-1]
                identity = (planner_session, msg.traj_id, seconds(msg.start_time))
                try:
                    if seconds(msg.start_time) > rosnow + .05:
                        raise ValueError('future trajectory')
                    if identity != activekey:
                        active = Trajectory({'id': msg.traj_id, 'start': seconds(msg.start_time), 'order': msg.order,
                            'duration': list(msg.duration), 'x': list(msg.coef_x), 'y': list(msg.coef_y),
                            'z': list(msg.coef_z)}, alignment.at_pose(lp, fp))
                        activekey = identity
                    frame_offset = active.translation_offset(lp, fp)
                    p, v, a = active.sample(rosnow, position_offset=frame_offset)
                    if not all(np.isfinite(x).all() for x in (p, v, a)):
                        raise ValueError('nonfinite reference')
                    reference = {'p': p.tolist(), 'v': v.tolist(), 'a': a.tolist(), 'yaw': frozen_yaw}
                except (ValueError, TypeError, KeyError, IndexError, OverflowError):
                    planning_errors.append('invalid_trajectory')
                    active = activekey = None
            else:
                planning_errors.append('await_upstream_trajectory')
        if session.entered and fc.mode != 'OFFBOARD':
            alignment = active = activekey = None
        target = session.step(now, rosnow, fc.mode, fc.armed, landed == 2, healthy, prepared,
                              reference is not None, current, reference, stop=stop)
        if session.phase == 'TRACK' and status.get('phase') in ('EMERGENCY_STOP', 'WAIT_TARGET'):
            session.phase = 'UPSTREAM_HOLD'
            session.reason = 'upstream:' + status['phase']
        emitted = False
        if pub is not None and target is not None:
            output = PositionTarget()
            output.header.stamp = node.get_clock().now().to_msg()
            output.header.frame_id = cache['fcpose'][0].header.frame_id
            # MAVROS receives ENU values here and performs the ENU -> NED conversion itself.
            output.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            output.type_mask = PositionTarget.IGNORE_YAW_RATE
            if isinstance(target, dict):
                p, v, a, yaw = target['p'], target['v'], target['a'], target['yaw']
            else:
                p, v, a, yaw = target[:3], [0., 0., 0.], [0., 0., 0.], target[3]
            output.position.x, output.position.y, output.position.z = [float(x) for x in p]
            output.velocity.x, output.velocity.y, output.velocity.z = [float(x) for x in v]
            output.acceleration_or_force.x, output.acceleration_or_force.y, output.acceleration_or_force.z = [float(x) for x in a]
            output.yaw = float(yaw)
            pub.publish(output)
            emitted = True
        if now - lastlog >= .1 or session.phase != lastpacket.get('phase'):
            lastpacket = {'stamp': rosnow, 'monotonic_stamp': now, 'phase': session.phase, 'reason': session.reason,
                'mode': fc.mode, 'armed': fc.armed, 'airborne': landed == 2, 'hard_errors': errors,
                'planning_errors': planning_errors, 'warnings': [], 'competing_publishers': {},
                'publish_requested': args.publish, 'output_enabled': emitted,
                'command_consumer_node': consumer_node, 'command_consumer_ready': consumer_ready,
                'tracking_output_enabled': bool(emitted and session.phase == 'TRACK'),
                'output_kind': ('path_tracking' if session.phase == 'TRACK' else 'upstream_hold' if session.phase == 'UPSTREAM_HOLD'
                    else 'prestream_current_pose' if fc.mode == 'POSCTL' else 'position_hold') if emitted else 'none',
                'cycle_id': session.cycle_id, 'goal_key': [planner_session, status.get('goal_id')],
                'trajectory_id': active.ident if active else None, 'trajectory_start': active.start if active else None,
                'trajectory_clock': 'upstream_native', 'time_scale': 1. if active else None,
                'limits': {'source': 'planner_parameters', 'horizontal_speed': 1., 'vertical_speed': .3,
                           'acceleration': .5, 'acceleration_tolerance': 1., 'acceleration_check': 'upstream_3d_norm'},
                'measured_fc_p': fp.tolist() if fp is not None else None,
                'measured_fc_q_xyzw': fq.tolist() if fq is not None else None,
                'measured_lio_p': lp.tolist() if lp is not None else None,
                'measured_lio_q_xyzw': lq.tolist() if lq is not None else None,
                'paired_fc_p': fp.tolist() if fp is not None else None,
                'trajectory_frame_t': active.frame_t.tolist() if active else None,
                'measured_speed': None, 'replan_recoveries': session.replan_recoveries,
                'alignment_policy': 'latest_pose_translation_frozen_rotation',
                'alignment_R': alignment.R.tolist() if alignment else None,
                'alignment_t': alignment.t.tolist() if alignment else None,
                'live_translation_offset': frame_offset.tolist() if frame_offset is not None else None,
                'reference_fc_enu': target, 'input_ages': ages, 'control_loop_gap': gap,
                'planner_status': status, 'parameters_read_only': {'policy': 'native_planner_parameters_no_additional_gate'}}
            # Preserve diagnostic JSON fields consumed by existing monitoring/recording tools.
            lastpacket.update(
                measured_fc_p=fp.tolist() if fp is not None else None,
                measured_fc_q_xyzw=fq.tolist() if fq is not None else None,
                measured_lio_p=lp.tolist() if lp is not None else None,
                measured_lio_q_xyzw=lq.tolist() if lq is not None else None,
                paired_fc_p=fp.tolist() if fp is not None else None,
                trajectory_frame_t=active.frame_t.tolist() if active else None,
                anchor_fc=anchor.tolist() if anchor is not None else None,
                warnings=[], competing_publishers={}, measured_speed=None,
                replan_recoveries=session.replan_recoveries,
                limits={'source': 'planner_parameters', 'horizontal_speed': 1., 'vertical_speed': .3,
                        'acceleration': .5, 'acceleration_tolerance': 1., 'acceleration_check': 'upstream_3d_norm'},
                note='RC mode switch owns handover; no arming/mode/parameter writes; native planner controls replans and emergency stops.')
            atomic(args.run / 'status.json', lastpacket)
            statuspub.publish(String(data=json.dumps(lastpacket, ensure_ascii=False, allow_nan=False)))
            lastlog = now
        if not args.publish and now >= check_deadline:
            check_exit = int(not healthy)
            return False
        return session.phase != 'DONE'

    try:
        next_tick = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=max(0., min(.02, next_tick - time.monotonic())))
            if time.monotonic() >= next_tick:
                if not tick():
                    break
                next_tick = time.monotonic() + .02
    finally:
        if modepub:
            modepub.publish(Bool(data=False))
        if lastpacket:
            lastpacket.update(output_enabled=False, tracking_output_enabled=False, phase='DONE')
            lastpacket['monotonic_stamp'] = time.monotonic()
            lastpacket['stamp'] = node.get_clock().now().nanoseconds * 1e-9
            atomic(args.run / 'status.json', lastpacket)
            statuspub.publish(String(data=json.dumps(lastpacket, ensure_ascii=False, allow_nan=False)))
            rclpy.spin_once(node, timeout_sec=.02)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if args.publish else check_exit


if __name__ == '__main__':
    raise SystemExit(main())
