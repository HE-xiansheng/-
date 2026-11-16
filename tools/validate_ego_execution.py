#!/usr/bin/env python3
"""Isolated native FSM + execution adapter + synthetic FC, no physical MAVROS node."""
import os
os.environ.update(ROS_DOMAIN_ID='87', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR='/tmp/ego-execution-ros-logs')
import importlib.util
import json
import math
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String
from mavros_msgs.msg import State, ExtendedState, PositionTarget, Mavlink
from traj_utils.msg import PolyTraj

ROOT = Path(os.environ.get('EGO_EXECUTION_BENCH_ROOT', '/home/orangepi/drone_platform/01_apps/ego_planner'))
sys.path.insert(0, str(ROOT/'px4_execution'))
from execution import Alignment, Trajectory, Session


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


def main():
    checks = {}
    rclpy.init()
    # Isolated synthetic command consumer has the real plugin identity; no physical MAVROS.
    node = rclpy.create_node('setpoint_raw', namespace='/mavros')
    node.declare_parameter('thrust_scaling', 1.0)
    transport_fixture = node.create_publisher(Mavlink, '/uas1/mavlink_sink', 10)
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    statuses, planner_statuses, commands, polynomials, modes = [], [], [], [], []
    node.create_subscription(String, '/ego_planner/px4_execution/status', lambda m: statuses.append(json.loads(m.data)), retained)
    node.create_subscription(String, '/ego_planner/manual_preview/status', lambda m: planner_statuses.append(json.loads(m.data)), retained)
    node.create_subscription(PositionTarget, '/mavros/setpoint_raw/local', commands.append, 10)
    node.create_subscription(PolyTraj, '/ego_planner/flight_execution/trajectory', polynomials.append, 10)
    node.create_subscription(Bool, '/ego_planner/flight_execution', lambda m: modes.append(m.data), retained)
    pubs = {key: node.create_publisher(typ, topic, 10) for key, typ, topic in [
        ('state', State, '/mavros/state'), ('landed', ExtendedState, '/mavros/extended_state'),
        ('fcpose', PoseStamped, '/mavros/local_position/pose'),
        ('lio', Odometry, '/factory_fastlio/odometry'),
        ('cloud', PointCloud2, '/factory_fastlio/cloud_world'),
        ('goal', PoseStamped, '/ego_planner/navigation/preview_goal')]}
    mode, armed, landed, connected = 'POSCTL', False, 1, True
    feed_lio = feed_cloud = True
    lio_frame, fc_offset, quaternion_valid = 'camera_init', 0., True
    fc_position = [10., 20., 3.]
    last_feed = 0.
    cloud = PointCloud2()
    cloud.header.frame_id = 'camera_init'
    cloud.height = 1
    cloud.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(('x','y','z'))]
    points = [(1.05, y*.1, z*.1) for y in range(-5,6) for z in range(-5,6)]
    cloud.width, cloud.point_step, cloud.row_step, cloud.is_dense = len(points), 12, len(points)*12, True
    cloud.data = b''.join(struct.pack('<fff', *p) for p in points)

    def pump(seconds, predicate=None):
        nonlocal last_feed
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_feed >= .025:
                last_feed = now
                stamp = node.get_clock().now()
                state = State()
                state.header.stamp = stamp.to_msg()
                state.connected, state.armed, state.mode = connected, armed, mode
                pubs['state'].publish(state)
                ext = ExtendedState()
                ext.header.stamp = stamp.to_msg()
                ext.landed_state = landed
                pubs['landed'].publish(ext)
                fc = PoseStamped()
                fc.header.stamp = (stamp + rclpy.duration.Duration(seconds=fc_offset)).to_msg()
                fc.header.frame_id = 'map'
                fc.pose.position.x, fc.pose.position.y, fc.pose.position.z = fc_position
                if quaternion_valid:
                    fc.pose.orientation.z = fc.pose.orientation.w = math.sqrt(.5)
                else:
                    fc.pose.orientation.x = fc.pose.orientation.y = fc.pose.orientation.z = fc.pose.orientation.w = 0.
                pubs['fcpose'].publish(fc)
                if feed_lio:
                    lio = Odometry()
                    lio.header.stamp, lio.header.frame_id = stamp.to_msg(), lio_frame
                    lio.child_frame_id = 'body'
                    lio.pose.pose.position.x = -2.
                    lio.pose.pose.orientation.w = 1.
                    pubs['lio'].publish(lio)
                if feed_cloud:
                    cloud.header.stamp = stamp.to_msg()
                    pubs['cloud'].publish(cloud)
            rclpy.spin_once(node, timeout_sec=.003)
            if predicate and predicate():
                return True
        return bool(predicate and predicate())

    def phase(value, timeout=5):
        return pump(timeout, lambda: statuses and statuses[-1]['phase'] == value)

    def manager(action):
        task = subprocess.Popen([sys.executable, str(ROOT/'px4_execution/manage.py'), action, '--isolated-test'],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
        if not pump(10, lambda: task.poll() is not None):
            task.kill()
            raise RuntimeError('manager timeout: '+action)
        out, err = task.communicate()
        return task.returncode, out.decode(), err.decode()

    def wait_error(value):
        return pump(4, lambda: statuses and value in statuses[-1]['hard_errors'])

    def no_commands_for(seconds=.2):
        pump(.1)
        before = len(commands)
        pump(seconds)
        return len(commands) == before

    def send_goal(x=-.5):
        goal = PoseStamped()
        goal.header.frame_id = 'camera_init'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.orientation.w = 1.
        goal.pose.position.x = x
        pubs['goal'].publish(goal)

    run = Path(tempfile.mkdtemp(prefix='ego-execution-bench-'))
    env = dict(os.environ, EGO_EXECUTION_RUNTIME_DIR=str(run))
    log = tempfile.TemporaryFile()
    planner = subprocess.Popen([str(ROOT/'workspace/install/plan_manage/lib/plan_manage/planner_node'), '--ros-args',
        '--params-file', str(ROOT/'config/planner.yaml'), '--params-file', str(ROOT/'config/manual_preview.yaml'),
        '-r', 'odom_world:=/factory_fastlio/odometry', '-r', 'grid_map/odom:=/factory_fastlio/odometry',
        '-r', 'grid_map/cloud:=/factory_fastlio/cloud_world'], stdout=log, stderr=subprocess.STDOUT)
    info = None
    try:
        checks['planner_ready'] = pump(8, lambda: planner_statuses and planner_statuses[-1]['phase'] == 'WAIT_GOAL')
        result, output, error = manager('enable')
        checks['manager_starts_adapter'] = result == 0
        if result:
            raise RuntimeError(output+error)
        info = json.loads((run/'active.json').read_text())
        checks['disarmed_waits_no_commands'] = phase('WAIT_FLIGHT') and no_commands_for()
        result, _, _ = manager('enable')
        checks['duplicate_enable_rejected'] = result != 0
        checks['single_publisher_enforced'] = manager('enable')[0] != 0
        result, output, _ = manager('status')
        checks['manager_status_reports_owned_process'] = result == 0 and json.loads(output)['pid'] == info['pid']
        armed = True
        checks['grounded_no_commands'] = phase('WAIT_FLIGHT') and no_commands_for()
        landed = 2
        checks['pilot_posctl_prestream'] = phase('READY') and pump(2, lambda: len(commands) >= 5)
        checks['synthetic_consumer_identity_ready'] = statuses[-1]['command_consumer_ready']
        if commands:
            out = commands[-1]
            checks['prestream_is_current_enu_pose'] = np.allclose([out.position.x,out.position.y,out.position.z],fc_position) and abs(out.yaw-math.pi/2) < 1e-5
            checks['mavros_conversion_contract'] = out.coordinate_frame == PositionTarget.FRAME_LOCAL_NED and out.type_mask == PositionTarget.IGNORE_YAW_RATE and out.header.frame_id == 'map'
        before = len(commands)
        started = time.monotonic()
        pump(.7)
        observed_rate = (len(commands)-before)/(time.monotonic()-started)
        checks['setpoint_rate_sustained'] = 30 < observed_rate < 65
        send_goal()
        checks['preview_goal_accepted'] = pump(4, lambda: planner_statuses and planner_statuses[-1].get('has_goal'))
        mode = 'OFFBOARD'
        checks['pilot_offboard_enters_tracking'] = phase('TRACK', 8)
        checks['mode_flag_drives_native_fsm'] = True in modes and bool(polynomials)
        checks['source_clock_preserved'] = statuses[-1].get('time_scale') == 1. and statuses[-1].get('trajectory_clock') == 'upstream_native'
        # Compare a status reference to an independent polynomial evaluation in the FC ENU frame.
        packet = statuses[-1]
        candidates = [m for m in polynomials if m.traj_id == packet['trajectory_id']]
        if candidates and packet.get('reference_fc_enu'):
            msg = candidates[-1]
            elapsed = min(sum(msg.duration), max(0.,packet['stamp']-stamp_seconds(msg.start_time)))
            i = min(len(msg.duration)-1, int(np.searchsorted(np.cumsum(msg.duration),elapsed,side='right')))
            t = elapsed - sum(msg.duration[:i])
            coeff = np.stack([np.array(c).reshape(-1,6)[i] for c in (msg.coef_x,msg.coef_y,msg.coef_z)])
            R = np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
            translation = np.array([10.,22.,3.])
            expected = [R@np.array([np.polyval(np.polyder(axis,k),t) for axis in coeff]) for k in range(3)]
            expected[0] += translation
            if elapsed >= sum(msg.duration):
                expected[1][:] = expected[2][:] = 0.
            ref = packet['reference_fc_enu']
            checks['pva_rotated_once_into_fc_enu'] = all(np.allclose(ref[k], v, atol=2e-4) for k,v in zip(('p','v','a'),expected))
            checks['yaw_frozen_at_handover'] = abs(ref['yaw']-math.pi/2)<1e-5
        else:
            checks['pva_rotated_once_into_fc_enu'] = False
        feed_lio = False
        checks['stale_odometry_stops_output'] = wait_error('lio:stale') and no_commands_for()
        feed_lio = True
        checks['fresh_odometry_resumes_native_plan'] = phase('TRACK', 8)
        feed_cloud = False
        checks['stale_cloud_stops_output'] = wait_error('cloud:stale') and no_commands_for()
        feed_cloud = True
        checks['fresh_cloud_recovers'] = phase('TRACK', 8)
        fc_offset = 2.
        checks['future_fc_pose_stops_output'] = wait_error('fcpose:stale') and no_commands_for()
        fc_offset = 0.
        checks['valid_fc_stamp_recovers'] = phase('TRACK', 8)
        quaternion_valid = False
        checks['invalid_quaternion_stops_output'] = wait_error('pose:invalid') and no_commands_for()
        quaternion_valid = True
        checks['valid_orientation_recovers'] = phase('TRACK', 8)
        lio_frame = 'wrong_frame'
        checks['frame_mismatch_stops_output'] = wait_error('localization:frame_mismatch') and no_commands_for()
        lio_frame = 'camera_init'
        checks['valid_frame_recovers'] = phase('TRACK', 8)
        planner.send_signal(signal.SIGSTOP)
        checks['stale_planner_stops_output'] = wait_error('status:stale') and no_commands_for()
        planner.send_signal(signal.SIGCONT)
        checks['fresh_planner_recovers'] = phase('TRACK', 8)
        connected = False
        checks['disconnect_stops_output'] = wait_error('fc:disconnected') and no_commands_for()
        connected = True
        checks['connection_recovers'] = phase('TRACK', 8)
        mode = 'POSCTL'
        checks['pilot_takeover_stops_tracking'] = phase('MANUAL_WAIT') and not statuses[-1]['tracking_output_enabled']
        pump(.3)
        checks['manual_prestream_has_zero_velocity'] = commands and all(abs(v)<1e-7 for v in (commands[-1].velocity.x,commands[-1].velocity.y,commands[-1].velocity.z))
        mode = 'OFFBOARD'
        checks['second_pilot_cycle_tracks'] = phase('TRACK',8) and statuses[-1]['cycle_id'] == 2
        result, _, _ = manager('stop')
        checks['stop_is_controlled_hold'] = result == 0 and phase('STOP_HOLD') and statuses[-1]['output_enabled'] and not statuses[-1]['tracking_output_enabled']
        if commands:
            out = commands[-1]
            checks['stop_hold_is_measured_position'] = np.allclose([out.position.x,out.position.y,out.position.z],fc_position)
        mode = 'POSCTL'
        checks['pilot_takeover_completes_stop'] = phase('DONE')
        checks['no_output_after_adapter_exit'] = no_commands_for(.4)
        result, output, _ = manager('status')
        checks['manager_detects_exit'] = result == 0 and not json.loads(output)['running']
        # Read-only check must create no command publisher and exit with valid inputs.
        result, _, error = manager('check')
        checks['check_mode_readonly_success'] = result == 0
        checks['check_mode_sends_no_commands'] = no_commands_for()
        checks['no_flight_services_called'] = not any(name in ('/mavros/cmd/arming', '/mavros/set_mode') for name,_ in node.get_service_names_and_types())
        # Exact piece boundary, endpoint hold, translation and malformed-coefficient checks.
        alignment = Alignment([0,0,0],[0,0,0,1],[1,2,3],[0,0,math.sqrt(.5),math.sqrt(.5)])
        data = {'id':1,'start':100.,'order':5,'duration':[1.,1.],
                'x':[0,0,0,0,1,0,0,0,0,0,1,1], 'y':[0.]*12,'z':[0.]*12}
        traj = Trajectory(data,alignment)
        p,v,a = traj.sample(101.)
        checks['piece_boundary_preserves_pva'] = np.allclose(p,[1,3,3]) and np.allclose(v,[0,1,0]) and np.allclose(a,0)
        p,v,a = traj.sample(103.)
        checks['completed_trajectory_holds_endpoint'] = np.allclose(p,[1,4,3]) and np.allclose(v,0) and np.allclose(a,0)
        try:
            Trajectory(dict(data,x=[float('nan')]*12),alignment)
            checks['invalid_coefficients_rejected'] = False
        except ValueError:
            checks['invalid_coefficients_rejected'] = True
    except Exception as exc:
        checks['exception_free'] = False
        print('BENCH_ERROR',repr(exc),flush=True)
    finally:
        mode = 'POSCTL'
        if info:
            try:
                (Path(info['run'])/'stop-request').touch()
                pump(.6)
                os.kill(info['pid'], signal.SIGTERM)
                pump(.3)
            except (OSError, ProcessLookupError):
                pass
        if planner.poll() is None:
            planner.send_signal(signal.SIGCONT)
            planner.send_signal(signal.SIGINT)
        try:
            planner.wait(timeout=6)
        except subprocess.TimeoutExpired:
            planner.kill(); planner.wait()
        checks['planner_clean_shutdown'] = planner.returncode == 0
        if not all(checks.values()):
            log.seek(0)
            print('PLANNER_LOG',log.read().decode(errors='replace')[-8000:])
            print('ADAPTER_LAST',json.dumps(statuses[-3:]))
        log.close()
        shutil.rmtree(run)
        node.destroy_node()
        rclpy.shutdown()
    result = {'passed': bool(checks) and all(checks.values()), 'checks':checks,
              'synthetic_fc_commands_received':len(commands),'setpoint_rate_hz':locals().get('observed_rate'),
              'native_execution_trajectories':len(polynomials),'physical_flight_commands_sent':False,
              'isolation':{'ROS_DOMAIN_ID':87,'ROS_LOCALHOST_ONLY':1}}
    print('EXECUTION_RESULT '+json.dumps(result,sort_keys=True))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
