#!/usr/bin/env python3
"""Exercise deployed shell entrypoints and owned process lifecycle on isolated DDS."""
import os
os.environ.update(ROS_DOMAIN_ID='87', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR='/tmp/ego-lifecycle-ros-logs')
import json
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node('ego_lifecycle_bench')
    root = Path('/home/orangepi/drone_platform/01_apps/ego_planner')
    run = Path(tempfile.mkdtemp(prefix='ego-lifecycle-bench-'))
    env = dict(os.environ, EGO_PREVIEW_RUNTIME_DIR=str(run/'preview'), EGO_EXECUTION_RUNTIME_DIR=str(run/'execution'),
               EGO_EXECUTION_ISOLATED_TEST='1')
    checks, statuses, commands = {}, [], []
    node.create_subscription(String, '/ego_planner/manual_preview/status', lambda m: statuses.append(json.loads(m.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(PositionTarget, '/mavros/setpoint_raw/local', commands.append, 10)
    statepub = node.create_publisher(State, '/mavros/state', 10)
    extpub = node.create_publisher(ExtendedState, '/mavros/extended_state', 10)
    posepub = node.create_publisher(PoseStamped, '/mavros/local_position/pose', 10)
    odompub = node.create_publisher(Odometry, '/factory_fastlio/odometry', 10)
    cloudpub = node.create_publisher(PointCloud2, '/factory_fastlio/cloud_world', 10)
    connected, armed, airborne = True, False, False
    cloud = PointCloud2()
    cloud.header.frame_id = 'camera_init'
    cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, 1, 12, 12
    cloud.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1) for i,n in enumerate(('x','y','z'))]
    cloud.data = struct.pack('<fff', 2.,0.,0.)
    lastfeed = 0.

    def pump(seconds, predicate=None):
        nonlocal lastfeed
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            if time.monotonic()-lastfeed>.05:
                lastfeed = time.monotonic()
                stamp = node.get_clock().now().to_msg()
                state = State()
                state.header.stamp, state.connected, state.armed, state.mode = stamp, connected, armed, 'POSCTL'
                statepub.publish(state)
                ext = ExtendedState()
                ext.header.stamp, ext.landed_state = stamp, 2 if airborne else 1
                extpub.publish(ext)
                pose = PoseStamped()
                pose.header.stamp, pose.header.frame_id = stamp,'map'
                pose.pose.orientation.w = 1.
                posepub.publish(pose)
                odom = Odometry()
                odom.header.stamp, odom.header.frame_id = stamp,'camera_init'
                odom.pose.pose.orientation.w = 1.
                odompub.publish(odom)
                cloud.header.stamp = stamp
                cloudpub.publish(cloud)
            rclpy.spin_once(node,timeout_sec=.005)
            if predicate and predicate():
                return True
        return bool(predicate and predicate())

    def call(script, action):
        proc = subprocess.Popen(['bash',str(root/script),action],env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        if not pump(15,lambda:proc.poll() is not None):
            raise RuntimeError('command timed out: '+script+' '+action)
        out, err = proc.communicate()
        return proc.returncode,out.decode(),err.decode()

    sentinel = subprocess.Popen(['python3','-c','import time; time.sleep(120)'],start_new_session=True)
    try:
        armed = True
        pump(.3)
        code,_,_ = call('run_board.sh','--start')
        checks['armed_start_refused'] = code != 0 and not (run/'preview/active.json').exists()
        armed, connected = False,False
        pump(.3)
        code,_,_ = call('run_board.sh','--start')
        checks['disconnected_start_refused'] = code != 0
        connected = True
        pump(.3)
        code,out,err = call('run_board.sh','--start')
        checks['shell_starts_native_launch'] = code == 0
        if code:
            raise RuntimeError(out+err)
        initial = json.loads(out)
        checks['native_status_received'] = pump(3,lambda:statuses and statuses[-1].get('session_id') == initial.get('session_id'))
        code,out,_ = call('run_board.sh','--start')
        checks['duplicate_preview_is_same_session'] = code==0 and json.loads(out)['pid']==initial['pid']
        code,out,_ = call('run_board.sh','--status')
        checks['shell_status_reports_owned_launch'] = code==0 and json.loads(out)['running']
        code,out,err = call('run_px4_execution.sh','enable')
        checks['execution_shell_starts_adapter'] = code==0
        if code:
            raise RuntimeError(out+err)
        armed = airborne = True
        checks['synthetic_posctl_prestream'] = pump(4,lambda:len(commands)>2)
        code,_,_ = call('run_board.sh','--stop')
        checks['cannot_stop_planner_during_control_output'] = code != 0
        code,_,_ = call('run_px4_execution.sh','stop')
        checks['execution_shell_controlled_stop'] = code==0
        pump(.4)
        code,out,_ = call('run_px4_execution.sh','status')
        checks['execution_shell_reports_stopped'] = code==0 and not json.loads(out)['running']
        code,out,err = call('run_board.sh','--stop')
        checks['preview_shell_stops_owned_launch'] = code==0 and not json.loads(out)['running']
        checks['unrelated_process_untouched'] = sentinel.poll() is None
        before = len(commands)
        pump(.3)
        checks['no_setpoints_after_stop'] = len(commands)==before
        code,out,_ = call('run_board.sh','--status')
        checks['preview_shell_reports_stopped'] = code==0 and not json.loads(out)['running']
        armed = airborne = False
        pump(.3)
        code,out,err = call('run_board.sh','--start')
        checks['preview_restarts_with_new_session'] = code==0 and json.loads(out)['session_id']!=initial['session_id']
        code,_,_ = call('run_board.sh','--stop')
        checks['restarted_preview_stops_cleanly'] = code==0
        checks['no_mavros_services'] = not any('/mavros/' in name for name,_ in node.get_service_names_and_types())
    except Exception as exc:
        checks['exception_free'] = False
        print('LIFECYCLE_ERROR',repr(exc))
        for path in run.glob('preview/session-*/console.log'):
            print('LAUNCH_LOG',path.read_text(errors='replace')[-5000:])
    finally:
        armed = airborne = False
        try:
            call('run_px4_execution.sh','stop')
            pump(.3)
            call('run_board.sh','--stop')
        except Exception as exc:
            checks['cleanup_succeeded'] = False
            print('CLEANUP_ERROR',repr(exc))
        sentinel.terminate();sentinel.wait(timeout=3)
        shutil.rmtree(run)
        node.destroy_node();rclpy.shutdown()
    result={'passed':bool(checks) and all(checks.values()),'checks':checks,'physical_flight_commands_sent':False,
            'synthetic_fc_commands_received':len(commands),'isolation':{'ROS_DOMAIN_ID':87,'ROS_LOCALHOST_ONLY':1}}
    print('LIFECYCLE_RESULT '+json.dumps(result,sort_keys=True))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
