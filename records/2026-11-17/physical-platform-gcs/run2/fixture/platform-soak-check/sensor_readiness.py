"""Wait for fresh, progressing LiDAR and IMU before starting a LIO session."""
import json
import struct
import time


def wait_for_sensors(timeout=15., hold=1.):
    import rclpy
    from livox_ros_driver2.msg import CustomMsg
    from sensor_msgs.msg import Imu
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node('lio_sensor_readiness', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    samples = {}
    counts = {'lidar': 0, 'imu': 0}
    oldest = {'lidar': 0., 'imu': 0.}
    regressions = {'lidar': 0, 'imu': 0}
    stable_since = None
    trace = []
    # A 10 Hz cloud stamp identifies scan start, about 100 ms before receipt.
    # Between clouds its age approaches 200 ms; allow 100 ms of delivery jitter.
    age_limit = {'lidar': .3, 'imu': .1}

    def receive(kind, raw):
        nonlocal stable_since
        if len(raw) < 12 or raw[:4] != b'\x00\x01\x00\x00':
            stable_since = None
            return
        sec, ns = struct.unpack_from('<iI', raw, 4)
        stamp = sec*10**9+ns
        if ns >= 10**9 or stamp <= 0:
            stable_since = None
            return
        previous = samples.get(kind)
        if previous and stamp <= previous[0]:
            regressions[kind] += 1
            stable_since = None
        samples[kind] = (stamp, time.monotonic())
        counts[kind] += 1
        oldest[kind] = max(oldest[kind], (node.get_clock().now().nanoseconds-stamp)/1e9)

    subscriptions = [node.create_subscription(typ, topic, lambda raw, k=kind: receive(k, raw),
        qos_profile_sensor_data, raw=True) for kind, typ, topic in
        [('lidar', CustomMsg, '/livox/lidar'), ('imu', Imu, '/livox/imu')]]
    started = time.monotonic()
    try:
        while time.monotonic()-started < timeout:
            executor.spin_once(timeout_sec=.02)
            now = node.get_clock().now().nanoseconds
            wall = time.monotonic()
            if not trace or wall-started-trace[-1]['elapsed_s'] >= 1.:
                trace.append({'elapsed_s': wall-started, 'counts': dict(counts),
                    'age_s': {k: (now-v[0])/1e9 for k, v in samples.items()}})
            fresh = len(samples) == 2 and all(
                -.02 <= (now-stamp)/1e9 <= age_limit[kind] and wall-receipt <= .2
                for kind, (stamp, receipt) in samples.items())
            aligned = fresh and abs(samples['lidar'][0]-samples['imu'][0]) <= 300_000_000
            if not aligned:
                stable_since = None
                continue
            stable_since = wall if stable_since is None else stable_since
            if wall-stable_since >= hold:
                return {'ready': True, 'wait_s': wall-started, 'hold_s': hold,
                    'observed_counts': counts, 'startup_max_age_s': oldest,
                    'startup_regressions': regressions,
                    'trace': trace,
                    'final_source_age_s': {k: (now-v[0])/1e9 for k, v in samples.items()}}
        raise RuntimeError('LiDAR/IMU not ready; FAST-LIO not started: '+json.dumps(trace))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
