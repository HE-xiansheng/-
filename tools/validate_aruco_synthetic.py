#!/usr/bin/env python3
"""Exercise deployed detector algorithms with synthetic images in isolated DDS.

Run over SSH stdin after sourcing Humble and the mission workspace. Results go
to stdout. All input/output topics are remapped and no flight services are used.
"""
import json
import os
import signal
import subprocess
import tempfile
import time

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String


def main():
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = '176'
    with tempfile.TemporaryDirectory(prefix='aruco-validation-') as log_dir:
        os.environ['ROS_LOG_DIR'] = log_dir
        rclpy.init()
        node = rclpy.create_node('aruco_synthetic_validation')
        prefix = '/migration_validation'
        publisher = node.create_publisher(Image, prefix + '/image', 1)
        poses, ids, metrics, previews = {}, [], [], []
        def pose_callback(msg):
            poses[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg
        subscriptions = [
            node.create_subscription(PoseStamped, prefix + '/pose', pose_callback, 10),
            node.create_subscription(Int32, prefix + '/id', lambda m: ids.append(m.data), 10),
            node.create_subscription(String, prefix + '/metrics', lambda m: metrics.append(json.loads(m.data)), 10),
            node.create_subscription(Image, prefix + '/preview', lambda m: previews.append((m.width, m.height, m.encoding)), 1),
        ]
        cmd = ['ros2', 'run', 'aruco_test', 'Safina_aruco_test', '--ros-args',
               '--params-file', '/home/orangepi/drone_platform/01_apps/mission_demo/workspace/install/aruco_test/share/aruco_test/config/aruco.yaml',
               '-p', 'img_topic_:=' + prefix + '/image', '-p', 'preview_color:=false',
               '-r', '/aruco_poseresult:=' + prefix + '/pose',
               '-r', '/aruco_target_id:=' + prefix + '/id',
               '-r', '/aruco_detector/metrics:=' + prefix + '/metrics',
               '-r', '/aruco_imgresult:=' + prefix + '/preview']
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        cases = []
        try:
            deadline = time.monotonic() + 15
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                if process.poll() is not None:
                    raise RuntimeError('Detector exited before input discovery')
            if publisher.get_subscription_count() == 0:
                raise RuntimeError('Detector input was not discovered')
            camera = np.array([[2042.0851788597752, 0, 959.0675902004489],
                               [0, 2042.8993693539358, 467.50991502443503], [0, 0, 1]])
            distortion = np.array([-.4849208954761301, .3575460777522052,
                                   .0024881576093297698, -.0002765905391136276, 0.])
            for name, family, marker_id, size in [('original', cv2.aruco.DICT_ARUCO_ORIGINAL, 0, .035),
                                                  ('inverted_5x5', cv2.aruco.DICT_5X5_250, 231, .195),
                                                  ('blank', None, -1, 0), ('wrong_resolution', None, -1, 0)]:
                image = np.full((1080, 1920), 255, dtype=np.uint8)
                if family is not None:
                    if name == 'inverted_5x5':
                        image[:] = 0
                    marker = cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(family), marker_id, 400)
                    if name == 'inverted_5x5':
                        marker = 255 - marker
                    half = size / 2
                    object_corners = np.array([[-half, half, 0], [half, half, 0],
                                               [half, -half, 0], [-half, -half, 0]])
                    pixels, _ = cv2.projectPoints(object_corners, np.array([np.pi, 0., 0.]),
                                                  np.array([0., 0., 1.5]), camera, distortion)
                    transform = cv2.getPerspectiveTransform(np.float32([[0, 0], [399, 0], [399, 399], [0, 399]]),
                                                           pixels.reshape(4, 2).astype(np.float32))
                    image = cv2.warpPerspective(marker, transform, (1920, 1080),
                                                borderValue=0 if name == 'inverted_5x5' else 255)
                if name == 'wrong_resolution':
                    image = cv2.resize(image, (640, 480))
                matched, samples = [], []
                start_ids, start_metrics = len(ids), len(metrics)
                for _ in range(12):
                    message = Image()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.header.frame_id = 'gimbal_camera_optical_frame'
                    message.height, message.width = image.shape
                    message.encoding, message.step = 'mono8', image.shape[1]
                    message.data = image.tobytes()
                    stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
                    publisher.publish(message)
                    deadline = time.monotonic() + .8
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=.03)
                        if stamp in poses:
                            break
                    if stamp in poses:
                        matched.append(poses[stamp])
                        samples.append(poses[stamp].pose.position.z)
                    # Allow family hysteresis and full-frame search to expire.
                    deadline = time.monotonic() + .12
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=.02)
                positive = [z for z in samples if z > 0]
                valid_depth = bool(positive) and abs(float(np.median(positive)) - 1.5) < .15
                passed = (valid_depth and marker_id in ids[start_ids:]) if family is not None else (
                    bool(samples) and all(z < 0 for z in samples) and -1 in ids[start_ids:])
                if name == 'wrong_resolution':
                    passed = passed and any(m['reason'] == 'resolution_mismatch' for m in metrics[start_metrics:])
                expected={sec*1000000000+nsec for sec,nsec in poses}
                pairs=[m for m in metrics[start_metrics:] if m.get('pose_stamp_ns') in expected]
                passed=passed and bool(pairs)
                cases.append({'case': name, 'passed': passed, 'source_stamps_preserved': len(matched), 'metric_pose_pairs':len(pairs),
                              'valid_depths': positive, 'ids': sorted(set(ids[start_ids:]))})
            result = {'passed': all(c['passed'] for c in cases) and bool(previews),
                      'cases': cases, 'annotated_preview_received': bool(previews),
                      'flight_commands_sent': False,
                      'limitations': ['Synthetic front-facing markers only; not physical camera calibration or landing validation',
                                      'No camera-to-airframe transform or PX4 estimator fusion tested']}
            print(json.dumps(result, indent=2))
            return 0 if result['passed'] else 1
        finally:
            os.killpg(process.pid, signal.SIGINT) if process.poll() is None else None
            try:
                output, _ = process.communicate(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                output, _ = process.communicate(timeout=5)
            print(output, file=__import__('sys').stderr)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
