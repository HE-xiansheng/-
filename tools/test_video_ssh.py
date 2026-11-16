#!/usr/bin/env python3
"""真实SSH→ROS2图像→VID1→原Qt控制器；仅使用localhost隔离域合成画面。"""
import os, subprocess, sys, tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
SSH=['/usr/bin/ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','UserKnownHostsFile=/home/tyzk/.planning/2026-11-16-ros2-platform-audit/emmc_known_hosts','-o','HostKeyAlias=opi-ros2-emmc']
REMOTE='''import os, signal, time
import cv2, rclpy
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge
rclpy.init();n=rclpy.create_node('ssh_video_fixture');b=CvBridge()
pubs=[n.create_publisher(CompressedImage,'/camera/color/image_raw/compressed',1),n.create_publisher(Image,'/yoloUltra_imgresult',1),n.create_publisher(Image,'/aruco_imgresult',1)]
pixels=cv2.imread('/tmp/gcs-video-ros2-check/bus.jpg');end=time.monotonic()+40
stop=False

def halt(*args):
 global stop
 stop=True
signal.signal(signal.SIGTERM,halt)
try:
 while not stop and time.monotonic()<end:
  for i,p in enumerate(pubs):
   if i==0:
    m=CompressedImage();m.format='jpeg';m.data=cv2.imencode('.jpg',pixels)[1].tobytes()
   else:m=b.cv2_to_imgmsg(pixels,encoding='bgr8')
   m.header.frame_id='ssh_fixture';m.header.stamp=n.get_clock().now().to_msg();p.publish(m)
  rclpy.spin_once(n,timeout_sec=.1)
finally:n.destroy_node();rclpy.shutdown()
'''
command="bash -lc 'source /home/orangepi/drone_platform/env.sh && ROS_DOMAIN_ID=199 ROS_LOCALHOST_ONLY=1 python3 -'"
with tempfile.TemporaryDirectory(prefix='gcs-video-ssh-') as tmp:
    tmp=Path(tmp)
    wrapper=tmp/'ssh'
    wrapper.write_text('#!/usr/bin/python3\nimport os,sys\nargs=sys.argv[1:]\nargs[-1]="export ROS_DOMAIN_ID=199 ROS_LOCALHOST_ONLY=1; "+args[-1]\nos.execv("/usr/bin/ssh",'+repr(SSH)+'+args)\n')
    wrapper.chmod(0o700)
    fixture=subprocess.Popen(SSH+['orangepi@192.168.1.224',command],stdin=subprocess.PIPE)
    fixture.stdin.write(REMOTE.encode());fixture.stdin.close()
    try:
        env=dict(os.environ,PATH=str(tmp)+':'+os.environ['PATH'],QT_QPA_PLATFORM='offscreen')
        rc=subprocess.call(['/home/tyzk/drone-migration-work/build-groundstation-video-test/video_reconnect_test','--real-ssh'],env=env)
    finally:
        # 固定40秒期限；保留SSH至fixture退出，避免遗留远端测试发布者。
        fixture.wait(timeout=50)
    sys.exit(rc)
