#!/usr/bin/env python3
import io
import os
import selectors
import signal
import socket
import subprocess
import sys
import time
import tempfile
import textwrap
from pathlib import Path
import xmlrpc.client
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from PIL import Image as PILImage
from sensor_msgs.msg import CompressedImage, Image


def main():
    # 在独立 ROS master 上验证节点生命周期和目标话题隔离，不连接真实飞控。
    bridge = str(Path(__file__).resolve().parent / 'ros_image_bridge.py')
    test_temp = tempfile.TemporaryDirectory(prefix='gcs-person-monitor-test-')
    test_dir = Path(test_temp.name)
    workspace = str(test_dir / 'workspace')
    setup = Path(workspace) / 'devel/setup.bash'
    setup.parent.mkdir(parents=True)
    setup.write_text('true\n')
    detector = Path(workspace) / 'devel/lib/manual_follow_yolo/manual_follow_yolo_node'
    detector.parent.mkdir(parents=True)
    detector.write_text(textwrap.dedent('''\
    #!/usr/bin/env python3
    import numpy as np
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32MultiArray, String

    rospy.init_node('fake_person_detector')
    image_pub = rospy.Publisher('/manual_person_follow/image', Image, queue_size=1)
    track_pub = rospy.Publisher('/manual_person_follow/track', Float32MultiArray, queue_size=1)
    target_pub = rospy.Publisher('/manual_person_follow/target', String, queue_size=1)
    image = np.zeros((720, 1280, 3), dtype=np.uint8)
    image[:] = (230, 10, 10)
    msg = CvBridge().cv2_to_imgmsg(image, encoding='bgr8')
    rate = rospy.Rate(12)
    while not rospy.is_shutdown():
        image_pub.publish(msg)
        track_pub.publish(Float32MultiArray(data=[-1.0]))
        target_pub.publish(String(data='test'))
        rate.sleep()
    '''))
    detector.chmod(0o755)
    model = Path(workspace) / 'src/manual_follow_yolo/weights/yolov11_s_det_int.rknn'
    model.parent.mkdir(parents=True)
    model.touch()
    sock = socket.socket()
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
    sock.close()
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:{}'.format(port)
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ['ROS_LOG_DIR'] = str(test_dir / 'roslog')
    os.environ['GCS_PERSON_WORKSPACE'] = workspace
    os.makedirs(os.environ['ROS_LOG_DIR'], exist_ok=True)
    core_log = open(test_dir / 'roscore.log', 'wb')
    core = subprocess.Popen(['roscore', '-p', str(port)], stdout=core_log, stderr=subprocess.STDOUT, start_new_session=True)
    master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
    children = []
    external_pub = None

    def wait_for(condition, message, timeout=8):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if condition():
                return
            time.sleep(.1)
        raise AssertionError(message)

    def publishers():
        return dict(master.getSystemState('/video_test')[2][0])

    def image_nodes():
        return publishers().get('/manual_person_follow/image', [])

    def monitor_nodes():
        return [n for n in image_nodes() if n.startswith('/gcs_person_monitor_')]

    def process_for_node(node):
        code, _, uri = master.lookupNode('/video_test', node)
        assert code == 1
        return xmlrpc.client.ServerProxy(uri).getPid('/video_test')[2]

    def alive(pid):
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        try:
            if open('/proc/{}/stat'.format(pid)).read().rsplit(')', 1)[1].split()[0] == 'Z':
                return False
        except FileNotFoundError:
            return False
        return True

    def start(mode, env=None):
        p = subprocess.Popen([sys.executable, '-u', bridge, '--source', mode],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
        children.append(p)
        return p

    def receive(p, mode, timeout=10):
        sel = selectors.DefaultSelector()
        sel.register(p.stdout, selectors.EVENT_READ)
        data = bytearray()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if p.poll() is not None:
                raise AssertionError('视频桥接退出: ' + p.stderr.read().decode())
            if not sel.select(.1):
                continue
            data.extend(os.read(p.stdout.fileno(), 65536))
            if len(data) < 8:
                continue
            assert data[:4] == b'VID1'
            size = int.from_bytes(data[4:8], 'big')
            assert 0 < size < 8 * 1024 * 1024
            if len(data) < size + 8:
                continue
            image = PILImage.open(io.BytesIO(bytes(data[8:size + 8])))
            image.load()
            assert image.format == 'JPEG' and image.size == (640, 360)
            pixel = image.convert('RGB').getpixel((20, 20))
            assert (pixel[0] > 200 and pixel[2] < 30) if mode == 'raw' else (pixel[2] > 200 and pixel[0] < 30)
            return
        raise AssertionError('等待 {} 画面超时'.format(mode))

    def stop(p, sig=signal.SIGTERM):
        if p.poll() is None:
            p.send_signal(sig)
            p.wait(timeout=7)

    def _master_ready():
        # master 尚未就绪时 XML-RPC 会拒绝连接。
        try:
            return master.getPid('/video_test')[0] == 1
        except (OSError, xmlrpc.client.Error):
            return False

    try:
        wait_for(_master_ready, 'ROS master 启动超时')
        rospy.init_node('gcs_monitor_test', disable_signals=True)
        raw_pub = rospy.Publisher('/camera/color/image_raw/compressed', CompressedImage, queue_size=1)
        raw_img = np.zeros((720, 1280, 3), dtype=np.uint8)
        raw_img[:] = (10, 10, 230)
        raw = CompressedImage(format='jpeg', data=cv2.imencode('.jpg', raw_img)[1].tobytes())
        blue_img = np.zeros((720, 1280, 3), dtype=np.uint8)
        blue_img[:] = (230, 10, 10)
        blue = CvBridge().cv2_to_imgmsg(blue_img, encoding='bgr8')

        def publish(_):
            raw_pub.publish(raw)
            if external_pub is not None:
                external_pub.publish(blue)
        timer = rospy.Timer(rospy.Duration(.08), publish)

        p = start('raw')
        receive(p, 'raw')
        stop(p)
        assert not monitor_nodes()
        print('PASS 原画模式不启动识别', flush=True)

        first = start('person')
        receive(first, 'person')
        wait_for(lambda: len(monitor_nodes()) == 1, '未启动独立检测节点')
        node = monitor_nodes()[0]
        first_pid = process_for_node(node)
        topics = publishers()
        assert '/manual_person_follow/track' not in topics
        assert '/manual_person_follow/target' not in topics
        assert any(k.startswith(node + '/') and k.endswith('/track') for k in topics), topics
        assert any(k.startswith(node + '/') and k.endswith('/target') for k in topics), topics
        assert not any(k.startswith('/mavros/setpoint') for k in topics)
        print('PASS 识别节点和目标话题隔离', flush=True)

        second = start('person')
        receive(second, 'person')
        assert monitor_nodes() == [node], monitor_nodes()
        stop(first)
        wait_for(lambda: not alive(first_pid), '第一个地面站的检测进程未退出')
        wait_for(lambda: len(monitor_nodes()) == 1 and process_for_node(monitor_nodes()[0]) != first_pid,
                 '第二个地面站未接管检测节点', timeout=10)
        receive(second, 'person')
        stop(second)
        wait_for(lambda: not monitor_nodes(), '关闭后检测节点未注销')
        print('PASS 双地面站复用及第一个退出后的接管', flush=True)

        external_pub = rospy.Publisher('/manual_person_follow/image', Image, queue_size=1)
        wait_for(lambda: '/gcs_monitor_test' in image_nodes(), '外部画面发布者未启动')
        reused = start('person')
        receive(reused, 'person')
        assert not monitor_nodes()
        stop(reused)
        assert '/gcs_monitor_test' in image_nodes()
        external_pub.unregister()
        external_pub = None
        wait_for(lambda: not image_nodes(), '外部画面发布者未注销')
        print('PASS 外部识别画面复用及保留', flush=True)

        killed = start('person')
        receive(killed, 'person')
        wait_for(lambda: len(monitor_nodes()) == 1, '检测节点未启动')
        child_pid = process_for_node(monitor_nodes()[0])
        stop(killed, signal.SIGKILL)
        wait_for(lambda: not alive(child_pid), '桥接被强杀后检测进程仍在运行', timeout=10)
        wait_for(lambda: not monitor_nodes(), '桥接强杀后检测节点未注销')
        print('PASS 桥接异常退出时检测节点退出', flush=True)

        env = dict(os.environ, GCS_PERSON_WORKSPACE=str(test_dir / 'missing'))
        failure = start('person', env)
        failure.wait(timeout=8)
        error = failure.stderr.read().decode()
        assert failure.returncode != 0 and 'GCS_PERSON_ERROR:' in error, error
        print('PASS 工作区不存在时返回明确错误', flush=True)
    finally:
        for child in children:
            if child.poll() is None:
                child.kill()
                child.wait()
        if rospy.core.is_initialized():
            rospy.signal_shutdown('测试结束')
        if core.poll() is None:
            os.killpg(core.pid, signal.SIGINT)
            try:
                core.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(core.pid, signal.SIGKILL)
                core.wait()
        core_log.close()
        test_temp.cleanup()

if __name__ == '__main__':
    main()
