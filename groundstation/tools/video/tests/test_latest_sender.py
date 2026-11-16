"""隔离验证慢接收时丢弃旧帧，以及停止和断连。"""
import importlib.util, struct, sys, threading, time, types, unittest
from pathlib import Path
rospy=types.ModuleType('rospy');rospy.signal_shutdown=lambda reason:None
sensor=types.ModuleType('sensor_msgs');msgs=types.ModuleType('sensor_msgs.msg')
msgs.CompressedImage=type('CompressedImage',(),{});msgs.Image=type('Image',(),{})
sys.modules.update(rospy=rospy,sensor_msgs=sensor,**{'sensor_msgs.msg':msgs})
spec=importlib.util.spec_from_file_location('bridge',Path(__file__).parents[1]/'ros_image_bridge.py')
bridge=importlib.util.module_from_spec(spec);spec.loader.exec_module(bridge)
class BlockingOutput:
    def __init__(self):self.enter=threading.Event();self.release=threading.Event();self.packets=[]
    def write(self,data):
        self.packets.append(data)
        if len(self.packets)==1:self.enter.set();self.release.wait(2)
    def flush(self):pass
class LatestTest(unittest.TestCase):
    def test_aruco_source_stamp_not_receipt_age(self):
        msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=100,nanosec=0)))
        self.assertTrue(bridge.aruco_source_fresh(msg,100.2))
        self.assertFalse(bridge.aruco_source_fresh(msg,101.1))
        self.assertFalse(bridge.aruco_source_fresh(msg,99.8))
        self.assertFalse(bridge.aruco_source_fresh(msg,float('nan')))
        msg.header.stamp.sec=0
        self.assertFalse(bridge.aruco_source_fresh(msg,0.))
    def test_slow_output_keeps_latest_pending_frame(self):
        output=BlockingOutput();sender=bridge.LatestFrameSender(output,0)
        try:
            sender.submit(lambda:b'first');self.assertTrue(output.enter.wait(1))
            for frame in (b'old1',b'old2',b'latest'):sender.submit(lambda f=frame:f)
            output.release.set();end=time.monotonic()+2
            while len(output.packets)<2 and time.monotonic()<end:time.sleep(.01)
            self.assertEqual([p[8:] for p in output.packets],[b'first',b'latest'])
            self.assertEqual(struct.unpack('>I',output.packets[1][4:8])[0],6)
        finally:output.release.set();sender.stop()
    def test_broken_pipe_shuts_down(self):
        done=threading.Event()
        class Broken:
            def write(self,data):raise BrokenPipeError()
        sender=bridge.LatestFrameSender(Broken(),0,shutdown=lambda reason:done.set())
        try:sender.submit(lambda:b'x');self.assertTrue(done.wait(1))
        finally:sender.stop()
if __name__=='__main__':unittest.main()
