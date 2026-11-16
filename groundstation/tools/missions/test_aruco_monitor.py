"""无ROS网络的监视回归：源戳、同帧匹配及latched旧状态。"""
import importlib.util,math,time,unittest
from pathlib import Path
from types import SimpleNamespace
spec=importlib.util.spec_from_file_location('manager',Path(__file__).with_name('task_manager.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class ArucoMonitorTests(unittest.TestCase):
    def setUp(self):
        self.pose=dict(x=.12,y=-.05,z=1.2,stamp=100.,seq=7,frame='optical')
        self.detector=dict(selected_id=231,reason='selected_pose_valid',pose_stamp=100.,pose_seq=7,source_stamp_known=True)
    def result(self,now=100.1):return m.aruco_summary(self.pose,self.detector,{},now)
    def test_same_frame_valid(self):
        r=self.result();self.assertTrue(r['valid']);self.assertEqual(r['id'],231);self.assertEqual(r['offset']['x'],.12)
    def test_repeated_delivery_cannot_refresh_source(self):
        r=self.result(100.401);self.assertFalse(r['valid']);self.assertEqual(r['reason'],'marker_source_stale');self.assertEqual(r['offset'],{})
    def test_missing_or_future_stamp(self):
        for stamp in (0.,100.21,float('nan')):
            self.pose['stamp']=stamp;self.assertFalse(self.result()['valid'])
    def test_invalid_pose(self):
        for key,value in [('z',-1.),('x',math.inf),('y',math.nan)]:
            with self.subTest(key=key):
                original=self.pose[key];self.pose[key]=value;self.assertFalse(self.result()['valid']);self.pose[key]=original
    def test_cross_topic_different_frame(self):
        for key,value in [('pose_seq',8),('pose_stamp',100.02)]:
            with self.subTest(key=key):
                original=self.detector[key];self.detector[key]=value;self.assertEqual(self.result()['reason'],'detector_pose_not_synchronized');self.detector[key]=original
    def test_old_detector_reports_unknown_instead_of_valid(self):
        self.detector.pop('pose_stamp');self.assertEqual(self.result()['reason'],'detector_stamp_unavailable')
    def test_unstamped_capture_marked_unknown(self):
        self.detector['source_stamp_known']=False;r=self.result();self.assertTrue(r['valid']);self.assertFalse(r['source_stamp_known'])
    def test_negative_id_or_no_detection(self):
        self.detector.update(selected_id=-1,reason='no_configured_id');self.assertFalse(self.result()['valid'])
    def test_latched_landing_source_age(self):
        ros=m.RosState.__new__(m.RosState);ros.cache={};ros.map_session=lambda:None
        ros.rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:100.1)))
        for key,value in dict(landing_header=['ros_time','phase','reason'],landing_row=['90','CANCELLED','rc_switch_out']).items():ros.cache[key]=(value,time.monotonic())
        self.assertFalse(ros.summary()['aruco']['landing_fresh'])
        ros.cache['landing_row']=(['100','CANCELLED','rc_stamp_stale'],time.monotonic())
        self.assertEqual(ros.summary()['aruco']['landing']['reason'],'rc_stamp_stale')
        ros.cache['landing_row']=(['100','CANCELLED'],time.monotonic());self.assertFalse(ros.summary()['aruco']['landing_fresh'])
    def test_fresh_receipt_cannot_make_stale_fc_safe_for_stop(self):
        ros=m.RosState.__new__(m.RosState);ros.cache={'fc':({'connected':True,'armed':False,'mode':'POSCTL'},time.monotonic())}
        ros.rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:100.1)))
        value,rx=ros.cache['fc'];ros.cache['fc']=(value,rx,98.);self.assertIsNone(ros.value('fc'))
        ros.cache['fc']=(value,rx,100.);self.assertIsNotNone(ros.value('fc'))
        ros.cache['fc']=(value,rx,0.);self.assertIsNone(ros.value('fc'))
    def test_aruco_run_freezes_config_and_owns_diagnostics(self):
        from tempfile import TemporaryDirectory
        import xml.etree.ElementTree as ET
        with TemporaryDirectory() as d:
            path=Path(d);cfg=path/'config.snapshot.yaml';log=path/'landing-diagnostics'
            xml=ET.fromstring(m.launch_xml('aruco',path,aruco_config=cfg,diagnostics=log))
            self.assertEqual(xml.find('rosparam').attrib['file'],str(cfg))
            self.assertEqual(next(p for p in xml.findall('param') if p.attrib['name'].endswith('diagnostic_directory')).attrib['value'],str(log))
    def test_private_scripts_refuse_binary_without_gcu_stub(self):
        import subprocess,sys,tempfile
        folder=Path(__file__).parent
        run=subprocess.run([sys.executable,str(folder/'test_aruco_preparation.py'),'/bin/true',str(folder/'test-native/fixtures/takeoff_land.snapshot.yaml')],capture_output=True,text=True)
        self.assertNotEqual(run.returncode,0);self.assertIn('GCU',run.stderr)
        with tempfile.TemporaryDirectory() as d:
            for name in ('aruco_landing_fixture','aruco_detector_fixture'):(Path(d)/name).symlink_to('/bin/true')
            run=subprocess.run([sys.executable,str(folder/'test_aruco_isolated.py'),d],capture_output=True,text=True)
            self.assertNotEqual(run.returncode,0);self.assertIn('GCU',run.stderr)
if __name__=='__main__':unittest.main()
