"""隔离任务管理回归：假的roslaunch进程，不连接ROS或真实飞控。"""
import importlib.util, json, os, signal, tempfile, threading, time, unittest, subprocess, sys
from pathlib import Path
from unittest.mock import patch
spec=importlib.util.spec_from_file_location('task_manager',Path(__file__).with_name('task_manager.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

class FakeRos:
    def __init__(self):self.fc={'connected':True,'armed':False,'mode':'POSCTL'};self.conflict={}
    def wait_fc(self):return self.fc
    def conflicts(self):return self.conflict
    def publishers(self):return {}
    def value(self,key,*args):return self.fc if key=='fc' else None
    def summary(self):return {'fc':self.fc,'aruco':{},'waypoint':{},'tracking':{}}
    def map_session(self):return {'pid':123,'ticks':'1','boot':'fake'}
    def map_alignment(self):return (dict(p=[0,0,0],q=[0,0,0,1]),dict(p=[0,0,0],q=[0,0,0,1]))

class Tests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.bin=self.root/'bin';self.bin.mkdir()
        launch=self.bin/'roslaunch';launch.write_text('#!/usr/bin/python3\nimport time,signal,sys\nsignal.signal(signal.SIGINT,lambda *args:sys.exit(0))\nwhile True:time.sleep(.1)\n');launch.chmod(0o755)
        self.env=patch.dict(os.environ,PATH=str(self.bin)+':'+os.environ['PATH']);self.env.start()
        for name in ('land_test/config/takeoff_land.yaml','track_test/config/track.yaml','yoloUltra_test/config/yolo_track.yaml'):
            p=self.root/'src'/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('valid: true\n')
        self.ros=FakeRos();self.manager=m.Manager(self.root,self.root/'store',self.ros)
    def tearDown(self):
        record=self.manager.load()
        if m.owned(record):
            os.killpg(record['pid'],signal.SIGINT)
            try:os.waitpid(record['pid'],0)
            except ChildProcessError:pass
        self.env.stop();self.temp.cleanup()
    def test_map_coordinates_with_yaw_offset_and_photo(self):
        import math
        lio=dict(p=[1,2,0],q=[0,0,0,1])
        fc=dict(p=[10,20,.1],q=[0,0,math.sqrt(.5),math.sqrt(.5)])
        payload=dict(version=4,frame='fastlio_world',waypoints=[dict(x=2,y=2,z=1,photo=True),dict(x=1,y=3,z=1.5,photo=False)])
        result=m.map_mission(payload,lio,fc,{'mission':{'max_speed':.5,'geofence_radius':25,'rc':{'channel':5}}})['mission']
        self.assertEqual(result['rc'],{'channel':5});self.assertEqual(result['max_speed'],.5)
        self.assertFalse(result['manual_airborne']);self.assertAlmostEqual(result['takeoff_z'],1.1);self.assertTrue(result['auto_land']);self.assertTrue(result['return_to_start'])
        a,b=result['waypoints'];self.assertAlmostEqual(a['x'],10);self.assertAlmostEqual(a['y'],21);self.assertAlmostEqual(a['z'],1.1)
        self.assertAlmostEqual(b['x'],9);self.assertAlmostEqual(b['y'],20)
        self.assertEqual(a['gimbal_pitch'],-90);self.assertEqual(a['hold'],2);self.assertEqual(b['hold'],2)
        self.assertNotIn('gimbal_pitch',b)
    def test_old_ground_station_is_rejected(self):
        pose=dict(p=[0,0,0],q=[0,0,0,1]);template={'mission':{}}
        for version in (1,2,3):
            payload=dict(version=version,frame='fastlio_world',waypoints=[dict(x=0,y=0,z=1)])
            with self.assertRaisesRegex(ValueError,'版本过旧'):m.map_mission(payload,pose,pose,template)
    def test_map_increases_only_ascent_limit(self):
        pose=dict(p=[0,0,0],q=[0,0,0,1])
        template={'mission':{'max_speed':.5,'max_z_speed':.25,'max_z_speed_up':.25}}
        payload=dict(version=4,frame='fastlio_world',waypoints=[dict(x=1,y=0,z=1)])
        result=m.map_mission(payload,pose,pose,template)['mission']
        self.assertEqual(result['max_z_speed_up'],.5)
        self.assertEqual(result['max_z_speed'],.25)
        self.assertEqual(result['max_speed'],.5)
        self.assertEqual(template['mission']['max_z_speed_up'],.25)
    def test_native_must_confirm_ascent_limit(self):
        p=self.root/'ascent.yaml'
        p.write_text('mission: {max_z_speed_up: 0.5, waypoints: [{x: 0, y: 0, z: 1}]}')
        for output in ('old parser ignores unknown field','max_z_speed_up: 0.25 m/s'):
            with self.subTest(output=output), patch.object(m.subprocess,'run',return_value=subprocess.CompletedProcess([],0,output)):
                with self.assertRaisesRegex(RuntimeError,'未确认独立上升限速'):self.manager.validate(str(p))
        with patch.object(m.subprocess,'run',return_value=subprocess.CompletedProcess([],0,'max_z_speed_up: 0.5 m/s, max_z_speed_down: 0.25 m/s')):
            self.assertTrue(self.manager.validate(str(p))['ok'])
    def test_takeoff_height_must_be_above_current_pose(self):
        pose=dict(p=[0,0,1.2],q=[0,0,0,1]);template={'mission':{}}
        payload=dict(version=4,frame='fastlio_world',waypoints=[dict(x=1,y=0,z=1.0)])
        with self.assertRaisesRegex(ValueError,'必须高于'):m.map_mission(payload,pose,pose,template)
    def test_map_task_rejects_already_armed(self):
        self.ros.fc.update(armed=True,mode='POSCTL')
        with self.assertRaisesRegex(RuntimeError,'落地并锁桨'):
            self.manager.start('waypoint','',dict(map_session=self.ros.map_session()))
        self.assertFalse(self.manager.load())
    def test_invalid_map_request_never_passes(self):
        pose=dict(p=[0,0,0],q=[0,0,0,1]);template={'mission':{}}
        for value in (float('nan'),float('inf'),True,'1'):
            with self.assertRaises(ValueError):m.map_mission(dict(version=4,frame='fastlio_world',waypoints=[dict(x=value,y=0,z=1)]),pose,pose,template)
    def test_old_native_program_cannot_silently_auto_takeoff(self):
        p=self.root/'old-parser.yaml';p.write_text('mission: {manual_airborne: true, waypoints: [{x: 0, y: 0, z: 1}]}')
        with patch.object(m.subprocess,'run',return_value=type('Result',(),dict(returncode=0,stdout='old parser ignores unknown field'))()):
            with self.assertRaisesRegex(RuntimeError,'未确认任务起飞模式'):self.manager.validate(str(p))
    def test_map_rejects_parser_without_landing_confirmation(self):
        p=self.root/'map.yaml';p.write_text('mission: {name: gcs_map, manual_airborne: false, auto_land: true, waypoints: [{x: 0, y: 0, z: 1}]}')
        with patch.object(m.subprocess,'run',return_value=type('Result',(),dict(returncode=0,stdout='manual_airborne: false\nreturn_to_start: false, auto_land: false'))()):
            with self.assertRaisesRegex(RuntimeError,'未确认返回起点并自动降落'):self.manager.validate(str(p))
    def test_map_upload_freezes_actual_mission(self):
        p=self.root/'src/waypoint_mission/missions/abc_photo.yaml';p.parent.mkdir(parents=True);p.write_text('mission: {max_speed: 0.5, rc: {channel: 5}}')
        payload=dict(version=4,frame='fastlio_world',map_session=self.ros.map_session(),waypoints=[dict(x=1,y=2,z=1,photo=True)])
        checked=[]
        with patch.object(self.manager,'validate',side_effect=lambda path:checked.append(path)):
            self.manager.start('waypoint','',payload)
        record=self.manager.load();cfg=m.yaml_data(Path(record['mission_snapshot']))['mission']
        self.assertEqual(checked,[record['mission_snapshot']]);self.assertFalse(cfg['manual_airborne']);self.assertEqual(cfg['takeoff_z'],cfg['waypoints'][0]['z']);self.assertEqual(cfg['waypoints'][0]['hold'],2)
        self.assertEqual(cfg['max_z_speed_up'],.5)
        self.assertTrue((Path(record['run'])/'map-request.json').is_file());self.assertTrue((Path(record['run'])/'alignment.json').is_file())
    def test_old_map_session_never_launches(self):
        with self.assertRaisesRegex(RuntimeError,'地图定位会话已变化'):
            self.manager.start('waypoint','',dict(map_session={'pid':1}))
        self.assertFalse(self.manager.load())
    def test_no_implicit_task_on_snapshot(self):
        state=self.manager.snapshot();self.assertFalse(state['running']);self.assertFalse((self.root/'store/active.json').exists())
    def test_own_task_start_is_idempotent_and_switch_requires_stop(self):
        self.manager.start('aruco','');pid=self.manager.load()['pid']
        self.manager.start('aruco','');self.assertEqual(pid,self.manager.load()['pid'])
        with self.assertRaisesRegex(RuntimeError,'先停止'):self.manager.start('tracking','')
        self.assertTrue(m.owned(self.manager.load()))
    def test_competitor_not_terminated(self):
        self.ros.conflict={'/mavros/setpoint_raw/local':['/existing_ego']}
        with self.assertRaisesRegex(RuntimeError,'已有飞控控制器'):self.manager.start('aruco','')
        self.assertFalse((self.root/'store/active.json').exists())
    def test_armed_offboard_does_not_start_another_task(self):
        self.ros.fc.update(armed=True,mode='OFFBOARD')
        with self.assertRaisesRegex(RuntimeError,'遥控接管'):self.manager.start('tracking','')
    def test_bad_yaml_never_launches(self):
        (self.root/'src/land_test/config/takeoff_land.yaml').write_bytes(b'x:\0bad')
        with self.assertRaisesRegex(ValueError,'空字节'):self.manager.start('aruco','')
        self.assertFalse((self.root/'store/active.json').exists())
    def test_pid_reuse_does_not_own_process(self):
        self.manager.start('aruco','');record=self.manager.load();record['ticks']='bad'
        self.assertFalse(m.owned(record))
    def test_stop_waits_for_pilot_and_then_reaps_task(self):
        self.manager.start('aruco','');record=self.manager.load();record['stop_pending']=True;self.manager.save(record)
        self.ros.fc.update(armed=True,mode='OFFBOARD')
        worker=threading.Thread(target=self.manager.stop_worker,daemon=True);worker.start();time.sleep(.4)
        self.assertTrue(m.owned(record),'OFFBOARD时不能杀控制器')
        self.ros.fc.update(mode='POSCTL');worker.join(6)
        self.assertFalse(worker.is_alive());self.assertFalse(m.owned(record));self.assertFalse(self.manager.load()['stop_pending'])
    def test_launch_preserves_native_rc_and_reuses_camera(self):
        xml=m.launch_xml('aruco',self.root,camera=False)
        self.assertIn('value="true"',xml);self.assertIn('gimbal_down_on_startup',xml);self.assertNotIn('rtsp_test',xml)
        self.assertNotIn('start_trigger',xml);self.assertNotIn('auto_takeoff',xml)
        self.assertIn('Safina_land.launch',xml)
    def test_mission_path_is_explicit_and_xml_escaped(self):
        with self.assertRaises(ValueError):m.mission_path('',self.root)
        p=self.root/'my & mission.yaml';p.write_text('mission:\n  waypoints:\n    - {x: 0, y: 0, z: 1}\n')
        self.assertEqual(m.mission_path(str(p),self.root),p)
        xml=m.launch_xml('waypoint',self.root,str(p));self.assertIn('&amp;',xml)
        import xml.etree.ElementTree as ET
        ET.fromstring(xml)
    def test_validate_uses_unique_node_name(self):
        p=self.root/'mission.yaml';p.write_text('mission:\n  waypoints:\n    - {x: 0, y: 0, z: 1}\n')
        class Result:
            returncode=0
            stdout='parsed'
        with patch.object(m.subprocess,'run',return_value=Result()) as call:
            self.manager.validate(str(p))
            args=call.call_args[0][0]
            self.assertIn('_dry_run:=true',args)
            self.assertTrue(any(x.startswith('__name:=gcs_waypoint_validate_') for x in args))
    def test_mission_executes_a_frozen_validated_snapshot(self):
        p=self.root/'mission.yaml';p.write_text('mission:\n  waypoints:\n    - {x: 1, y: 0, z: 1}\n')
        validated=[]
        def validate(path):
            validated.append(path)
            return {'ok':True,'mission_file':path}
        with patch.object(self.manager,'validate',side_effect=validate):
            self.manager.start('waypoint',str(p))
        r=self.manager.load();self.assertNotEqual(r['mission_snapshot'],str(p))
        self.assertEqual(validated,[r['mission_snapshot']])
        p.write_text('changed later')
        self.assertIn('x: 1',Path(r['mission_snapshot']).read_text())
    def test_orphan_child_remains_owned_until_group_exits(self):
        code="import subprocess,sys,time; p=subprocess.Popen([sys.executable,'-c','import time;time.sleep(20)']); print(p.pid,flush=True);time.sleep(.3)"
        p=subprocess.Popen([sys.executable,'-c',code],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,start_new_session=True)
        record=m.identity(p.pid)
        try:
            child=int(p.stdout.readline());p.wait(timeout=2)
            self.assertTrue(m.owned(record),'roslaunch退出不能漏掉仍存活子进程')
        finally:
            try:os.killpg(p.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            p.stdout.close()
    def test_exited_task_is_not_reported_as_prepared(self):
        self.manager.start('aruco','');record=self.manager.load()
        os.killpg(record['pid'],signal.SIGINT)
        for _ in range(30):
            if not m.owned(record):break
            time.sleep(.05)
        state=self.manager.snapshot();self.assertFalse(state['running'])
        self.assertIn('已退出',state['message'])
    def test_status_event_keeps_spaces(self):
        fields=m.parse_fields('phase=ABORT wp=1/5 last_event=pose stale after takeoff')
        self.assertEqual(fields['last_event'],'pose stale after takeoff');self.assertEqual(fields['wp'],'1/5')

if __name__=='__main__':unittest.main()
