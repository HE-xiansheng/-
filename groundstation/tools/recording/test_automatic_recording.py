#!/usr/bin/python3
"""隔离ROS/真实rosbag：自动录制只读状态，OFFBOARD开始，落地+未解锁结束。"""
import os,sys,subprocess,time,threading,json,signal,importlib.util
from pathlib import Path
assert os.environ.get('ROS_MASTER_URI')=='http://127.0.0.1:11383'
source=Path('/home/tyzk/diff-planner-comparison-20261016/deploy/ego_planner')
base=Path('/home/tyzk/dianyunGCS-review-20261026/.recording-upgrade')/('auto-isolated-'+str(os.getpid()));root=base/'workspace';(root/'src').mkdir(parents=True)
for name in ('record_session.py','recording_manage.py','recording_auto.py'):(root/name).write_bytes((source/name).read_bytes())
sys.path.insert(0,str(root));import recording_manage as m;import recording_auto as auto
master=subprocess.Popen(['roscore','-p','11383'],stdout=(base/'master.log').open('w'),stderr=subprocess.STDOUT);stop=threading.Event();thread=None
try:
 time.sleep(2)
 import rospy,rosbag,rosgraph
 from mavros_msgs.msg import State,ExtendedState
 rospy.init_node('auto_recording_isolated_test',disable_signals=True)
 sp=rospy.Publisher('/mavros/state',State,queue_size=1);lp=rospy.Publisher('/mavros/extended_state',ExtendedState,queue_size=1)
 values={'armed':False,'mode':'POSCTL','landed':1,'pause':False}
 def feed():
  while not stop.is_set():
   if not values['pause']:
    s=State();s.header.stamp=rospy.Time.now();s.connected=True;s.armed=values['armed'];s.mode=values['mode'];sp.publish(s)
    l=ExtendedState();l.header.stamp=s.header.stamp;l.landed_state=values['landed'];lp.publish(l)
   stop.wait(.1)
 thread=threading.Thread(target=feed);thread.start()
 def phase(wanted,timeout=45):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   h=m.health(root)
   if h.get('phase')=='ERROR':raise AssertionError(h)
   if h.get('phase') in wanted:return h
   time.sleep(.1)
  raise AssertionError('phase timeout: '+str(m.health(root)))
 assert not m.health(root)['auto_enabled']
 auto.configure(True,root);phase({'WAIT_MANUAL'})
 end=time.monotonic()+8
 while not m.health(root).get('auto_fresh') and time.monotonic()<end:time.sleep(.1)
 assert m.health(root)['auto_fresh'];time.sleep(.5);assert not list(root.rglob('*.bag*'))
 values.update(armed=True,mode='OFFBOARD',landed=2);first=phase({'RECORDING'});clip=first['clip_id'];run=Path(first['run']);time.sleep(1)
 values.update(mode='POSCTL');time.sleep(.8);assert m.health(root)['phase']=='RECORDING'
 values.update(landed=1);time.sleep(.8);assert m.health(root)['phase']=='RECORDING','landed before disarm stopped recording'
 values.update(armed=False,landed=2);time.sleep(.8);assert m.health(root)['phase']=='RECORDING','disarmed before landed stopped recording'
 values.update(pause=True);time.sleep(2.4);values.update(landed=1);time.sleep(.7);assert m.health(root)['phase']=='RECORDING','stale messages stopped recording'
 values.update(pause=False);saved=phase({'STOPPED'});assert not list(run.glob('*.active'))
 events=[json.loads(line) for line in (run/'automatic-recording-events.jsonl').read_text().splitlines()];assert [e['action'] for e in events]==['start','stop'] and all(e['source_sha256'] for e in events)
 with rosbag.Bag(str(next(run.glob('*.bag')))) as bag:assert bag.get_message_count('/mavros/state')>10
 values.update(armed=True,mode='OFFBOARD',landed=2);second=phase({'RECORDING'});assert second['clip_id']!=clip
 m.request('stop',root);phase({'STOPPED'});time.sleep(1.5);assert m.health(root)['phase']=='STOPPED','manual stop auto restarted during same OFFBOARD'
 auto.configure(False,root);values.update(mode='POSCTL');time.sleep(.5);values.update(mode='OFFBOARD');time.sleep(.8);assert m.health(root)['phase']=='STOPPED'
 pubs=dict(rosgraph.Master(rospy.get_name()).getSystemState()[0]);assert not any(t.startswith('/mavros/setpoint') or t=='/ego_planner/flight_execution' for t in pubs)
 result={'passed':True,'checks':['default_off','prepare_before_flight','offboard_start','POSCTL_continues','landed_alone_continues','disarmed_alone_continues','stale_never_stops','landed_and_disarmed_finalize','real_bag_messages','second_flight_new_clip','manual_stop_does_not_restart_same_OFFBOARD','disable_prevents_future_start','no_control_publishers'],'run':str(run)}
 (base/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
finally:
 stop.set()
 if thread:thread.join(timeout=2)
 for info in (auto.current(root),m.current(root)):
  if info:
   try:os.kill(info['pid'],signal.SIGTERM)
   except OSError:pass
 master.terminate()
 try:master.wait(timeout=8)
 except subprocess.TimeoutExpired:master.kill()
