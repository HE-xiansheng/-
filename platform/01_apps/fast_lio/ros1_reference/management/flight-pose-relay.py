"""Launch proven vendor pose relay on fixed LIO; no controller/flight commands.
No timeout or armed-stop: intended to keep estimator input alive during user flight.
"""
import sys,json,os,signal,time,subprocess
from pathlib import Path
ROOT=Path.home()/'drone_platform/01_apps/fast_lio'
sys.path.insert(0,str(ROOT/'management'))
import board
state=ROOT/'active-flight-pose-relay.json'
action=sys.argv[1] if len(sys.argv)>1 else 'status'
if action=='start':
 source=board.verify_source();s,pubs=board.gate()
 conflicts={t:pubs[t] for t in ['/mavros/vision_pose/pose','/mavros/vision_pose/pose_cov','/mavros/odometry/out','/Odometry'] if pubs.get(t)}
 if conflicts:raise RuntimeError('已有定位来源/转发，不自动替换: '+str(conflicts))
 executable=Path.home()/'drone_platform/02_drivers/sensors/lib/to_mavros/vision2mavros'
 if not executable.is_file():raise RuntimeError('原厂转发程序不存在')
 run=ROOT/'runs'/time.strftime('%Y%m%d-%H%M%S-flight-pose-relay');run.mkdir()
 cmd=[str(executable),'__name:=factory_vision2mavros','_vision_pose_topic:=/factory_fastlio/odometry','_publish_rate:=30.0','_window_size:=50','_body_lidar_x:=0.0','_body_lidar_y:=0.0','_body_lidar_z:=0.0']
 with (run/'relay.log').open('w') as log:p=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 time.sleep(1)
 if p.poll() is not None:raise RuntimeError('转发退出: '+(run/'relay.log').read_text()[-1000:])
 d={'pid':p.pid,'run':str(run),'command':cmd,'source':source,'note':'continuous vendor pose-only relay; no controller; vendor cached-pose behavior unchanged'}
 state.write_text(json.dumps(d,indent=2)+'\n');(run/'start.json').write_text(json.dumps(d,indent=2)+'\n');print(json.dumps(d,indent=2))
elif action=='stop':
 board.gate()
 d=json.loads(state.read_text());pid=d['pid'];cmd=(Path('/proc')/str(pid)/'cmdline').read_bytes()
 if b'__name:=factory_vision2mavros' not in cmd:raise RuntimeError('PID身份不匹配')
 os.killpg(pid,signal.SIGINT);print('已停止本独立Pose转发；未停止定位')
elif action=='status':print(state.read_text() if state.exists() else '尚未启动')
else:raise RuntimeError('start/status/stop')
