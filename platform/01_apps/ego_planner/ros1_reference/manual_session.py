#!/usr/bin/env python3
"""Own only this manual preview's roslaunch process; never touch flight services."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import fcntl

root = Path(__file__).resolve().parent
state = root / '.manual-preview.json'
lock = open(str(root / '.manual-preview.lock'), 'a')
fcntl.flock(lock, fcntl.LOCK_EX)
launch = str(root / 'src/ego_planner/plan_manage/launch/realflight/run_planner_board.launch')

def identity(pid):
    try:
        stat = Path('/proc/%d/stat' % pid).read_text().split(') ', 1)[1].split()
        cmd = Path('/proc/%d/cmdline' % pid).read_bytes().replace(b'\0', b' ').decode()
        return stat[19], cmd
    except (OSError, IndexError):
        return None, ''

def owned():
    try:
        info = json.loads(state.read_text())
        tick, cmd = identity(info['pid'])
        if tick == info['start_tick'] and launch in cmd and 'manual_preview:=true' in cmd:
            return info
    except (OSError, ValueError, KeyError):
        pass
    return None

arg = sys.argv[1] if len(sys.argv) > 1 else '--start'
info = owned()
def stop_recording(data):
    if data and data.get('recording'):
        try:
            health=json.loads((Path(data['recording'])/'recording-status.json').read_text())
            if health.get('policy')=='manual':return  # 手动录制只由录制按钮结束，不随规划停止。
        except (OSError,ValueError):pass
        (Path(data['recording'])/'stop-recording').touch()

if arg == '--status':
    if info and info.get('recording'):
        try:
            health=json.loads((Path(info['recording'])/'recording-status.json').read_text())
            health['age_s']=time.time()-health['stamp'];info['recorder_health']=health
        except (OSError,ValueError,KeyError):info['recorder_health']={'phase':'UNKNOWN'}
    print(json.dumps(info or {'running': False}, ensure_ascii=False)); sys.exit(0)
if arg == '--stop':
    if info:
        os.killpg(info['pid'], signal.SIGINT)
        for _ in range(100):
            if not owned(): break
            time.sleep(.1)
        if owned(): raise RuntimeError('规划预览未退出，请检查 ROS 日志')
        state.unlink(missing_ok=True)
    try: stop_recording(json.loads(state.read_text()))
    except (OSError,ValueError): pass
    stop_recording(info)
    print('手动预览已停止；未操作其他 ROS 节点。'); sys.exit(0)
if arg != '--start': raise RuntimeError('Use --start / --stop / --status')
if info:
    print('手动预览已运行: '+json.dumps(info)); sys.exit(0)
# Refuse to start beside an existing upstream/other planner session.
for entry in Path('/proc').iterdir():
    if entry.name.isdigit():
        _, cmd = identity(int(entry.name))
        if '/devel/lib/ego_planner/planner_node ' in cmd:
            raise RuntimeError('Existing EGO-Planner found; stop its verified launch first')
import rospy
from mavros_msgs.msg import State
rospy.init_node('manual_preview_start_check', anonymous=True, disable_signals=True)
flight = rospy.wait_for_message('/mavros/state', State, timeout=5)
if not flight.connected or flight.armed or flight.header.stamp.is_zero() or not -.1 <= (rospy.Time.now()-flight.header.stamp).to_sec() <= 2:
    raise RuntimeError('Need fresh connected, disarmed MAVROS state before starting preview')
import recording_manage
recording_info=recording_manage.prepare(root)
run=Path(recording_info['run'])
deadline=time.monotonic()+40
while time.monotonic()<deadline:
    health=recording_manage.health(root)
    if health.get('phase') in ('WAIT_MANUAL','STOPPED','RECORDING'):break
    if health.get('phase')=='ERROR':raise RuntimeError(str(health))
    if recording_manage.current(root) is None:raise RuntimeError('Recording preparation exited; inspect '+str(run))
    time.sleep(.1)
else:
    raise RuntimeError('Recording preparation not ready; planner was not started: '+str(run))
env=dict(os.environ,EGO_PLANNER_RECORD_DIR=str(run))
# 常规 ROS 消息由 ROS 日志与录制负责，不再单独复制预览控制台输出。
proc = subprocess.Popen(['roslaunch', launch, 'manual_preview:=true'], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True,env=env)
tick, _ = identity(proc.pid)
data = {'pid': proc.pid, 'start_tick': tick, 'running': True, 'recording':str(run), 'recorder_pid':recording_info['pid'], 'recording_policy':'manual'}
state.write_text(json.dumps(data))
time.sleep(2)
if proc.poll() is not None:
    stop_recording(data)
    state.unlink(missing_ok=True)
    raise RuntimeError('规划预览启动失败，请检查 ROS 启动日志')
from std_msgs.msg import String
try:
    status = json.loads(rospy.wait_for_message('/ego_planner/manual_preview/status', String, timeout=10).data)
    if status.get('mode') != 'manual_preview' or status.get('command_output') is not False:
        raise RuntimeError('Unexpected preview status')
except Exception:
    if owned(): os.killpg(proc.pid, signal.SIGINT)
    stop_recording(data)
    raise
try:
    subprocess.run(['rosparam','dump',str(run/'params-after.yaml')],timeout=15,check=True)
except Exception as exc:
    (run/'params-after-error.txt').write_text(str(exc))
print('手动预览启动，无轨迹执行服务器: '+json.dumps(data, ensure_ascii=False))
