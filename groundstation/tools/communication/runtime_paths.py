"""ROS2试验版使用独立本机状态目录，不能接管原地面站的桥接进程。"""
import os
import sys
import threading
from pathlib import Path


def state_directory():
    return Path(os.environ.get('GCS_STATE_DIR') or (Path(os.environ.get('XDG_DATA_HOME') or (Path.home()/'.local/share'))/'drone-gcs-ros2'))


def watch_gui_owner(stopped, stop):
    """Linux 窗口退出后结束其只读桥接；直接运行 CLI 时保留原生命周期。"""
    value = os.environ.get('GCS_GUI_OWNER_PID')
    if sys.platform != 'linux' or not value:
        return None
    pid = int(value)
    if pid <= 1:
        raise ValueError('地面站所属进程 PID 无效')

    def identity():
        try:
            fields = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')', 1)[1].split()
            return fields[19] if fields[0] != 'Z' else None
        except (OSError, IndexError):
            return None

    expected = identity()
    if expected is None:
        stop()
        return None

    def watch():
        while not stopped.wait(.25):
            if identity() != expected:
                print('地面站所属窗口已退出，结束本会话只读遥测', file=sys.stderr, flush=True)
                stop()
                return

    thread = threading.Thread(target=watch, daemon=True, name='gcs-owner')
    thread.start()
    return thread
