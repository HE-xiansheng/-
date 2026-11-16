"""ROS2试验版使用独立本机状态目录，不能接管原地面站的桥接进程。"""
import os
from pathlib import Path


def state_directory():
    return Path(os.environ.get('GCS_STATE_DIR') or (Path(os.environ.get('XDG_DATA_HOME') or (Path.home()/'.local/share'))/'drone-gcs-ros2'))
