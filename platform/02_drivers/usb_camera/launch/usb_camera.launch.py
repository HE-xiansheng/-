"""Optional independent USB camera; never selects Gemini or HDMI implicitly."""
import os
from pathlib import Path
import stat
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def validate_device(value):
    supplied = Path(value)
    try:
        device = supplied.resolve(strict=True)
    except FileNotFoundError:
        raise RuntimeError('Independent USB camera not connected: ' + value)
    if not stat.S_ISCHR(device.stat().st_mode):
        raise RuntimeError('Not a character video device: ' + str(device))
    sysfs = Path('/sys/class/video4linux') / device.name / 'device'
    if not sysfs.exists():
        raise RuntimeError('Not a V4L2 device: ' + str(device))
    for parent in [sysfs.resolve()] + list(sysfs.resolve().parents):
        vendor = parent / 'idVendor'
        if vendor.is_file():
            if vendor.read_text().strip().lower() == '2bc5':
                raise RuntimeError('Orbbec camera belongs to the Gemini driver, not independent USB capture')
            if not os.access(device, os.R_OK | os.W_OK):
                raise RuntimeError('USB video device is not readable/writable: ' + str(device))
            return str(device)
    raise RuntimeError('Device is not a USB camera (HDMI/platform input is excluded): ' + str(device))


def start(context):
    device = validate_device(LaunchConfiguration('video_device').perform(context))
    module = Path(__file__).resolve().parents[1]
    camera = Node(package='usb_cam', executable='usb_cam_node_exe', name='usb_cam', namespace='usb_cam',
        output='screen', parameters=[str(module/'config/usb_cam.yaml'), {'video_device': device}])
    return [RegisterEventHandler(OnProcessExit(target_action=camera,
        on_exit=[EmitEvent(event=Shutdown(reason='Independent USB camera exited'))])), camera,
        ExecuteProcess(cmd=[sys.executable, str(module/'capture_compat.py')], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/usb_cam'),
        OpaqueFunction(function=start),
    ])
