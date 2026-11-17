"""整套ROS2传感器候选；所有选中设备先预检，再共同启动。"""
import json
import os
from pathlib import Path
import stat
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    drivers=Path(__file__).resolve().parents[2]
    selected={}
    for name in ['mavros','camera','lidar','rtsp']:
        value=LaunchConfiguration('enable_'+name).perform(context).lower()
        if value not in ['true','false']:
            raise RuntimeError('enable_'+name+' must be true or false')
        selected[name]=value=='true'
    if not any(selected.values()):
        raise RuntimeError('Select at least one sensor module')
    fcu=LaunchConfiguration('fcu_url').perform(context)
    if selected['mavros']:
        get_package_share_directory('mavros')
        if fcu.startswith('/dev/'):
            device=Path(fcu.split(':',1)[0])
            if not device.exists() or not stat.S_ISCHR(device.stat().st_mode) or not os.access(device,os.R_OK|os.W_OK):
                raise RuntimeError('PX4 serial device missing or inaccessible: '+str(device))
    if selected['camera']:
        get_package_share_directory('orbbec_camera')
    config=LaunchConfiguration('device_config').perform(context)
    if selected['lidar']:
        get_package_share_directory('livox_ros_driver2')
        with open(config,encoding='utf-8') as stream:device=json.load(stream)
        interfaces=json.loads(subprocess.check_output(['ip','-j','address'],text=True))
        local={a['local'] for interface in interfaces for a in interface.get('addr_info',[])}
        missing=[h['host_ip'] for h in device['Mid360s']['host_net_info'] if h['host_ip'] not in local]
        if missing:raise RuntimeError('MID360s host IP is not assigned: '+', '.join(missing))
    if selected['rtsp']:
        get_package_share_directory('rtsp_test')

    def include(path,**args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),launch_arguments=args.items())

    def unexpected_exit(event,launch_context):
        if launch_context.is_shutdown:return []
        raise RuntimeError('Sensor process exited unexpectedly (code %s): %s' % (event.returncode,event.action))

    actions=[RegisterEventHandler(OnProcessExit(on_exit=unexpected_exit))]
    if selected['mavros']:
        actions.append(include(drivers/'sensors/launch/mavros_platform.launch.py',fcu_url=fcu))
    if selected['camera']:
        actions.append(include(drivers/'orbbec/launch/gemini335l.launch.py'))
    if selected['lidar']:
        actions.append(include(drivers/'sensors/launch/livox_mid360s.launch.py',device_config=config))
    if selected['rtsp']:
        actions.append(Node(package='rtsp_test',executable='rtsp_hw_pub_ultra',name='rtsp_hw_pub_ultra',
            output='screen',parameters=[{'rtsp_url':LaunchConfiguration('rtsp_url').perform(context)}]))
    return actions


def generate_launch_description():
    defaults={**{'enable_'+name:'true' for name in ['mavros','camera','lidar','rtsp']},
        'fcu_url':'/dev/serial/by-id/usb-Matek_HKUST_UAV_NxtPX4_0-if00:57600',
        'device_config':'/home/orangepi/drone_platform/02_drivers/sensors/config/MID360s_config.json',
        'rtsp_url':'rtsp://192.168.144.108'}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+
        [OpaqueFunction(function=start)])
