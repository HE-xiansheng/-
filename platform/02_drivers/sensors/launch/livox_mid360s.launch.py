"""Start the native ROS2 MID360s driver after checking the configured host IP."""
import json
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_driver(context):
    path = LaunchConfiguration('device_config').perform(context)
    with open(path, encoding='utf-8') as stream:
        config = json.load(stream)
    interfaces = json.loads(subprocess.check_output(['ip', '-j', 'address'], text=True))
    local_ips = {address['local'] for interface in interfaces
                 for address in interface.get('addr_info', [])}
    hosts = config['Mid360s']['host_net_info']
    missing = [host['host_ip'] for host in hosts if host['host_ip'] not in local_ips]
    if missing:
        raise RuntimeError('MID360s host IP is not assigned on this machine: '
                           + ', '.join(missing)
                           + '. Configure a nonconflicting lidar interface and device_config first.')
    return [Node(
        package='livox_ros_driver2', executable='livox_ros_driver2_node',
        name='livox_lidar_publisher2', output='screen',
        parameters=[{'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
                     'publish_freq': 10.0, 'output_data_type': 0,
                     'frame_id': 'livox_frame', 'user_config_path': path,
                     'cmdline_input_bd_code': 'ARMCP1V0038817',
                     'lvx_file_path': '', 'enable_lidar_bag': False,
                     'enable_imu_bag': False}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_config',
            default_value='/home/orangepi/drone_platform/02_drivers/sensors/config/MID360s_config.json'),
        OpaqueFunction(function=start_driver),
    ])
