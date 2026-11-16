"""Full PX4 interface candidate; telemetry-only services are unchanged."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mavros = Path(get_package_share_directory('mavros')) / 'launch'
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='/dev/serial/by-id/usb-Matek_HKUST_UAV_NxtPX4_0-if00:57600'),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('tgt_system', default_value='1'),
        DeclareLaunchArgument('tgt_component', default_value='1'),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(str(mavros / 'node.launch')),
            launch_arguments={
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'tgt_system': LaunchConfiguration('tgt_system'),
                'tgt_component': LaunchConfiguration('tgt_component'),
                'namespace': '', 'fcu_protocol': 'v2.0',
                'pluginlists_yaml': str(mavros / 'px4_pluginlists.yaml'),
                'config_yaml': str(mavros / 'px4_config.yaml'),
            }.items()),
    ])
