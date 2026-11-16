"""PX4 telemetry validation; deliberately no command/setpoint plugins."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    module = Path(__file__).resolve().parents[1]
    mavros = Path(get_package_share_directory('mavros'))
    return LaunchDescription([IncludeLaunchDescription(
        AnyLaunchDescriptionSource(str(mavros / 'launch/node.launch')),
        launch_arguments={
            'fcu_url': '/dev/serial/by-id/usb-Matek_HKUST_UAV_NxtPX4_0-if00:57600',
            'gcs_url': '', 'tgt_system': '1', 'tgt_component': '1',
            'fcu_protocol': 'v2.0', 'namespace': '',
            'pluginlists_yaml': str(module / 'config/telemetry_plugins.yaml'),
            'config_yaml': str(mavros / 'launch/px4_config.yaml'),
        }.items())])
