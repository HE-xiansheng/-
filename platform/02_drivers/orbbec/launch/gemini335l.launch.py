from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap


def generate_launch_description():
    config = Path(__file__).resolve().parents[1] / 'config/gemini335l.yaml'
    upstream = Path(get_package_share_directory('orbbec_camera')) / 'launch/gemini_330_series.launch.py'
    return LaunchDescription([GroupAction([
        # Same remap as ROS1. With colored cloud disabled it need not publish.
        SetRemap(src='/camera/depth/color/points', dst='/camera/depth_registered/points'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(upstream)),
                                 launch_arguments={'config_file_path': str(config)}.items()),
    ])])
