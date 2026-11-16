from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap


def generate_launch_description():
    config = Path(__file__).resolve().parents[1] / 'config/gemini335l.yaml'
    upstream = Path(get_package_share_directory('orbbec_camera')) / 'launch/gemini_330_series.launch.py'
    actions = []
    # Scope the large-image transport to this driver and honor explicit profiles.
    if (os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp') == 'rmw_fastrtps_cpp'
            and not os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
            and not os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE')):
        profile = config.parent / 'fastdds_large_images.xml'
        if not profile.is_file():
            raise FileNotFoundError(profile)
        actions.append(SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', str(profile)))
    return LaunchDescription([GroupAction(actions + [
        # Same remap as ROS1. With colored cloud disabled it need not publish.
        SetRemap(src='/camera/depth/color/points', dst='/camera/depth_registered/points'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(upstream)),
                                 launch_arguments={'config_file_path': str(config)}.items()),
    ])])
