from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    module = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(module / 'launch/mavros_telemetry.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(module.parent / 'orbbec/launch/gemini335l.launch.py'))),
    ])
