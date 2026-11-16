from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([Node(package='track_test', executable='Safina_track_test', output='screen',
        parameters=[os.path.join(get_package_share_directory('track_test'),'config','track.yaml')])])
