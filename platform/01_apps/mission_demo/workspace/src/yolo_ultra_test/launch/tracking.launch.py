from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share = get_package_share_directory('yolo_ultra_test')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'yolo_track.yaml')),
        Node(package='yolo_ultra_test', executable='Safina_yoloUltra_test',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
