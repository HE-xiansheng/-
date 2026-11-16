"""RTSP capture and ArUco detection only; no landing or flight control node."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rtsp_url', default_value='rtsp://192.168.144.108'),
        Node(package='rtsp_test', executable='rtsp_hw_pub_ultra', output='screen',
             parameters=[{'rtsp_url': LaunchConfiguration('rtsp_url')}]),
        Node(package='aruco_test', executable='Safina_aruco_test', output='screen',
             parameters=[str(Path(get_package_share_directory('aruco_test')) / 'config/aruco.yaml')]),
    ])
