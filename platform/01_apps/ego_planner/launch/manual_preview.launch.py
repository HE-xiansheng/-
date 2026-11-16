"""Manual start only. Data topics remain explicit launch arguments."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(__file__).resolve().parent.parent / 'config'
    return LaunchDescription([
        DeclareLaunchArgument('odom', default_value='/factory_fastlio/odometry'),
        DeclareLaunchArgument('cloud', default_value='/factory_fastlio/cloud_registered'),
        Node(package='plan_manage', executable='planner_node', name='ego_planner_node',
             output='screen', parameters=[str(config / 'planner.yaml'), str(config / 'manual_preview.yaml')],
             remappings=[('odom_world', LaunchConfiguration('odom')),
                         ('grid_map/odom', LaunchConfiguration('odom')),
                         ('grid_map/cloud', LaunchConfiguration('cloud'))]),
    ])
