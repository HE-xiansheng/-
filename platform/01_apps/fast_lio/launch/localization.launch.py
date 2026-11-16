"""Localization only; no pose forwarding or flight controller commands."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='fast_lio', executable='fastlio_mapping', name='factory_laserMapping',
            output='screen',
            parameters=['/home/orangepi/drone_platform/01_apps/fast_lio/config/lidar.yaml',
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('/Odometry', '/factory_fastlio/odometry'),
                        ('/cloud_registered_body', '/factory_fastlio/cloud_body'),
                        ('/cloud_registered', '/factory_fastlio/cloud_world'),
                        ('/cloud_effected', '/factory_fastlio/cloud_effected'),
                        ('/Laser_map', '/factory_fastlio/map'),
                        ('/path', '/factory_fastlio/path'),
                        ('/tf', '/factory_fastlio/tf')],
        ),
    ])
