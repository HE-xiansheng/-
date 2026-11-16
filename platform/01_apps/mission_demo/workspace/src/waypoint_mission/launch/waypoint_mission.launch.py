from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = LaunchConfiguration
    pkg = get_package_share_directory('waypoint_mission')
    args = [DeclareLaunchArgument(k, default_value=v) for k,v in [
        ('mission_file',os.path.join(pkg,'missions','abc_photo.yaml')),('dry_run','false'),
        ('bench_mode','false'),('start_camera','false'),('start_photo','true'),
        ('odom_topic','/factory_fastlio/odometry'),('photo_dir','/tmp/waypoint_mission/photos'),
        ('log_dir','/tmp/waypoint_mission/logs')]]
    return LaunchDescription(args + [
        Node(package='waypoint_mission', executable='waypoint_mission', output='screen',
             parameters=[{'mission_file':cfg('mission_file'),
                          'dry_run':ParameterValue(cfg('dry_run'),value_type=bool),
                          'bench_mode':ParameterValue(cfg('bench_mode'),value_type=bool),
                          'photo_dir':cfg('photo_dir'),'log_dir':cfg('log_dir')}],
             remappings=[('/Odometry',cfg('odom_topic'))]),
        Node(package='waypoint_mission', executable='waypoint_photo', output='screen',
             parameters=[{'image_topic':'/rtsp_cam/image_color'}], condition=IfCondition(cfg('start_photo'))),
        Node(package='rtsp_test', executable='rtsp_hw_pub_ultra', output='screen',
             condition=IfCondition(cfg('start_camera')))
    ])
