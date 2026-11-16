from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([Node(package='land_test', executable='Safina_land_test', output='screen',
        parameters=[os.path.join(get_package_share_directory('land_test'),'config','takeoff_land.yaml')])])
