import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    pkg_rplidar_ros = get_package_share_directory('rplidar_ros')

    # Paths
    rplidar_launch = PathJoinSubstitution([pkg_rplidar_ros, 'launch', 'rplidar_a2m8_launch.py'])

    # ROS2 RPLidar A2M8 Node
    rplidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rplidar_launch])
    )

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(rplidar_node)
    return ld