#!/usr/bin/env python3

# Launch Gazebo Ros Bridge

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    pkg_helmoro_gazebo_tools = get_package_share_directory('helmoro_gazebo_tools')

    # ROS Gazebo Bridge Node
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_helmoro_gazebo_tools, 'config', 'ros_gazebo_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(bridge)
    return ld