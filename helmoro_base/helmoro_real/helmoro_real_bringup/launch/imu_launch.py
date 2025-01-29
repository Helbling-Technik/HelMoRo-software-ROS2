#!/usr/bin/env python3

# Launch Gazebo Ros Bridge

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Directories
    pkg_helmoro_real_bringup = get_package_share_directory("helmoro_real_bringup")

    # Paths
    bno055_config = PathJoinSubstitution(
        [pkg_helmoro_real_bringup, "config", "bno055_params_i2c.yaml"]
    )

    # ROS2 bno055 IMU Node
    bno055 = Node(package="bno055", executable="bno055", parameters=[bno055_config])

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(bno055)
    return ld
