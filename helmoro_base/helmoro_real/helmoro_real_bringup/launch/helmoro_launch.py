#!/usr/bin/env python3

# Launch Gazebo Ros Bridge

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    pkg_helmoro_real_bringup = get_package_share_directory('helmoro_real_bringup')
    pkg_helmoro_motors = get_package_share_directory('helmoro_motors')
    pkg_helmoro_description = get_package_share_directory('helmoro_description')
    pkg_helmoro_common = get_package_share_directory('helmoro_common_bringup')
    pkg_helmoro_navigation = get_package_share_directory('helmoro_navigation')

    # Paths
    imu_launch = PathJoinSubstitution([pkg_helmoro_real_bringup, 'launch', 'imu_launch.py'])
    lidar_launch = PathJoinSubstitution([pkg_helmoro_real_bringup, 'launch', 'lidar_launch.py'])
    motor_controller_launch = PathJoinSubstitution([pkg_helmoro_motors, 'launch', 'helmoro_motors.launch.py'])
    helmoro_description_launch = PathJoinSubstitution([pkg_helmoro_description, 'launch', 'helmoro_description_launch.py'])
    helmoro_common_launch = PathJoinSubstitution([pkg_helmoro_common, 'launch', 'common_launch.py'])
    navigation_launch = PathJoinSubstitution(
        [pkg_helmoro_navigation, 'launch', 'nav2_launch.py'])


    # Launch Sensors
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([imu_launch]),
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lidar_launch]),
    )
    
    # Motor Controllers and Motor Command
    motor_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([motor_controller_launch]),
    )


    helmoro_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([helmoro_common_launch]),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_launch]),
        launch_arguments=[
            ('use_sim_time', 'true')]   
    )

    transform_msg = ExecuteProcess(
        cmd=['ros2', 'run', 'topic_tools', 'relay', '/motors/odom', '/ekf_filter_node/wheel_odom', '--wait-for-start'],
        output='screen'
    )

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(imu)
    ld.add_action(lidar)
    ld.add_action(motor_controllers)
    ld.add_action(helmoro_common)
    ld.add_action(navigation)
    ld.add_action(transform_msg)

    return ld