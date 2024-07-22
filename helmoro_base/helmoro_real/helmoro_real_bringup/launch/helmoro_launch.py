#!/usr/bin/env python3

# Launch Gazebo Ros Bridge

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    pkg_helmoro_real_bringup = get_package_share_directory('helmoro_real_bringup')
    pkg_helmoro_motors = get_package_share_directory('helmoro_motors')
    pkg_helmoro_description = get_package_share_directory('helmoro_description')

    # Paths
    imu_launch = PathJoinSubstitution([pkg_helmoro_real_bringup, 'launch', 'imu_launch.py'])
    lidar_launch = PathJoinSubstitution([pkg_helmoro_real_bringup, 'launch', 'lidar_launch.py'])
    motor_controller_launch = PathJoinSubstitution([pkg_helmoro_motors, 'launch', 'helmoro_motors.launch.py'])
    xacro_file = PathJoinSubstitution([pkg_helmoro_description, 'urdf', 'helmoro.urdf'])

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

    #Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description':
             Command(
                  ['xacro ', xacro_file])},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(imu)
    ld.add_action(lidar)
    ld.add_action(motor_controllers)
    ld.add_action(robot_state_pub_node)
    return ld