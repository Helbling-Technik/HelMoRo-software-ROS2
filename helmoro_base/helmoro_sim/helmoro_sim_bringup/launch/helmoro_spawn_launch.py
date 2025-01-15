#!/usr/bin/env python3

# Launch Helmoro in Gazebo and optionally also in RViz.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    if pose_element != 'z':
        ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                        description=f'{pose_element} component of the robot pose.'))
    else:
        ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.1',
                        description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_helmoro_description = get_package_share_directory('helmoro_description')
    pkg_helmoro_common = get_package_share_directory('helmoro_common_bringup')
    pkg_helmoro_navigation = get_package_share_directory('helmoro_navigation')

    # Paths
    robot_description_launch_file = PathJoinSubstitution(
        [pkg_helmoro_description, 'launch', 'helmoro_description_launch.py'])
    
    helmoro_common_launch = PathJoinSubstitution(
        [pkg_helmoro_common, 'launch', 'common_launch.py'])
    navigation_launch = PathJoinSubstitution(
        [pkg_helmoro_navigation, 'launch', 'nav2_launch.py'])

    # Helmoro robot model and description
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_launch_file])  ,
        launch_arguments=[('use_sim_time', 'true')]   
    )

    # Spawn HelMoRo
    spawn_helmoro = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'helmoro',
                    '-x', LaunchConfiguration('x'),
                    '-y', LaunchConfiguration('y'),
                    '-z', LaunchConfiguration('z'),
                    '-Y', LaunchConfiguration('yaw'),
                    '-topic', 'robot_description'],
        output='screen',
    )

    helmoro_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([helmoro_common_launch]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('use_rviz', LaunchConfiguration('use_rviz'))
            ]   
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_launch]),
        launch_arguments=[('use_sim_time', 'true')]   
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    #ld.add_action(robot_description)
    ld.add_action(spawn_helmoro)
    ld.add_action(helmoro_common)
    ld.add_action(navigation)
    return ld