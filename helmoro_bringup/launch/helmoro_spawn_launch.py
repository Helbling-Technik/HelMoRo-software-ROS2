#!/usr/bin/env python3

# Launch Helmoro in Gazebo and optionally also in RViz.

import os

from ament_index_python.packages import get_package_share_directory

from helmoro_bringup.namespace import GetNamespacedName

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
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


# Rviz requires US locale to correctly display the wheels
# os.environ['LC_NUMERIC'] = 'en_US.UTF-8'


def generate_launch_description():
    # Directories
    pkg_helmoro_bringup = get_package_share_directory('helmoro_bringup')

    # Paths
    # create3_nodes_launch_file = PathJoinSubstitution(
    #     [pkg_create3_common_bringup, 'launch', 'create3_nodes.launch.py'])  //ToDo
    robot_description_launch_file = PathJoinSubstitution(
        [pkg_helmoro_bringup, 'launch', 'helmoro_description_launch.py'])
    rviz_launch_file = PathJoinSubstitution(
        [pkg_helmoro_bringup, 'launch', 'rviz_launch.py'])

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_name = GetNamespacedName(namespace, 'helmoro_blue')

    spawn_robot_group_action = GroupAction([
        PushRosNamespace(namespace),

        # Helmoro robot model and description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch_file]),
            launch_arguments={'gazebo': 'classic'}.items(),
        ),

        # helmoro spawn
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_helmoro',
            arguments=['-entity',
                       robot_name,
                       '-topic',
                       'robot_description',
                       '-x', x,
                       '-y', y,
                       '-z', z,
                       '-Y', yaw],
            output='screen',
        ),

        # Create 3 nodes //ToDo
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([create3_nodes_launch_file]),
        #     launch_arguments=[
        #         ('namespace', namespace)
        #     ]
        # ),

        # RVIZ2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rviz_launch_file]),
            condition=IfCondition(use_rviz),
        ),
    ])

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_robot_group_action)
    return ld