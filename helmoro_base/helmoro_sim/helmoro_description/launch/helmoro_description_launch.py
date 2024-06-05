#!/usr/bin/env python3

# Launch HelMoRo 3 state publishers.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]


def generate_launch_description():
    # Directroies
    pkg_helmoro_description = get_package_share_directory('helmoro_description')

    # Paths
    xacro_file = PathJoinSubstitution([pkg_helmoro_description, 'urdf', 'helmoro.urdf'])

    # Launch Configuration
    namespace = LaunchConfiguration('namespace')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description':
             Command(
                  ['xacro', ' ', xacro_file, ' ',
                   'namespace:=', namespace])},
        ],
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)

    # Add nodes to LaunchDescription
    ld.add_action(robot_state_publisher)

    return ld