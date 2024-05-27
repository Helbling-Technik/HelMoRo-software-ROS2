#!/usr/bin/env python3

# Launch HelMoRo 3 state publishers.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    # DeclareLaunchArgument('gazebo', default_value='classic',
    #                       choices=['classic', 'ignition'],
    #                       description='Which gazebo simulator to use'),
    # DeclareLaunchArgument('visualize_rays', default_value='false',
    #                       choices=['true', 'false'],
    #                       description='Enable/disable ray visualization'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]


def generate_launch_description():
    pkg_helmoro_description = get_package_share_directory('helmoro_description')
    xacro_file = PathJoinSubstitution([pkg_helmoro_description, 'urdf', 'helmoro.urdf'])
    gazebo_simulator = LaunchConfiguration('gazebo')
    visualize_rays = LaunchConfiguration('visualize_rays')
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
                #    'gazebo:=', gazebo_simulator, ' ',
                #    'visualize_rays:=', visualize_rays, ' ',
                   'namespace:=', namespace])},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)

    # Add nodes to LaunchDescription
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)

    return ld