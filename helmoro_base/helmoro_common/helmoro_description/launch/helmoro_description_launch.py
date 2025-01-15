#!/usr/bin/env python3

# Launch HelMoRo 3 state publishers.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false', 
                          description='Use simulation (Gazebo) clock if true'),
]


def generate_launch_description():
    # Directroies
    pkg_helmoro_description = get_package_share_directory('helmoro_description')
    
    # Paths
    xacro_file = PathJoinSubstitution([pkg_helmoro_description, 'urdf', 'helmoro.urdf'])
    
    # Launch Configuration
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description':
             Command(
                  ['xacro ', xacro_file])},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # Launch Description
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_pub_node)
    
    return ld