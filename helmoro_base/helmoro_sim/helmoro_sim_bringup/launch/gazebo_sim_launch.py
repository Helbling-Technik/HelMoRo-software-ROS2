#!/usr/bin/env python3

# Launch Helmoro in Gazebo Fortess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='empty',
                          description='Ignition World'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

def generate_launch_description():

    # Directories
    pkg_helmoro_sim_bringup = get_package_share_directory(
        'helmoro_sim_bringup')

    # Paths
    gazebo_launch = PathJoinSubstitution(
        [pkg_helmoro_sim_bringup, 'launch', 'gazebo_launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_helmoro_sim_bringup, 'launch', 'helmoro_spawn_launch.py'])

    # Launch Descriptions
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('use_rviz', LaunchConfiguration('use_rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(robot_spawn)
    return ld