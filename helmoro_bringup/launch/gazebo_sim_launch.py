#!/usr/bin/env python3

# Launch Helmoro in Gazebo

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
        DeclareLaunchArgument('use_gazebo_gui', default_value='true',
                          choices=['true', 'false'],
                          description='Set "false" to run gazebo headless.'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start RViz.'),
        DeclareLaunchArgument('world_path', default_value='',
                          description='Set world path, by default is empty.world'),
        DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]

# Robot Position
for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                    description=f'{pose_element} component of the robot pose.'))

def generate_launch_description():

    # Get the path to the 'helmoro_bringup' package
    pkg_helmoro_bringup = get_package_share_directory('helmoro_bringup')
    
    # Paths
    gazebo_launch = PathJoinSubstitution(
        [pkg_helmoro_bringup, 'launch', 'gazebo_launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_helmoro_bringup, 'launch', 'rviz_launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_helmoro_bringup, 'launch', 'helmoro_spawn_launch.py'])

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('world_path', LaunchConfiguration('world_path')),
            ('use_gazebo_gui', LaunchConfiguration('use_gazebo_gui'))
        ]
    )

    #Spawn HelMoRo
    robot_spawn = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([robot_spawn_launch]),
    launch_arguments=[
        ('namespace', LaunchConfiguration('namespace')),
        ('use_rviz', LaunchConfiguration('use_rviz')),
        ('x', LaunchConfiguration('x')),
        ('y', LaunchConfiguration('y')),
        ('z', LaunchConfiguration('z')),
        ('yaw', LaunchConfiguration('yaw'))])

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Gazebo
    ld.add_action(gazebo)
    # Robot spawn
    ld.add_action(robot_spawn)
    return ld