#!/usr/bin/env python3

# Launch Gazebo

import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='depot',
                          description='Gazebo Fortress World'),
]

def generate_launch_description():
    # Directories
    pkg_helmoro_gazebo_tools = get_package_share_directory('helmoro_gazebo_tools')
    pkg_helmoro_description = get_package_share_directory('helmoro_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Set gazebo resource path
    gz_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',
                                            value=[os.path.join(
                                                    pkg_helmoro_gazebo_tools,
                                                    'worlds'), ':' +
                                                    str(Path(
                                                        pkg_helmoro_description).
                                                        parent.resolve())])

    # Launch configurations
    world_path = LaunchConfiguration('world_path')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')


    # Gazebo
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'),
                          '.sdf',
                          ' -v 4',])
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Gazebo processes
    ld.add_action(gz_resource_path)
    ld.add_action(gazebo)

    return ld