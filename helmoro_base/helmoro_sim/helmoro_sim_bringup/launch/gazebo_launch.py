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

ARGUMENTS = [
    DeclareLaunchArgument('use_gazebo_gui', default_value='true',
                          choices=['true', 'false'],
                          description='Set "false" to run gazebo headless.'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='Gazebo Fortress World'),
]


# # Rviz requires US locale to correctly display the wheels
# os.environ['LC_NUMERIC'] = 'en_US.UTF-8'


def generate_launch_description():
    # Directories
    pkg_helmoro_sim_bringup = get_package_share_directory('helmoro_sim_bringup')
    pkg_helmoro_description = get_package_share_directory('helmoro_description')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    # Set gazebo resource path
    # gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
    #                                             EnvironmentVariable('GAZEBO_MODEL_PATH',
    #                                                                 default_value=''),
    #                                             '/usr/share/gazebo-11/models/:',
    #                                             str(Path(pkg_helmoro_description).
    #                                                 parent.resolve())])
    gz_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',
                                            value=[os.path.join(
                                                    pkg_helmoro_sim_bringup,
                                                    'worlds'), ':' +
                                                    str(Path(
                                                        pkg_helmoro_description).
                                                        parent.resolve())])

    # Set GAZEBO_MODEL_URI to empty string to prevent Gazebo from downloading models
    # gz_model_uri = SetEnvironmentVariable(name='GAZEBO_MODEL_URI', value=[''])


    # Launch configurations
    world_path = LaunchConfiguration('world_path')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')


    # Gazebo
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('ign_args', [LaunchConfiguration('world'),
                          '.sdf',
                          ' -v 4',])
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Gazebo processes
    ld.add_action(gz_resource_path)
    # ld.add_action(gz_model_uri)
    ld.add_action(gazebo)

    return ld