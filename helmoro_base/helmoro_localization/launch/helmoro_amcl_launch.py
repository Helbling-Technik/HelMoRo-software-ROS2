#!/usr/bin/env python3

# Launch Localization Package

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Get the launch directory
    helmoro_localization_pkg = get_package_share_directory('helmoro_localization')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    # Files
    nav2_amcl_launch = PathJoinSubstitution(
        [nav2_bringup_pkg, 'launch', 'localization_launch.py'])
    nav2_slam_launch = PathJoinSubstitution(
        [nav2_bringup_pkg, 'launch', 'slam_launch.py'])
    nav2_params = PathJoinSubstitution(
        [helmoro_localization_pkg, 'config', 'nav2_params.yaml'])
    map_yaml_file = PathJoinSubstitution(
        [helmoro_localization_pkg, 'maps', 'empty_world.yaml'])

    nav2_localization_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_amcl_launch]),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': nav2_params,
            'use_sim_time': 'True', 
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
            'container_name': 'nav2_container'
        }.items()
    )

    # nav2_localization_stack = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([nav2_slam_launch]),
    #     launch_arguments={
    #         'params_file': nav2_params,
    #         'use_sim_time': 'True', 
    #         'autostart': 'True',
    #         'use_respawn': 'False',
    #     }.items()
    # )
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Add localization stack
    ld.add_action(nav2_localization_stack)

    return ld