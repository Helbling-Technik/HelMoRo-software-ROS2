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
    helmoro_navigation_pkg = get_package_share_directory('helmoro_navigation')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    # Files
    nav2_navigation_launch = PathJoinSubstitution(
        [nav2_bringup_pkg, 'launch', 'navigation_launch.py'])
    nav2_params = PathJoinSubstitution(
        [helmoro_navigation_pkg, 'config', 'nav2_params.yaml'])


    nav2_nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_navigation_launch]),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'True', 
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
            'namespace': 'helmoro_navigation'
        }.items()
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Add localization stack
    ld.add_action(nav2_nav_stack)

    return ld