#!/usr/bin/env python3

# Launch RViz.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import UnlessCondition
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'], description='Use sim time.'),
]

def generate_launch_description():
    # Get the path to the 'helmoro_bringup' package
    pkg_helmoro_common_bringup = get_package_share_directory('helmoro_common_bringup')

    # Define the path to the RViz configuration file
    rviz_config = PathJoinSubstitution([pkg_helmoro_common_bringup, 'rviz', 'rviz_config.rviz'])

    # Define the RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '--display-config', rviz_config,
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # Start republisher to display images from camera
    uncompress_color_image = ExecuteProcess(
        cmd=['ros2', 'run', 'image_transport', 'republish', 'compressed', 'raw', '--ros-args', 
             '--remap', 'in/compressed:=/sensors/camera/color/image_raw/compressed', 
             '--remap', 'out:=/sensors/camera/color/image_raw/uncompressed'],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    uncompress_depth = ExecuteProcess(
        cmd=['ros2', 'run', 'image_transport', 'republish', 'compressedDepth', 'raw', '--ros-args', 
             '--remap', 'in/compressedDepth:=/sensors/camera/depth/image_raw/compressedDepth', 
             '--remap', 'out:=/sensors/camera/depth/image_raw/uncompressed'],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    # Create the launch description and populate
    ld = LaunchDescription(ARGUMENTS)

    # Add the RViz node to the launch description
    ld.add_action(rviz)
    ld.add_action(uncompress_color_image)
    ld.add_action(uncompress_depth)

    return ld