#!/usr/bin/env python3

# Launch HelMoRo 3 state publishers.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    IncludeLaunchDescription,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    ),
]


def generate_launch_description():
    # Directroies
    pkg_helmoro_description = get_package_share_directory("helmoro_description")

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    ##  Launch Configuration
    # If Simulation
    robot_state_pub_node_sim = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        PathJoinSubstitution(
                            [pkg_helmoro_description, "urdf", "helmoro_sim.urdf"]
                        ),
                    ]
                )
            },
        ],
        remappings=remappings,
        condition=IfCondition(LaunchConfiguration("use_sim_time")),
    )

    # If Real
    robot_state_pub_node_real = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        PathJoinSubstitution(
                            [pkg_helmoro_description, "urdf", "helmoro_real.urdf"]
                        ),
                    ]
                )
            },
        ],
        remappings=remappings,
        condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
    )

    # Launch Description
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_pub_node_sim)
    ld.add_action(robot_state_pub_node_real)

    return ld
