#!/usr/bin/env python3

# Launch HelMoRo 3 state publishers.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]


def generate_launch_description():
    # Directroies
    pkg_helmoro_description = get_package_share_directory('helmoro_description')
    pkg_helmoro_sim_controller = get_package_share_directory('helmoro_sim_control')

    # Paths
    xacro_file = PathJoinSubstitution([pkg_helmoro_description, 'urdf', 'helmoro.urdf'])
    robot_controller = PathJoinSubstitution([pkg_helmoro_sim_controller, 'config', 'helmoro_controller.yaml'])

    # Launch Configuration
    namespace = LaunchConfiguration('namespace')

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description':
             Command(
                  ['xacro', ' ', xacro_file, ' '])},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_diff_drive_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_diff_drive_controller],
        )
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )
  
    nodes = [
        bridge,
        robot_state_pub_node,
        load_diff_drive_controller,
        delay_diff_drive_controller_after_joint_state_broadcaster,
    ]

    return LaunchDescription(nodes)