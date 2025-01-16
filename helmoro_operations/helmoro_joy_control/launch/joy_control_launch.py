#!/usr/bin/env python3

# Launch helmoro joy manager node

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]

def generate_launch_description():
  pkg_helmoro_joy_control = get_package_share_directory('helmoro_joy_control')

  print(pkg_helmoro_joy_control)
  param_yaml_file = PathJoinSubstitution(
        [pkg_helmoro_joy_control, 'param', 'parameters.yaml'])

  # ROS 2 joy node
  joy_node = Node(
      package='joy',
      name='joy_node',
      executable='joy_node',
      parameters=[{
          'deadzone': 0.2,
          'autorepeat_rate': 20.0,
      }],
  )

  # HelMoRo joy manager Node
  helmoro_joy_control_node = Node(
      package='helmoro_joy_control',
      name='helmoro_joy_control',
      executable='helmoro_joymanager',
      parameters=[param_yaml_file,
                  {'use_sim_time': True}],
      output='screen',
      remappings=[('/helmoro_joy_control/cmd_vel', '/cmd_vel')]
  )

  # Create the launch description and populate
  ld = LaunchDescription(ARGUMENTS)

  # Add the helmoro joy manager node to the launch description
  ld.add_action(PushRosNamespace('helmoro_joy_control'))
  ld.add_action(joy_node)
  ld.add_action(helmoro_joy_control_node)

  return ld