#!/usr/bin/env python3

# Launch Motor Commands Node

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]

def generate_launch_description():
  pkg_helmoro_motor_commands = get_package_share_directory('helmoro_motor_commands')


  param_yaml_file = PathJoinSubstitution(
        [pkg_helmoro_motor_commands, 'param', 'parameters.yaml'])

  # HelMoRo Motor Commands Node
  motor_commands_node = Node(
      package='helmoro_motor_commands',
      name='helmoro_motor_commands',
      executable='helmoro_motor_commands',
      parameters=[param_yaml_file,
                  {'use_sim_time': True}],
      output='screen',
  )

  # Create the launch description and populate
  ld = LaunchDescription(ARGUMENTS)

  # Add the helmoro motor commands node to the launch description
  ld.add_action(motor_commands_node)

  return ld