from launch_ros.actions import SetParameter, Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

import os

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='undefined_namespace',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'], description='Use sim time.'),
]

def generate_launch_description():    
    # Paths
    filter_config = PathJoinSubstitution(
        [
            FindPackageShare('helmoro_state_estimation'), 
            'config',
            'fuse_params.yaml'
        ]
    )
    
    fuse_optimizer = Node(
          package='fuse_optimizers',
          executable='fixed_lag_smoother_node',
          name='state_estimation',
          namespace=LaunchConfiguration('namespace'),
          parameters=[
            filter_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
      )
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(fuse_optimizer)
    return ld
    