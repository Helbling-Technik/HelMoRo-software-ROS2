# This is a slightly modified version of the official online_async_launch.py in the slam_toolbox package.

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution, TextSubstitution)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'
    ),
    DeclareLaunchArgument('namespace', 
        default_value=TextSubstitution(text='undefined_namespace'),
        description='Robot namespace'
    )
]

def generate_launch_description():
    # Declare Launch Arguments
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')

    # Launch Configuration
    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
          LaunchConfiguration('slam_params_file'),
          {
            'use_lifecycle_manager': LaunchConfiguration("use_lifecycle_manager"),
            'use_sim_time': LaunchConfiguration('use_sim_time')
          }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=LaunchConfiguration('namespace'),
        remappings=[
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
        ]
    )

    configure_event = EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
          transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(LaunchConfiguration('autostart'), NotSubstitution(LaunchConfiguration("use_lifecycle_manager"))))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(LaunchConfiguration('autostart'), NotSubstitution(LaunchConfiguration("use_lifecycle_manager"))))
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld