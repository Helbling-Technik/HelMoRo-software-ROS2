from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    ),
]

def generate_launch_description():

    pkg_helmoro_visualization = get_package_share_directory('helmoro_visualization')
    pkg_helmoro_description = get_package_share_directory('helmoro_description')

    rviz2_config = PathJoinSubstitution(
        [pkg_helmoro_visualization, 'rviz', 'model.rviz'])
    description_launch = PathJoinSubstitution(
        [pkg_helmoro_description, 'launch', 'description.launch.py']
    )

    namespace = LaunchConfiguration('namespace')

    rviz = GroupAction([
        PushRosNamespace(namespace),

        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz2_config],
             remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
             ],
             output='screen'),

        # Delay launch of robot description to allow Rviz2 to load first.
        # Prevents visual bugs in the model.
        TimerAction(
            period=1.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([description_launch])
                )])
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rviz)
    return ld