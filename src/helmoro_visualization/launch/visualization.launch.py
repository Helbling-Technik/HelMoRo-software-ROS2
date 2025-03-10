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
        default_value='default_namespace',
        description='Robot namespace'
    ),
]

def generate_launch_description():

    pkg_helmoro_visualization = get_package_share_directory('helmoro_visualization')

    rviz2_config = PathJoinSubstitution(
        [pkg_helmoro_visualization, 'rviz', 'model.rviz'])

    rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             namespace=LaunchConfiguration('namespace'),
             arguments=['-d', rviz2_config],
             remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
             ],
             output='screen')

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rviz)
    return ld