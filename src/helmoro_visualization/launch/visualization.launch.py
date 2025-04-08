from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='robot1',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'], description='Use sim time.'),
    DeclareLaunchArgument('rviz_config', default_value='debug.rviz',
                          description='Config preset')
]

def generate_launch_description():

    pkg_helmoro_visualization = get_package_share_directory('helmoro_visualization')

    rviz2_config = PathJoinSubstitution(
        [pkg_helmoro_visualization, 'rviz', LaunchConfiguration('rviz_config')])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=LaunchConfiguration('namespace'),
        arguments=['-d', rviz2_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        remappings = [
        ('/tf', 'tf'), 
        ('/tf_static', 'tf_static')
    ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rviz)
    return ld