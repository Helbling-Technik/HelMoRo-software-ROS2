from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import SetRemap, PushROSNamespace

from nav2_common.launch import RewrittenYaml, ReplaceString
from launch.launch_description_sources import PythonLaunchDescriptionSource

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('namespace', default_value=TextSubstitution(text='undefined_namespace'),
                          description='Robot namespace')
]

def generate_launch_description():
    # Directories
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_helmoro_navigation = get_package_share_directory('helmoro_navigation')
    
    # Paths
    navigation_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py']
    )
    
    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # in config file 'nav2_multirobot_params.yaml' as a default & example.
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    params_file = ReplaceString(
        source_file=PathJoinSubstitution([pkg_helmoro_navigation, 'params', 'nav2_params.yaml']),
        replacements={'<robot_namespace>': ('/', LaunchConfiguration('namespace'))},
    )
        
    # Descriptions        
    nav2 = GroupAction(
        actions=[
            PushROSNamespace(LaunchConfiguration('namespace')),  
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'), 
            SetRemap(src='/map', dst='map'),      

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={
                    'namespace': LaunchConfiguration('namespace'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': 'true',
                    'params_file': params_file,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                    'container_name': 'nav2_container',
                }.items(),
            ),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2)
    return ld