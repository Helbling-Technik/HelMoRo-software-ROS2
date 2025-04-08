from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import SetRemap

from nav2_common.launch import RewrittenYaml
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
    pkg_helmoro_slam = get_package_share_directory('helmoro_slam')
    
    # Paths
    slam_toolbox_launch = PathJoinSubstitution(
        [pkg_helmoro_slam, 'launch', 'online_async_launch.py']
    )
    
    # Rewrite the SLAM Toolbox configuration file    
    params = RewrittenYaml(
        source_file=PathJoinSubstitution([pkg_helmoro_slam, 'config', 'slam_toolbox.yaml']),
        root_key=LaunchConfiguration('namespace'),
        param_rewrites={
                'odom_frame': PathJoinSubstitution([LaunchConfiguration('namespace'), 'odom']),
                'map_frame': PathJoinSubstitution([LaunchConfiguration('namespace'), 'map']),
                'base_frame': PathJoinSubstitution([LaunchConfiguration('namespace'), 'base_link']),
                'scan_topic': 'sensor/lidar/scan'
            },
        convert_types=True
    )
    
    # Descriptions        
    slam = GroupAction(
        actions=[
            # Remapping required to have a slam session subscribe & publish in optional namespaces
            # TODO: Implement a solution to not require Remaps here, but use instead parameters from the params file
            # I.e. currently the param scan_topic doesn't do anything 
            #SetRemap(src='/scan', dst='scan'),
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            SetRemap(src='/map', dst='map'),
            SetRemap(src='/map_metadata', dst='map_metadata'),
            

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([slam_toolbox_launch]),
                launch_arguments=[
                    ('use_sim_time', LaunchConfiguration('use_sim_time')),
                    ('slam_params_file', params),
                    ('namespace', LaunchConfiguration('namespace'))
                ]
            )
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(slam)
    return ld