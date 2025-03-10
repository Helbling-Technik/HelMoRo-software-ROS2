import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch_ros.actions import Node, PushRosNamespace

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='default_namespace',
                          description='Robot namespace'),
    DeclareLaunchArgument('run_in_simulation', default_value='true',
                          choices=['true', 'false'], description='Use sim time.'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='empty',
                          description='Simulation World')
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))
    
def generate_launch_description():
    # Directories
    pkg_helmoro_bringup = get_package_share_directory(
        'helmoro_bringup')
    pkg_helmoro_simulator = get_package_share_directory(
        'helmoro_simulator')
    pkg_helmoro_visualization = get_package_share_directory(
        'helmoro_visualization')
    
    # Paths
    common_launch = PathJoinSubstitution(
        [pkg_helmoro_bringup, 'launch', 'common.launch.py'])
    gazebo_launch = PathJoinSubstitution(
        [pkg_helmoro_simulator, 'launch', 'simulator.launch.py'])
    visualization_launch = PathJoinSubstitution(
        [pkg_helmoro_visualization, 'launch', 'visualization.launch.py'])

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([common_launch]),
        launch_arguments=[
            ('run_in_simulation', LaunchConfiguration('run_in_simulation')),
            ('namespace', LaunchConfiguration('namespace'))
        ] 
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    spawn_helmoro_group_action = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        
        # Spawn Helmoro
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', LaunchConfiguration('namespace'),
                       '-x', LaunchConfiguration('x'),
                       '-y', LaunchConfiguration('y'),
                       '-z', LaunchConfiguration('z'),
                       '-Y', LaunchConfiguration('yaw'),
                       '-topic', 'robot_description'],
            output='screen'
        ),
        
        # ROS GZ bridge        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': os.path.join(pkg_helmoro_bringup, 'config', 'ros_gz_bridge.yaml'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }],
            arguments=[
                '--ros-args', '--remap', 'expand_gz_topic_names:=true'
            ],
            output='screen'
        )
    ])
    
    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([visualization_launch])
    )
    
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(common)
    ld.add_action(spawn_helmoro_group_action)
    ld.add_action(visualization)
    return ld
