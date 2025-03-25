from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='default_namespace',
                          description='Robot namespace'),
    DeclareLaunchArgument('run_in_simulation', default_value='false',
                          choices=['true', 'false'], description='Use sim time.'),
    DeclareLaunchArgument('use_rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
]

def generate_launch_description():

    # Directories
    pkg_helmoro_description = get_package_share_directory('helmoro_description')
    pkg_helmoro_state_estimation = get_package_share_directory('helmoro_state_estimation')
    pkg_helmoro_control = get_package_share_directory('helmoro_control')
    
    # Paths
    description_launch = PathJoinSubstitution(
        [pkg_helmoro_description, 'launch', 'description.launch.py']
    )
    state_estimation_launch = PathJoinSubstitution(
        [pkg_helmoro_state_estimation, 'launch', 'estimation.launch.py']
    )
    control_launch = PathJoinSubstitution(
        [pkg_helmoro_control, 'launch', 'control.launch.py']
    )

    # Launch Description    
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([description_launch]),
        launch_arguments=[
            ('run_in_simulation', LaunchConfiguration('run_in_simulation')),
            ('namespace', LaunchConfiguration('namespace'))
        ]   
    )
    
    state_estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([state_estimation_launch]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('run_in_simulation')),
            ('namespace', LaunchConfiguration('namespace'))
        ]
    )
    
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([control_launch]),
        launch_arguments=[
            ('run_in_simulation', LaunchConfiguration('run_in_simulation')),
            ('namespace', LaunchConfiguration('namespace'))
        ]   
    )
    
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(description)
    ld.add_action(state_estimation)
    ld.add_action(control)
    return ld
