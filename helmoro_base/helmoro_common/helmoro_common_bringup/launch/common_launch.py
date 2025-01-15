from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node 
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('use_rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'], description='Use sim time.'),
]

def generate_launch_description():

    # Directories
    pkg_helmoro_common = get_package_share_directory('helmoro_common_bringup')
    pkg_helmoro_state_estimation = get_package_share_directory('helmoro_state_estimation')
    pkg_helmoro_slam = get_package_share_directory('helmoro_slam')
    pkg_helmoro_navigation = get_package_share_directory('helmoro_navigation')
    pkg_helmoro_description = get_package_share_directory('helmoro_description')

    # Paths
    launch_slam = PathJoinSubstitution(
        [pkg_helmoro_slam, 'launch', 'slam_launch.py'])
    launch_state_estimation = PathJoinSubstitution(
        [pkg_helmoro_state_estimation, 'launch', 'helmoro_state_estimation_launch.py'])
    robot_description_launch_file = PathJoinSubstitution(
        [pkg_helmoro_description, 'launch', 'helmoro_description_launch.py'])
    controller_params = PathJoinSubstitution(
        [
            FindPackageShare('helmoro_control'),
            "config",
            "helmoro_controller.yaml",
        ]
    )

    # Launch Description    
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_launch_file]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time'))
        ]   
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_controller'],
        output='screen'
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_diff_drive_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_diff_drive_controller],
        )
    )

    
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [pkg_helmoro_common, 'launch', 'rviz_launch.py'])]),
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_slam]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time'))
        ]
    )

    state_estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_state_estimation]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time'))
        ]
    )


    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_description)
    ld.add_action(rviz)
    ld.add_action(state_estimation)
    ld.add_action(slam)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(delay_diff_drive_controller_after_joint_state_broadcaster)
    return ld

    