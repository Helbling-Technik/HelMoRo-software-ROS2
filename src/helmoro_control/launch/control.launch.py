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
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'], description='Use sim time.'),
    DeclareLaunchArgument('namespace', default_value='example_robot_name',
                          description='Robot namespace'),
]

def generate_launch_description():
    # Paths
    controller_params = PathJoinSubstitution(
        [
            FindPackageShare('helmoro_control'),
            "config",
            "helmoro_controller.yaml",
        ]
    )

    # Launch Description   
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration('namespace'),
        parameters=[controller_params],
        output="both",
        remappings=[
            ("~/robot_description", "robot_description"),
            ("/diagnostics", "diagnostics")
        ]
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=LaunchConfiguration('namespace'),
        arguments=['joint_state_broadcaster'],
    )
    
    diff_drive_base_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace=LaunchConfiguration('namespace'),
        arguments=['diff_drive_controller']
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    start_diff_drive_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diff_drive_base_controller],
        )
    )


    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    #ld.add_action(control_node)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(start_diff_drive_controller_after_joint_state_broadcaster)
    return ld

    