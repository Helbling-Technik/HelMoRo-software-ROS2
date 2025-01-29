from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        choices=["true", "false"],
        description="Start rviz.",
    ),
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Use sim time.",
    ),
]


def generate_launch_description():

    # Directories
    pkg_helmoro_common = get_package_share_directory("helmoro_common_bringup")
    pkg_helmoro_state_estimation = get_package_share_directory(
        "helmoro_state_estimation"
    )
    pkg_helmoro_slam = get_package_share_directory("helmoro_slam")
    pkg_helmoro_navigation = get_package_share_directory("helmoro_navigation")
    pkg_helmoro_description = get_package_share_directory("helmoro_description")

    # Paths
    launch_slam = PathJoinSubstitution([pkg_helmoro_slam, "launch", "slam_launch.py"])
    launch_state_estimation = PathJoinSubstitution(
        [pkg_helmoro_state_estimation, "launch", "helmoro_state_estimation_launch.py"]
    )
    robot_description_launch_file = PathJoinSubstitution(
        [pkg_helmoro_description, "launch", "helmoro_description_launch.py"]
    )
    controller_params = PathJoinSubstitution(
        [
            FindPackageShare("helmoro_control"),
            "config",
            "helmoro_controller.yaml",
        ]
    )

    twist_mux_params = PathJoinSubstitution(
        [
            FindPackageShare("helmoro_common_bringup"),
            "config",
            "twist_mux.yaml",
        ]
    )

    # Launch Description
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_launch_file]),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "diff_drive_controller",
        ],
        output="screen",
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_diff_drive_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_diff_drive_controller],
        )
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_helmoro_common, "launch", "rviz_launch.py"])]
        ),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_slam]),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
    )

    state_estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_state_estimation]),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[
            twist_mux_params,
            {
                "topics.nav2.topic": "cmd_vel",
                "topics.nav2.priority": 10,
                "topics.nav2.timeout": 0.12,
                "topics.joy.topic": "helmoro_joy_control/cmd_vel",
                "topics.joy.priority": 20,
                "topics.joy.timeout": 0.12,
            },
        ],
        output="screen",
        remappings=[("/cmd_vel_out", "/twist_mux/cmd_vel")],
    )

    cmd_vel_stamped = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "topic_tools",
            "relay_field",
            "/twist_mux/cmd_vel",
            "/diff_drive_controller/cmd_vel",
            "geometry_msgs/msg/TwistStamped",
            "{twist: m}",
            "--wait-for-start",
        ],
        output="screen",
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_description)
    ld.add_action(rviz)
    ld.add_action(slam)
    ld.add_action(control_node)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(delay_diff_drive_controller_after_joint_state_broadcaster)
    ld.add_action(state_estimation)
    ld.add_action(twist_mux)
    ld.add_action(cmd_vel_stamped)
    return ld
