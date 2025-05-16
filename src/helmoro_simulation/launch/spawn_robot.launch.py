from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        "namespace", default_value="empty_namespace", description="Robot namespace"
    ),
    DeclareLaunchArgument("x", default_value="0.0", description="x position"),
    DeclareLaunchArgument("y", default_value="0.0", description="y position"),
    DeclareLaunchArgument("z", default_value="0.0", description="z position"),
    DeclareLaunchArgument("yaw", default_value="0.0", description="yaw rotation"),
]


def generate_launch_description():

    spawn_robot = Node(
        package="ros_gz_sim",
        namespace=LaunchConfiguration("namespace"),
        name="create",
        executable="create",
        arguments=[
            "-name",
            LaunchConfiguration("namespace"),
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-Y",
            LaunchConfiguration("yaw"),
            "-topic",
            "robot_description",
        ],
        output="screen",
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_robot)
    return ld
