import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='empty',
                          description='Simulation World'),
    DeclareLaunchArgument('headless', default_value='false',
                          choices=['false', 'False', 'true', 'True'],
                          description='Run the simulation headless')
]


def generate_launch_description():
    # Paths
    pkg_helmoro_simulation = get_package_share_directory('helmoro_simulation')
    pkg_helmoro_description = get_package_share_directory('helmoro_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim_launch = PathJoinSubstitution([
        pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'
    ])

    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_helmoro_simulation, 'worlds'),
            os.path.join(pkg_helmoro_simulation, 'models'),
            str(Path(pkg_helmoro_description).parent.resolve())
        ])
    )

    # Launch Gazebo headless or with GUI
    gz_args = [
        LaunchConfiguration('world'),
        '.sdf', 
        ' -r',
        ' -v 2'
    ]
    
    gazebo = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gz_sim_launch]),
            launch_arguments=[('gz_args', gz_args)],
            condition=UnlessCondition(LaunchConfiguration('headless')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gz_sim_launch]),
            launch_arguments=[('gz_args', gz_args + [' -s'])],
            condition=IfCondition(LaunchConfiguration('headless')),
        )
    ])

    # Clock bridge node
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    # Compose launch description
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(gazebo)
    ld.add_action(clock_bridge)
    return ld
