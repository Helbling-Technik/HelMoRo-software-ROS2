import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

from pathlib import Path


ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='empty',
                          choices=['depot', 'empty', 'lake_house'],
                          description='Simulation World')
]

def generate_launch_description():
    # Paths
    pkg_helmoro_simulator = get_package_share_directory(
        'helmoro_simulator')
    
    pkg_helmoro_description = get_package_share_directory(
        'helmoro_description')
    
    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim')
    
    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    
    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_helmoro_simulator, 'worlds'),
            str(Path(pkg_helmoro_description).parent.resolve())
        ])
    )
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [
                LaunchConfiguration('world'),
                '.sdf',
                ' -r',
                ' -v 2',
                ' --gui-config ',
                PathJoinSubstitution([
                    pkg_helmoro_simulator,
                    'gui',
                    'gui.config'
                ])
            ])
        ]
    )
    
    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock'
                        ])
    
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(gazebo)
    ld.add_action(clock_bridge)
    return ld