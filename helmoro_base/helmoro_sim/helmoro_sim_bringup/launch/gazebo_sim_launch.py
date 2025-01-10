#!/usr/bin/env python3

# Launch Helmoro in Gazebo Fortess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument('use_rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='empty',
                          description='Ignition World'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

def generate_launch_description():

    z_pos = LaunchConfiguration('z')

    # Directories
    pkg_helmoro_sim_bringup = get_package_share_directory('helmoro_sim_bringup')
    pkg_helmoro_state_estimation = get_package_share_directory('helmoro_state_estimation')
    pkg_helmoro_slam = get_package_share_directory('helmoro_slam')
    pkg_helmoro_navigation = get_package_share_directory('helmoro_navigation')

    # Paths
    gazebo_launch = PathJoinSubstitution(
        [pkg_helmoro_sim_bringup, 'launch', 'gazebo_launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_helmoro_sim_bringup, 'launch', 'helmoro_spawn_launch.py'])
    slam_launch = PathJoinSubstitution(
        [pkg_helmoro_slam, 'launch', 'slam_launch.py'])
    state_estimation_launch = PathJoinSubstitution(
        [pkg_helmoro_state_estimation, 'launch', 'helmoro_state_estimation_launch.py'])
    navigation_launch = PathJoinSubstitution(
        [pkg_helmoro_navigation, 'launch', 'nav2_launch.py'])

    # Launch Descriptions
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('use_rviz', LaunchConfiguration('use_rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', str(0.1)),
            ('yaw', LaunchConfiguration('yaw'))])
    
    # SLAM (responsilby for map->odom tf and map generation)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_launch]),
        launch_arguments=[
            ('use_sim_time', 'true')]    
    )

    state_estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([state_estimation_launch]),
        launch_arguments=[
            ('use_sim_time', 'true')]    
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_launch]),
        launch_arguments=[
            ('use_sim_time', 'true')]   
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(robot_spawn)
    ld.add_action(slam)
    ld.add_action(state_estimation)
    ld.add_action(navigation)
    return ld