from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


ARGUMENTS = [
    DeclareLaunchArgument('run_in_simulation', default_value='false',
                          choices=['true', 'false'],
                          description='run_in_simulation'),
    DeclareLaunchArgument('robot_name', default_value='id_23',
                          description='Robot name'),
    DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('robot_name'),
                          description='Robot namespace'),
]


def generate_launch_description():
    pkg_helmoro_description = get_package_share_directory('helmoro_description')
    xacro_file = PathJoinSubstitution([pkg_helmoro_description,
                                       'urdf',
                                       'helmoro.urdf.xacro'])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('run_in_simulation')},
            {'robot_description': ParameterValue(
                Command([
                    'xacro', ' ', xacro_file, ' ',
                    'run_in_simulation:=', LaunchConfiguration('run_in_simulation'), ' ',
                    'namespace:=', LaunchConfiguration('namespace')
                ]), value_type=str)},
        ]
    )
    
    # Workaround until Gazebo Ionic upgrades to sdformat 1.12 and allows <frame_id> tags
    static_transforms_publisher = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_broadcaster_imu',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1',  
                       PathJoinSubstitution([LaunchConfiguration('namespace'), 'imu']), 
                       PathJoinSubstitution([LaunchConfiguration('namespace'), LaunchConfiguration('namespace'), 'base_link/imu_sensor'])]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_broadcaster_lidar',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', 
                       PathJoinSubstitution([LaunchConfiguration('namespace'), 'lidar']), 
                       PathJoinSubstitution([LaunchConfiguration('namespace'), LaunchConfiguration('namespace'), 'base_link/rplidar'])]
        )
    ])


    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('run_in_simulation')}]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    
    # Add nodes to LaunchDescription
    ld.add_action(robot_state_publisher)
    ld.add_action(static_transforms_publisher)
    ld.add_action(joint_state_publisher)
    return ld