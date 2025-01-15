
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    # Directories
    helmoro_state_estimation_pkg = get_package_share_directory('helmoro_state_estimation')
    topic_tools_pkg = get_package_share_directory('topic_tools')

    # Files
    state_estimation_params = PathJoinSubstitution([helmoro_state_estimation_pkg, 'config', 'ekf_params.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    transform_msg = ExecuteProcess(
        cmd=['ros2', 'run', 'topic_tools', 'relay_field', '/diff_drive_controller/cmd_vel_out', '/ekf_filter_node/cmd_vel_in',
             'geometry_msgs/msg/Twist', '{linear: m.twist.linear, angular: m.twist.angular}'],
        output='screen'
    )

    load_nodes = GroupAction(
        actions=[
            # PushRosNamespace('helmoro_state_estimation'),
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[state_estimation_params, {'use_sim_time': use_sim_time}],
                remappings=[('/helmoro_state_estimation/tf', 'tf'), 
                            ('/helmoro_state_estimation/tf_static', 'tf_static'),
                            ('/odometry/filtered', 'odom'),
                            ('/cmd_vel', '/ekf_filter_node/cmd_vel_in')],
            ),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(load_nodes)
    ld.add_action(transform_msg)

    return ld