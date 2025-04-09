from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'], description='Use sim time.'),
    DeclareLaunchArgument('namespace', default_value='example_robot_name',
                          description='Robot namespace'),
]

def generate_launch_description():
    # Launch Description   
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=LaunchConfiguration('namespace'),
        arguments=['joint_state_broadcaster',
                   '-c', 'controller_manager',
                   '--switch-timeout', '30.0']
    )
    
    diff_drive_base_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace=LaunchConfiguration('namespace'),
        arguments=['diff_drive_controller',
                   '-c', 'controller_manager',
                   '--switch-timeout', '30.0',
                   "--controller-ros-args",
                    "-r ~/cmd_vel:=cmd_vel_stamped"]
    )
    
    cmd_vel_stamper = Node(
        package='topic_tools',
        executable='relay_field',
        name='cmdvel_to_stamped',
        namespace=LaunchConfiguration('namespace'),
        arguments=[
            'cmd_vel',  # input topic
            'cmd_vel_stamped',  # output topic
            'geometry_msgs/msg/TwistStamped',  # output type
            "{header: {frame_id: 'base_link'}, twist: m}",  # transformation
            '--wait-for-start'
        ],
        output='screen'
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
    ld.add_action(joint_state_broadcaster)
    ld.add_action(start_diff_drive_controller_after_joint_state_broadcaster)
    ld.add_action(cmd_vel_stamper)
    return ld

    