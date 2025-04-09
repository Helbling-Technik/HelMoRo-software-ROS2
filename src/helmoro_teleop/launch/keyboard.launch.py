from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='undefined_namespace',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'], description='Use sim time.')
]

def generate_launch_description():
    teleop =  Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        prefix='xterm -e',  # This opens the node in a new terminal window
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'stamped': True}],
        remappings=[
            ('cmd_vel', 'cmd_vel_stamped')
        ]
    )
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(teleop)
    return ld
