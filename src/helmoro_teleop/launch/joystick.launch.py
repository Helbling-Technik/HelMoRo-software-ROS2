from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='undefined_namespace',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'], description='Use sim time.'),
    DeclareLaunchArgument('joy_dev', default_value='0', 
                          description='the name of the controller in /dev'),
    DeclareLaunchArgument('controller_config', default_value='logitech_f710.yaml',
                          description='config file with controller parameters')
]

def generate_launch_description():
    joystick_config = PathJoinSubstitution(
        [
            FindPackageShare('helmoro_teleop'), 
            'config',
            LaunchConfiguration('controller_config')
        ]
    )
    
    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }])
    
    teleop_node = Node(
        package='teleop_twist_joy', executable='teleop_node',
        name='teleop_twist_joy_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            joystick_config, 
            {'publish_stamped_twist': True}
        ]
    )
        
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    return ld
