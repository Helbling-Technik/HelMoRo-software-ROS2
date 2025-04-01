import launch
import launch.actions
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to your rqt configuration file
    rqt_config_file = PathJoinSubstitution([
        FindPackageShare('helmoro_visualization'),
        'rqt',
        'developer_view'
    ])

    # Launch rqt_gui with the saved configuration
    rqt_node = launch.actions.ExecuteProcess(
        cmd=['rqt', '--perspective-file', rqt_config_file],
        output='screen'
    )

    return launch.LaunchDescription([
        rqt_node
    ])
