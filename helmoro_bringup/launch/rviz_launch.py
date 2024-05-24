
# Launch RViz.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the 'helmoro_bringup' package
    helmoro_bringup = get_package_share_directory('helmoro_bringup')

    # Define the path to the RViz configuration file
    rviz_config = PathJoinSubstitution([helmoro_bringup, 'rviz', 'rviz_config.rviz'])

    # Define the RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '--display-config', rviz_config,
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the RViz node to the launch description
    ld.add_action(rviz)

    return ld