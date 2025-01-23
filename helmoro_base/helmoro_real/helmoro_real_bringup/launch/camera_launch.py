import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import PushRosNamespace, SetRemap

def generate_launch_description():
    # Directories
    pkg_orbbec_camera = get_package_share_directory('orbbec_camera')

    # Paths
    camera_launch = PathJoinSubstitution([pkg_orbbec_camera, 'launch', 'astra.launch.py'])

    # ROS2 RPLidar A2M8 Node
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([camera_launch])
    )

    camera_node_with_namespace = GroupAction(
     actions=[
        PushRosNamespace('sensors'),
        camera_node,
      ]

   )

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(camera_node_with_namespace)
    return ld