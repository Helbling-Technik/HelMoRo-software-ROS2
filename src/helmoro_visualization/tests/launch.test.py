import time
import unittest
import rclpy

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_testing.actions import ReadyToTest

import pytest

namespace = "test_robot"
ARGUMENTS = [
    ("use_sim_time", "true"),
    ("namespace", namespace),
    ("rviz_config", "debug.rviz"),
]


@pytest.mark.launch_test
def generate_test_description():
    """Generate a LaunchDescription for the test."""
    visualization_dir = get_package_share_directory("helmoro_visualization")
    visualization_path = PathJoinSubstitution(
        [visualization_dir, "launch", "launch.py"]
    )

    # Include the launch description with arguments
    launch_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([visualization_path]),
        launch_arguments=ARGUMENTS,
    )

    ld = LaunchDescription()
    ld.add_action(launch_visualization)
    ld.add_action(ReadyToTest())

    return ld


class TestProcess(unittest.TestCase):

    def setUp(self):
        """Initialize the ROS node before each test."""
        rclpy.init()
        self.node = rclpy.create_node("test_node")

    def tearDown(self):
        """Shut down the ROS node after each test."""
        self.node.destroy_node()
        rclpy.shutdown()

    def test_namespace(self):
        nodes_and_namespaces = self.node.get_node_names_and_namespaces()
        rviz_ns = None

        node_found = False
        namespace_found = False
        start = time.time()

        # Look for node
        while time.time() - start < 5.0 and not namespace_found:
            for name, ns in nodes_and_namespaces:
                if name == "rviz2":
                    node_found = True
                    rviz_ns
                    if ns == f"/{namespace}":
                        namespace_found = True
                        break

        # If test fails, give debug output
        if not namespace_found:
            print("Printing all found nodes...")
            for name, ns in nodes_and_namespaces:
                print("Nodename: ", name, " namespace: ", namespace)

        assert node_found, "rviz2 node not found!"
        assert namespace_found, f"Expected namespace '/{namespace}', got '{rviz_ns}'"
