import unittest
import rclpy
import time
import pytest

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription 
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_testing.actions import ReadyToTest
import launch_testing.markers

from rosgraph_msgs.msg import Clock


ARGUMENTS = [
    ("world", "depot"),
    ("headless", "true")
]

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Generate a LaunchDescription for the test."""
    pkg_simulation = get_package_share_directory("helmoro_simulation")
    path_simulation = PathJoinSubstitution([pkg_simulation, "launch", "launch.py"])

    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([path_simulation]),
        launch_arguments=ARGUMENTS
    )

    ld = LaunchDescription()
    ld.add_action(launch_simulation)
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

    def test_clock_bridge_start(self):
        """Test if the clock_bridge node started"""      
        assert "clock_bridge" in self.node.get_node_names(), "clock_bridge node not found!"
                
    def test_publishes_clock(self, proc_output):
        """Check whether clock messages are published"""
        msgs_rx = []
        sub = self.node.create_subscription(
            Clock, '/clock',
            lambda msg: msgs_rx.append(msg), 1)
        try:
            end_time = time.time() + 10
            while time.time() < end_time and len(msgs_rx) < 1:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            assert len(msgs_rx) > 0, "No clock messages received"
        finally:
            self.node.destroy_subscription(sub)
