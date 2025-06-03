import time
import unittest


import pytest
import rclpy
import std_msgs.msg


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_testing.actions import ReadyToTest
import launch_testing.markers
from launch_testing.io_handler import ActiveIoHandler
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from helmoro_utils.test_collection import wait_for_node, wait_for_topic, wait_for_message

NAMESPACE = "robot_namespace"


# Generate the launch description for testing the robot_state_publisher
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Generate a LaunchDescription for the test."""
    # Define the package and launch file path
    pkg_helmoro_description = get_package_share_directory("helmoro_description")
    description_launch = PathJoinSubstitution(
        [pkg_helmoro_description, "launch", "description.launch.py"]
    )

    # Include the launch description with arguments
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([description_launch]),
        launch_arguments=[("run_in_simulation", "true"), ("namespace", NAMESPACE)],
    )

    # Action to confirm when the test is ready
    ready_to_test = TimerAction(period=0.5, actions=[ReadyToTest()])

    # Create and return the complete launch description
    ld = LaunchDescription()
    ld.add_action(description)
    ld.add_action(ready_to_test)
    return ld


class TestCollection(unittest.TestCase):
    """Test suite for checking the robot_state_publisher functionality."""

    def setUp(self):
        """Initialize the ROS node before each test."""
        rclpy.init()
        self.node = rclpy.create_node("test_node")

    def tearDown(self):
        """Shut down the ROS node after each test."""
        self.node.destroy_node()
        rclpy.shutdown()

    def test_robot_state_publisher_node_start(self, proc_output: ActiveIoHandler):
        """Test if the robot_state_publisher node has started."""
        wait_for_node(self.node, "robot_state_publisher", timeout=2.0)

    def test_robot_state_publisher_advertise_topic(self, proc_output: ActiveIoHandler):
        """Test if the robot_description topic is advertised by the node."""
        wait_for_topic(
            self.node,
            "/" + NAMESPACE + "/robot_description",
            timeout=10.0
        )

    def test_robot_state_publisher_publish_msgs(self, proc_output: ActiveIoHandler):
        """Test if messages are published to the correct topic.""" 
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        wait_for_message(
            self.node,
            "/" + NAMESPACE + "/robot_description",
            std_msgs.msg.String,
            timeout=3.0,
            qos_profile=qos_profile
        )

    def test_joint_state_publisher_node_start(self, proc_output: ActiveIoHandler):
        """Test if the joint_state_publisher node has started."""
        wait_for_node(self.node, "joint_state_publisher", timeout=2.0)

    def test_joint_state_publisher_advertise_topic(self, proc_output: ActiveIoHandler):
        """Test if the joint_state topic is advertised by the node."""
        wait_for_topic(
            self.node,
            "/" + NAMESPACE + "/joint_states",
            timeout=10.0
        )
