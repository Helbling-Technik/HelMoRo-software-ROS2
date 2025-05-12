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
        found = False
        print("Waiting for node...")
        start = time.time()

        # Wait for the node to start up and become available
        while time.time() - start < 10.0 and not found:
            found = "robot_state_publisher" in self.node.get_node_names()
            time.sleep(0.1)

        # Assert that the node was found
        assert found, "Node not found!"

    def test_robot_state_publisher_advertise_topic(self, proc_output: ActiveIoHandler):
        """Test if the robot_description topic is advertised by the node."""
        received = False
        print("Listening for topics...")
        start = time.time()

        # Wait for the topic to be advertised by the robot_state_publisher node
        while time.time() - start < 10.0 and not received:
            # Check if the node is publishing messages
            topic_names = self.node.get_topic_names_and_types()
            for topic_name, types in topic_names:
                if topic_name == "/" + NAMESPACE + "/robot_description":
                    received = True
                    break
            time.sleep(0.1)

        # Assert that the topic was advertised
        assert received, "Topic not advertised!"

    def test_robot_state_publisher_publish_msgs(self, proc_output: ActiveIoHandler):
        """Test if messages are published to the correct topic."""
        msgs_rx = []  # List to store received messages

        # Set up QoS profile with 'transient local' durability settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Create a subscription to the robot_description topic
        sub = self.node.create_subscription(
            std_msgs.msg.String,
            "/" + NAMESPACE + "/robot_description",
            lambda msg: msgs_rx.append(msg),
            qos_profile,
        )

        try:
            # Wait for messages to be received on the topic for up to 10 seconds
            end_time = time.time() + 10
            while time.time() < end_time and len(msgs_rx) == 0:
                # Spin once to execute the subscriber callback
                rclpy.spin_once(self.node, timeout_sec=1)

            # Assert that at least one message has been received
            assert (
                len(msgs_rx) > 0
            ), "No messages received on the robot_description topic!"

        finally:
            # Ensure that the subscription is destroyed after the test
            self.node.destroy_subscription(sub)

    def test_joint_state_publisher_node_start(self, proc_output: ActiveIoHandler):
        """Test if the joint_state_publisher node has started."""
        found = False
        print("Waiting for node...")
        start = time.time()

        # Wait for the node to start up and become available
        while time.time() - start < 10.0 and not found:
            found = "joint_state_publisher" in self.node.get_node_names()
            time.sleep(0.1)

        # Assert that the node was found
        assert found, "Node not found!"

    def test_joint_state_publisher_advertise_topic(self, proc_output: ActiveIoHandler):
        """Test if the joint_state topic is advertised by the node."""
        received = False
        print("Listening for topics...")
        start = time.time()

        # Wait for the topic to be advertised by the robot_state_publisher node
        while time.time() - start < 10.0 and not received:
            # Check if the node is publishing messages
            topic_names = self.node.get_topic_names_and_types()
            for topic_name, types in topic_names:
                if topic_name == "/" + NAMESPACE + "/joint_states":
                    received = True
                    break
            time.sleep(0.1)

        # Assert that the topic was advertised
        assert received, "Topic not advertised!"
