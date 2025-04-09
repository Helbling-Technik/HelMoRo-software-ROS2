import time
import unittest
import pytest
import rclpy
import std_msgs.msg
import sensor_msgs.msg

import launch_testing.markers

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node

from launch_testing.actions import ReadyToTest
from launch_testing.io_handler import ActiveIoHandler


NAMESPACE = 'test_robot'

# Generate the launch description for testing the robot_state_publisher
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Generate a LaunchDescription for the test."""
    # Define the package and launch file path
    pkg_helmoro_control = get_package_share_directory('helmoro_control')
    control_launch = PathJoinSubstitution([pkg_helmoro_control, 'launch', 'control.launch.py'])

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([control_launch]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('namespace', NAMESPACE)
        ]   
    )
    
    # Action to confirm when the test is ready
    ready_to_test = TimerAction(period=0.5, actions=[ReadyToTest()])
    
     # Create and return the complete launch description
    ld = LaunchDescription()
    ld.add_action(ready_to_test)
    ld.add_action(control)
    return ld

class TestJointStateBroadcaster(unittest.TestCase):
    """Test suite for checking the joint_state_broadcaster functionality."""

    def setUp(self):
        """Initialize the ROS node before each test."""
        rclpy.init()
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        """Shut down the ROS node after each test."""
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_start(self, proc_output: ActiveIoHandler):
        """Test if the joint_state_broadcaster node has started."""
        found = False
        print('Waiting for node...')
        start = time.time()
        
        # Wait for the node to start up and become available
        while time.time() - start < 10.0 and not found:
            found = 'spawner_joint_state_broadcaster' in self.node.get_node_names()
            time.sleep(0.1)

        # Assert that the node was found
        assert found, 'Node not found!'
