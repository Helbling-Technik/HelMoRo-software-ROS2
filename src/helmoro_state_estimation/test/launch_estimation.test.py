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
    pkg_helmoro_state_estimation = get_package_share_directory('helmoro_state_estimation')
    state_estimation_launch = PathJoinSubstitution([pkg_helmoro_state_estimation, 'launch', 'estimation.launch.py'])

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([state_estimation_launch]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('namespace', NAMESPACE)
        ]   
    )
    
    params_path = PathJoinSubstitution(
        [pkg_helmoro_state_estimation, 'test', 'rosbags', '2025-04-07-08-05-43', 'params.yaml']
    )
    rosbag_path = PathJoinSubstitution(
        [pkg_helmoro_state_estimation, 'test', 'rosbags', '2025-04-07-08-05-43', '2025-04-07-08-05-43_0.mcap']
    )
    rosbag_play = Node(
        package='rosbag2_transport',
        executable='player',
        name='player',
        output="screen",
        parameters=[
            params_path,
            {'storage.uri': rosbag_path}
        ],
    )
    
    # Action to confirm when the test is ready
    ready_to_test = TimerAction(period=0.5, actions=[ReadyToTest()])
    
     # Create and return the complete launch description
    ld = LaunchDescription()
    ld.add_action(rosbag_play)
    ld.add_action(ready_to_test)
    ld.add_action(control)
    return ld

class TestStateEstimation(unittest.TestCase):
    """Test suite for checking the state_estimation functionality."""

    def setUp(self):
        """Initialize the ROS node before each test."""
        rclpy.init()
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        """Shut down the ROS node after each test."""
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_start(self, proc_output: ActiveIoHandler):
        """Test if the state_estimation node has started."""
        found = False
        print('Waiting for node...')
        start = time.time()
        
        # Wait for the node to start up and become available
        while time.time() - start < 3.0 and not found:
            found = 'state_estimation' in self.node.get_node_names()
            time.sleep(0.1)

        # Assert that the node was found
        assert found, 'Node not found!'
        
    def test_advertise_topic_acceleration(self, proc_output: ActiveIoHandler):
        """Test if the acceleration/filtered topic is advertised by the node."""
        received = False
        print("Listening for topics...")
        start = time.time()
        
        # Wait for the topic to be advertised
        while time.time() - start < 3.0 and not received:
            # Check if the node is publishing messages
            topic_names = self.node.get_topic_names_and_types()
            for topic_name, types in topic_names:
                if topic_name == '/' + NAMESPACE + '/acceleration/filtered':
                    received = True
                    break
            time.sleep(0.1)
        
        # Assert that the topic was advertised 
        assert received, 'Topic not advertised!'

    def test_publish_acceleration(self, proc_output: ActiveIoHandler):
        """Test if messages are published to the correct topic."""
        msgs_rx = []  # List to store received messages
        
        # Create a subscription to the robot_description topic
        sub = self.node.create_subscription(
            std_msgs.msg.String,
            '/' + NAMESPACE + '/acceleration/filtered',
            lambda msg: msgs_rx.append(msg),
            10
        )
        
        try:
            # Wait for messages to be received on the topic for up to 10 seconds
            end_time = time.time() + 3
            while time.time() < end_time and len(msgs_rx) == 0:
                # Spin once to execute the subscriber callback
                rclpy.spin_once(self.node, timeout_sec=1)
                
             # Assert that at least one message has been received
            assert len(msgs_rx) > 0, "No messages received on the robot_description topic!"
            
        finally:
            # Ensure that the subscription is destroyed after the test
            self.node.destroy_subscription(sub)
      
    def test_advertise_topic_odom(self, proc_output: ActiveIoHandler):
      """Test if the acceleration/filtered topic is advertised by the node."""
      received = False
      print("Listening for topics...")
      start = time.time()
      
      # Wait for the topic to be advertised
      while time.time() - start < 3.0 and not received:
          # Check if the node is publishing messages
          topic_names = self.node.get_topic_names_and_types()
          for topic_name, types in topic_names:
              if topic_name == '/' + NAMESPACE + '/odom':
                  received = True
                  break
          time.sleep(0.1)
      
      # Assert that the topic was advertised 
      assert received, 'Topic not advertised!'

    def test_publish_odom(self, proc_output: ActiveIoHandler):
        """Test if messages are published to the correct topic."""
        msgs_rx = []  # List to store received messages
        
        # Create a subscription to the robot_description topic
        sub = self.node.create_subscription(
            std_msgs.msg.String,
            '/' + NAMESPACE + '/odom',
            lambda msg: msgs_rx.append(msg),
            10
        )
        
        try:
            # Wait for messages to be received on the topic for up to 10 seconds
            end_time = time.time() + 3
            while time.time() < end_time and len(msgs_rx) == 0:
                # Spin once to execute the subscriber callback
                rclpy.spin_once(self.node, timeout_sec=1)
                
              # Assert that at least one message has been received
            assert len(msgs_rx) > 0, "No messages received!"
            
        finally:
            # Ensure that the subscription is destroyed after the test
            self.node.destroy_subscription(sub)
