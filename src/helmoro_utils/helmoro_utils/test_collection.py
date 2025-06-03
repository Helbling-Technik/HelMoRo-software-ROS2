# helmoro_utils/test_collection.py

import time
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
import std_msgs.msg
import rosgraph_msgs.msg
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def wait_for_node(node, node_name: str, timeout: float = 10.0):
    """
    Waits until the specified node name appears in the ROS graph.

    Args:
        node: rclpy Node object used to query node names.
        node_name: Name of the node to look for.
        timeout: How long to wait before failing.
        poll_interval: How often to check.

    Raises:
        AssertionError: If the node is not found within the timeout.
    """
    found = False
    start = time.time()

    while time.time() - start < timeout:
        if node_name in node.get_node_names():
            found = True
            break
        time.sleep(0.1)

    assert found, f"Node '{node_name}' not found within {timeout} seconds. Nodes: {node.get_node_names()}"
    
def wait_for_node_with_namespace(node: Node, node_name: str, node_namespace: str, timeout: float = 10.0):
    """
    Waits until the specified node with the exact name and namespace appears in the ROS graph.

    Args:
        node: rclpy Node object used to query node names and namespaces.
        node_name: Name of the node to look for (e.g., 'my_node').
        node_namespace: Expected namespace of the node (e.g., '/robot1').
        timeout: Max time to wait for the node.

    Raises:
        AssertionError: If the node with the namespace is not found within the timeout.
    """
    found = False
    start = time.time()

    while time.time() - start < timeout:
        node_tuples = node.get_node_names_and_namespaces()  # List of (name, namespace)
        for name, namespace in node_tuples:
            if name == node_name and namespace == node_namespace:
                found = True
                break
        if found:
            break
        time.sleep(0.1)

    assert found, (
        f"Node '{node_name}' with namespace '{node_namespace}' not found within {timeout} seconds.\n"
        f"Available nodes: {node_tuples}"
    )

def wait_for_topic(node, topic_name: str, timeout: float = 10.0):
    """
    Waits until the specified topic is advertised in the ROS graph.

    Args:
        node: rclpy Node object used to query topics.
        topic_name: Name of the topic to look for.
        timeout: How long to wait before failing.
        poll_interval: How often to check.

    Raises:
        AssertionError: If the topic is not found within the timeout.
    """
    received = False
    start = time.time()

    while time.time() - start < timeout:
        topic_names = node.get_topic_names_and_types()
        if any(topic_name == name for name, _ in topic_names):
            received = True
            break
        time.sleep(0.1)

    assert received, f"Topic '{topic_name}' not advertised within {timeout} seconds. Topics: {topic_names}"
    
def wait_for_message(node: Node, topic_name: str, msg_type_str: str, timeout: float = 10.0, qos_profile: QoSProfile = 10):
    """Waits until a message is received on the topic within the timeout."""
    received_msg = []
    
    def callback(msg):
        received_msg.append(msg)
    
    sub = node.create_subscription(msg_type_str, topic_name, callback, qos_profile)

    start = time.time()
    while time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if received_msg:
            break

    node.destroy_subscription(sub)
    assert received_msg, f"No message received on {topic_name} within {timeout} seconds."
