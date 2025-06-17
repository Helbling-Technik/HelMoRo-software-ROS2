import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile
        
def publish_twist_stamped(
    node: Node,
    topic_name: str,
    x_vel: float,
    rot_vel: float,
    duration: float = 1.0,
    frequency: int = 10,
    qos_profile: QoSProfile = QoSProfile(depth=10)
):
    """
    Publishes a TwistStamped message at a specified frequency for a set duration.

    This function is non-blocking: it uses a ROS 2 timer to periodically publish
    the message without blocking the main event loop.

    Args:
        node (Node): The ROS 2 node that owns the publisher and timer.
        topic_name (str): The name of the topic to publish to (e.g., '/robot/cmd_vel').
        x_vel (float): Linear velocity in the x-direction (forward/backward).
        rot_vel (float): Angular velocity around the z-axis (yaw).
        duration (float): Duration (in seconds) to publish the message.
        frequency (int, optional): Publishing rate in Hz. Defaults to 10.
        qos_profile (QoSProfile, optional): QoS profile for the publisher. Defaults to QoSProfile(depth=10).

    Example:
        publish_twist_stamped(node, '/robot/cmd_vel', 1.0, 0.2, duration=2.0)
    """
    pub = node.create_publisher(TwistStamped, topic_name, qos_profile)

    # Prepare the constant part of the message
    msg = TwistStamped()
    msg.header.frame_id = 'base_link'
    msg.twist.linear.x = x_vel
    msg.twist.angular.z = rot_vel

    # Timer setup
    count = 0
    max_count = int(frequency * duration)
    period = 1.0 / frequency

    def timer_callback():
        nonlocal count
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        count += 1
        if count >= max_count:
            timer.cancel()

    timer = node.create_timer(period, timer_callback)