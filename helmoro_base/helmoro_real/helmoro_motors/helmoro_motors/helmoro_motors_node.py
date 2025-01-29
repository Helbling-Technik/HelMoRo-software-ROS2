import rclpy
import time
import math
from rclpy.node import Node

from helmoro_motors.robot_handler import RobotHandler

from geometry_msgs.msg import TwistStamped, Twist, Pose, Point
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
import numpy as np


class RosHandler(Node):
    def __init__(self):
        super().__init__("helmoro_motor_commands_node")

        # Parameters
        # Wheel Geometry
        self.dx_wheels = 0.115
        self.dy_wheels = 0.195
        self.dia_wheels = 0.09
        self.wheel_circumference = self.dia_wheels * math.pi
        # Velocity constraints
        self.max_lin_vel = 1.1  # m/s
        self.max_ang_vel = 10.5  # rad/s
        # Runtim
        self.frequency = 10.0

        # Variables
        self._v_i = 0.0  # Integration Error
        self._ki = 0.4  # Integration facto
        self.vx_cmd = 0.0
        self.yaw_cmd = 0.0
        self.omega_imu = 0.0  # Derivative of yaw
        self.wheel_pos = [0.0, 0.0, 0.0, 0.0]  # in meters
        self.wheel_vel = [0.0, 0.0, 0.0, 0.0]  # in meters per second
        self.last_update_time = time.time()

        # Odometry
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom.pose.covariance = [
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
        ]

        self.odom.twist.covariance = [
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
        ]

        # Subscribers and Publishers
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            "/diff_drive_controller/cmd_vel_out",
            self.cmd_vel_callback,
            10,
        )
        self.imu_sub = self.create_subscription(
            Imu, "sensors/imu/imu", self.imu_callback, 10
        )  # prevent unused variable warning
        self.odom_pub = self.create_publisher(Odometry, "/motors/odom", 10)
        self.joint_states_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Init robot handler class
        self.robot_handler = RobotHandler()
        self.robot_handler.set_wheel_circumference(self.wheel_circumference)
        self.encoder_res = self.robot_handler._encoder_res
        self.max_motor_speed = (
            self.robot_handler._max_speed / self.encoder_res * self.wheel_circumference
        )

        # Perodically send Motor Commands
        self.update_timer = self.create_timer(1 / self.frequency, self.update)

    def update(self):
        self.wheel_pos = self.robot_handler.get_wheel_positions()
        self.wheel_vel = self.robot_handler.get_wheel_velocities()
        self.wheel_vel_cmd = self.calculate_wheel_vel_cmd()
        self.robot_handler.send_command(self.wheel_vel_cmd)
        self.publish_joint_state()
        self.update_and_publish_odom()

    def cmd_vel_callback(self, msg):
        self.vx_cmd = msg.twist.linear.x
        self.yaw_cmd = msg.twist.angular.z

    def imu_callback(self, msg):
        self.omega_imu = msg.angular_velocity.z

    def publish_joint_state(self):
        joint_state_msg = JointState()

        # Fill in the header
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = ""  # Reference frame (if applicable)

        # Fill in the joint state data
        joint_state_msg.name = ["Left_1", "Right_1", "Left_2", "Right_2"]
        joint_state_msg.position = self.wheel_pos
        joint_state_msg.velocity = self.wheel_vel
        joint_state_msg.effort = []

        # Publish the message
        self.joint_states_pub.publish(joint_state_msg)

    def calculate_wheel_vel_cmd(self):
        wheel_vel_cmd = [0.0, 0.0, 0.0, 0.0]
        wheel_vel_cmd[0] = self.vx_cmd + self.yaw_cmd * self.dy_wheels / 2
        wheel_vel_cmd[1] = self.vx_cmd - self.yaw_cmd * self.dy_wheels / 2
        wheel_vel_cmd[2] = wheel_vel_cmd[0]
        wheel_vel_cmd[3] = wheel_vel_cmd[1]

        if wheel_vel_cmd[1] > self.max_motor_speed:
            self.get_logger().info(
                "Requested left_motors vel of: "
                + str(wheel_vel_cmd[1])
                + " meters per seconds"
            )
            self.get_logger().info(
                "Max left_motor vel is: " + self.max_motor_speed + " meters per seconds"
            )

        if wheel_vel_cmd[0] > self.max_motor_speed:
            self.get_logger().info(
                "Requested right_motors vel of: "
                + str(wheel_vel_cmd[0])
                + " meters per seconds"
            )
            self.get_logger().info(
                "Max right_motors vel is: "
                + self.max_motor_speed
                + " meters per seconds"
            )

        return wheel_vel_cmd

    def update_and_publish_odom(self):
        # Calculate velocity
        vel_average = sum(self.wheel_vel) / float(len(self.wheel_vel))
        vel_right = (self.wheel_vel[0] + self.wheel_vel[2]) / 2
        vel_left = (self.wheel_vel[1] + self.wheel_vel[3]) / 2
        omega = ((vel_right - vel_left) / 2) / (self.dy_wheels / 2)  # v_diff/2 / radius

        # Calculate Twist
        twist = Twist()
        twist.linear.x = vel_average
        twist.angular.z = omega

        # Calculate Pose
        quat_prev = Quaternion()
        quat_prev.x = self.odom.pose.pose.orientation.x
        quat_prev.y = self.odom.pose.pose.orientation.y
        quat_prev.z = self.odom.pose.pose.orientation.z
        quat_prev.w = self.odom.pose.pose.orientation.w
        quat_new = Quaternion()
        quat_new = self.quaternion_from_euler(
            twist.angular.x / self.frequency,
            twist.angular.y / self.frequency,
            twist.angular.z / self.frequency,
        )
        pose = Pose()
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = self.multiply_quaternions(quat_new, quat_prev)
        roll, pitch, yaw = self.euler_from_quaternion(pose.orientation)
        position = Point()
        position.x = (
            self.odom.pose.pose.position.x
            + vel_average * math.cos(yaw) / self.frequency
        )
        position.y = (
            self.odom.pose.pose.position.y
            + vel_average * math.sin(yaw) / self.frequency
        )

        pose.position = position

        # Update odom
        self.odom.twist.twist = twist
        self.odom.pose.pose = pose

        self.odom_pub.publish(self.odom)

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion
        quat = [w, x, y, z]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def multiply_quaternions(self, q1, q2):
        """
        Multiplies two quaternions q1 and q2.

        :param q1: First quaternion as a list or array [x, y, z, w]
        :param q2: Second quaternion as a list or array [x, y, z, w]
        :return: Resultant quaternion [x, y, z, w]
        """
        x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
        x2, y2, z2, w2 = q2.x, q2.y, q2.z, q2.w

        # Quaternion multiplication formula
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)

    rh = RosHandler()

    rclpy.spin(rh)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # rh.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
