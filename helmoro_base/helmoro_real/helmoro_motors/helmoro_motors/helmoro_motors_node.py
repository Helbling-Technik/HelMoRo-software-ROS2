import rclpy
import time
import math
from rclpy.node import Node

from helmoro_motors.robot_handler import RobotHandler

from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation


class RosHandler(Node):
    def __init__(self):
        super().__init__('helmoro_motor_commands_node')

        # Parameters
        # Wheel Geometry
        self.dx_wheels = 0.115
        self.dy_wheels = 0.195
        self.dia_wheels = 0.09
        self.wheel_circumference = self.dia_wheels * math.pi
        # Velocity constraints
        self.max_lin_vel = 1.1 #m/s
        self.max_ang_vel = 10.5 #rad/s

        # Variables
        self._v_i = 0.0 # Integration Error
        self._ki = 0.4 # Integration facto
        self.vx_cmd = 0.0
        self.yaw_cmd = 0.0
        self.omega_imu = 0.0 # Derivative of yaw
        self.wheel_pos = [0.0, 0.0, 0.0, 0.0] # in meters
        self.wheel_vel = [0.0, 0.0, 0.0, 0.0] # in meters per second
        self.last_update_time = time.time()

        # Subscribers and Publishers
        self.cmd_vel_sub = self.create_subscription(TwistStamped, '/diff_drive_controller/cmd_vel_out', self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'sensors/imu/imu', self.imu_callback, 10)  # prevent unused variable warning
        self.odom_pub = self.create_publisher(Odometry, '/motors/odom', 10)
        self.joint_states_pub = self.create_publisher(JointState, '/motors/joint_states', 10)

        # Init robot handler class
        self.robot_handler = RobotHandler()

        # Perodically send Motor Commands
        self.update_timer = self.create_timer(0.1, self.update)

    def update(self):
        self.wheel_pos = self.robot_handler.get_wheel_positions(self.wheel_circumference)
        self.wheel_vel = self.robot_handler.get_wheel_velocities(self.wheel_circumference)
        self.wheel_vel_cmd = self.calculate_wheel_vel_cmd()
        self.robot_handler.send_command([int(x * 2797) for x in self.wheel_vel_cmd])
        self.publish_joint_state()
        self.publish_odom()

    def cmd_vel_callback(self, msg):
        self.vx_cmd = msg.twist.linear.x
        self.yaw_cmd = msg.twist.angular.z

    def imu_callback(self, msg):
        self.omega_imu = msg.angular_velocity.z

    def publish_joint_state(self):
        joint_state_msg = JointState()

        # Fill in the header
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = ''  # Reference frame (if applicable)

        # Fill in the joint state data
        joint_state_msg.name = ["Left_1", "Right_1", "Left_2", "Right_2"]
        joint_state_msg.position = self.wheel_pos 
        joint_state_msg.velocity = self.wheel_vel
        joint_state_msg.effort = []

        # Publish the message
        self.joint_states_pub.publish(joint_state_msg)
        
    def publish_odom(self):
        # Create Odometry message
        odom_msg = Odometry()

        # Fill the header
        odom_msg.header.stamp = self.get_clock().now().to_msg()

        # Set position and orientation in the pose
        odom_msg.pose.pose.position.x = 0.0  
        odom_msg.pose.pose.position.y = 0.0  
        odom_msg.pose.pose.position.z = 0.0  
        odom_msg.pose.pose.orientation.x = 0.0  
        odom_msg.pose.pose.orientation.y = 0.0 
        odom_msg.pose.pose.orientation.z = 0.0  
        odom_msg.pose.pose.orientation.w = 0.0  

        # Covariance (example: identity matrix)
        odom_msg.pose.covariance = [0.9] * 36

        # Set velocity in the twist
        vel_average = sum(self.wheel_vel) / float(len(self.wheel_vel))
        vel_right = (self.wheel_vel[0] + self.wheel_vel[2]) / 2
        vel_left = (self.wheel_vel[1] + self.wheel_vel[3]) / 2

        odom_msg.twist.twist.linear.x = vel_average  # Average of wheel velocity
        odom_msg.twist.twist.linear.y = 0.0  
        odom_msg.twist.twist.linear.z = 0.0  
        odom_msg.twist.twist.angular.x = 0.0  
        odom_msg.twist.twist.angular.y = 0.0  
        odom_msg.twist.twist.angular.z = ((vel_right - vel_left) / 2) / (self.dia_wheels / 2) # v_diff/2 / radius
        
        # Covariance (example: identity matrix)
        odom_msg.twist.covariance = [0.9] * 36

        # Publish the message
        self.odom_pub.publish(odom_msg)

    def calculate_wheel_vel_cmd(self):
        wheel_vel_cmd = [0.0, 0.0, 0.0, 0.0]
        wheel_vel_cmd[0] = self.vx_cmd + self.yaw_cmd * self.dy_wheels / 2
        wheel_vel_cmd[1] = self.vx_cmd - self.yaw_cmd * self.dy_wheels / 2
        wheel_vel_cmd[2] = wheel_vel_cmd[0]
        wheel_vel_cmd[3] = wheel_vel_cmd[1]

        return wheel_vel_cmd
    
def main(args=None):
    rclpy.init(args=args)

    rh = RosHandler()

    rclpy.spin(rh)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # rh.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()