import rclpy
import time
from rclpy.node import Node

from helmoro_motors.robot_handler import RobotHandler

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState


class RosHandler(Node):

    # Integration Error as static variable
    v_i = 0.0

    def __init__(self):
        super().__init__('helmoro_motor_commands_node')

        # Parameters
        # Wheel Geometry
        self.dx_wheels = 0.115
        self.dy_wheels = 0.195
        self.dia_wheels = 0.09
        # Velocity constraints
        self.max_lin_vel = 1.1 #m/s
        self.max_ang_vel = 10.5 #rad/s

        # Variables
        self.vx_cmd = 0.0
        self.yaw_cmd = 0.0
        self.omega_imu = 0.0 # Derivative of yaw
        self.wheel_vel_cmd = [0.0, 0.0, 0.0, 0.0]
        self.last_update_time = time.time()

        # Subscribers and Publishers
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'sensors/imu/imu', self.imu_callback, 10)  # prevent unused variable warning
        self.odom_pub = self.create_publisher(Twist, 'odom', 10)
        self.joint_states_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Init robot handler class
        self.robot_handler = RobotHandler()

        # Perodically send Motor Commands
        self.update_timer = self.create_timer(1, self.update)

    def update(self):
        self.wheel_vel_cmd = self.calculate_wheel_vel_cmd()
        self.robot_handler.send_command(self.wheel_vel_cmd)
        self.publish_odom_and_joint_state()

    def cmd_vel_callback(self, msg):
        self.vx_cmd = msg.linear.x
        self.yaw_cmd = msg.angular.z
        self.get_logger().info('I heard: "%f"' % msg.linear.x)

    def imu_callback(self, msg):
        self.omega_imu = msg.angular_velocity.z

    def publish_odom_and_joint_state(self):
        pass

    def calculate_wheel_vel_cmd(self):
        linear_scaling_factor = 0.0
        rot_lin_vel = 0.0
        vel_sum_left = 0.0
        vel_sum_right = 0.0

        # calculate wheel speed from base rotation
        rot_lin_vel = self.yaw_cmd * (self.dy_wheels / 2)

        # calculate scaling factor
        if (self.vx_cmd + rot_lin_vel > self.max_lin_vel):
            linear_scaling_factor = self.max_lin_vel / (self.vx_cmd + rot_lin_vel)
        else:
            linear_scaling_factor = 1

        # integration term on angular velocity error
        dt = self._constrain(time.time()-self.last_update_time, 0.0, 0.1)

        if (self.vx_cmd + rot_lin_vel != 0) {
            v_i += self.ki * (self.yaw_cmd - self.omega_imu) * dt;
            v_i = self._constrain(v_i, -0.15, 0.15);

            rot_lin_vel = rot_lin_vel + v_i;
        }

        vel_sum_left = (self.vx_cmd + rot_lin_vel) * linear_scaling_factor
        vel_sum_right = (self.vx_cmd - rot_lin_vel) * linear_scaling_factor

        wheel_vel_cmd[0] = 2.0 * vel_sum_left / self.dia_wheels
        wheel_vel_cmd[1] = 2.0 * vel_sum_right / self.dia_wheels
        wheel_vel_cmd[2] = wheel_vel_cmd[0]
        wheel_vel_cmd[3] = wheel_vel_cmd[1]

        return wheel_vel_cmd

    def _constrain(self, val, min_val, max_val):
        if val > max_val:
            return max_val
        elif val < min_val:
            return min_val
        else:
            return val

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