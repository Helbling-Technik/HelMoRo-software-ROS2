#include "helmoro_joy_control/helmoro_joy_control.hpp"

namespace helmoro_joy_control
{
using namespace std::placeholders;

HelmoroJoyControl::HelmoroJoyControl(const rclcpp::NodeOptions & options)
  : rclcpp::Node("helmoro_joy_control", options)
{
  // initialize publishers and subscribers
  user_input_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS());

  joystick_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::SensorDataQoS(),
      std::bind(&HelmoroJoyControl::joystickCallback, this, _1));

  // load scaling factors from parameter file
  linearVelocityScalingFactor_ = this->declare_parameter("linear_velocity", 1.0);
  angularVelocityScalingFactor_ = this->declare_parameter("angular_velocity", 1.0);
  maxLinVel_ = this->declare_parameter("max_lin_vel", 1.0);
  maxAngVel_ = this->declare_parameter("max_ang_vel", 6.3);

  right_trigger_initalized_ = false;  // needs to be 0 so that velocity commands are accepted
  left_trigger_initalized_ = false;
}

void HelmoroJoyControl::joystickCallback(sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  geometry_msgs::msg::Twist cmd_vel;

  // check if the right trigger has been initalized
  if ((not right_trigger_initalized_) && msg->axes[5] != 0)
    right_trigger_initalized_ = true;

  if ((not left_trigger_initalized_) && msg->axes[2] != 0)
    left_trigger_initalized_ = true;

  // set the linear and angular velocity fo the robot
  if (right_trigger_initalized_ && left_trigger_initalized_) {  // only if no value is == 0, as this is the case at INIT
    cmd_vel.linear.x = (-(msg->axes[5] - 1) - (-(msg->axes[2] - 1))) * 0.5 * linearVelocityScalingFactor_ * maxLinVel_;
  }

  cmd_vel.angular.z = msg->axes[0] * angularVelocityScalingFactor_ * maxAngVel_;

  user_input_pub_->publish(cmd_vel);
}

}  // namespace helmoro_joy_control

RCLCPP_COMPONENTS_REGISTER_NODE(helmoro_joy_control::HelmoroJoyControl)
