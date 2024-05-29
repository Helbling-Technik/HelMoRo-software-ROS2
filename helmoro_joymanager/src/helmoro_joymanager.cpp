#include "helmoro_joymanager/helmoro_joymanager.hpp"

namespace helmoro_joymanager
{
using namespace std::placeholders;

HelmoroJoyManager::HelmoroJoyManager(const rclcpp::NodeOptions & options)
  : rclcpp::Node("helmoro_joymanager", options)
{
  // initialize publishers and subscribers
  user_input_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS());

  joystick_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::SensorDataQoS(),
      std::bind(&HelmoroJoyManager::joystickCallback, this, _1));

  // load scaling factors from parameter file
  this->declare_parameter("/helmoro_joymanager/scaling_factors/linear_velocity", 1.0);
  linearVelocityScalingFactor_ = this->get_parameter("/helmoro_joymanager/scaling_factors/linear_velocity").as_double();
  this->declare_parameter("/helmoro_joymanager/scaling_factors/angular_velocity", 1.0);
  angularVelocityScalingFactor_ = this->get_parameter("/helmoro_joymanager/scaling_factors/angular_velocity").as_double();

  this->declare_parameter("/helmoro_motor_commands/velocities/max_lin_vel", 1.0);
  maxLinVel_ = this->get_parameter("/helmoro_motor_commands/velocities/max_lin_vel").as_double();
  this->declare_parameter("/helmoro_motor_commands/velocities/max_ang_vel", 6.3);
  maxAngVel_ = this->get_parameter("/helmoro_motor_commands/velocities/max_ang_vel").as_double();

  right_trigger_initalized_ = false;  // needs to be 0 so that velocity commands are accepted
  left_trigger_initalized_ = false;
}

void HelmoroJoyManager::joystickCallback(sensor_msgs::msg::Joy::ConstSharedPtr msg)
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

}  // namespace helmoro_joymanager

RCLCPP_COMPONENTS_REGISTER_NODE(helmoro_joymanager::HelmoroJoyManager)
