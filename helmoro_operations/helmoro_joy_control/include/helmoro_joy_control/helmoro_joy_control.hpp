#pragma once

// ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

// stl
#include <string>

namespace helmoro_joy_control
{
class HelmoroJoyControl : public rclcpp::Node
{
public:
  /// \brief Constructor
  explicit HelmoroJoyControl(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void joystickCallback(sensor_msgs::msg::Joy::ConstSharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr user_input_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;

  double linearVelocityScalingFactor_;
  double angularVelocityScalingFactor_;
  double maxLinVel_;
  double maxAngVel_;
  bool right_trigger_initalized_;
  bool left_trigger_initalized_;
};

}  // namespace helmoro_joy_control
