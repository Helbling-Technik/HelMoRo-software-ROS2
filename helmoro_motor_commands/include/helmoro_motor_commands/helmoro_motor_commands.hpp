#pragma once

// motor controller
#include "helmoro_motor_commands/motor_interface/motor_interface.h"
#include "helmoro_motor_commands/timer/timer.h"

// STD
#include <math.h>

#include <string>
#include <vector>

// Helmoro
#include "helmoro_description/enums/enums.hpp"
#include "helmoro_description/helmoro_names.hpp"
#include "helmoro_msgs/msg/helmoro_joint_commands.hpp"

// ros
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"

namespace helmoro_motor_commands
{
class HelmoroMotorCommands : public rclcpp::Node
{
public:
  /// \brief Constructor
  explicit HelmoroMotorCommands(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:

  void update();
  void getWheelVelocitiesCommand();
  void PublishMotorCommands();
  void PublishJointStates();
  void GetParams();

  /// \brief Callback for new velocity commands
  void CmdVelocityCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);

  /// \brief Callback for new imu data
  void imuDataCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg);

  /// \brief Callback for new imu data
  void motorStatesCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg);

  static constexpr unsigned int nb_actuators_ = static_cast<int>(helmoro_description::ActuatorEnum::NrActuators);
  bool is_real_robot_{true};

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr move_base_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_states_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<helmoro_msgs::msg::HelmoroJointCommands>::SharedPtr motor_commands_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_ {nullptr};

  const std::string base_frame_ {"base_link"};

  // motor controllers
  std::string port_left_;
  std::string port_right_;
  uint8_t addr_left_;
  uint8_t addr_right_;
  int baud_;
  MotorInterface motors_left_;
  MotorInterface motors_right_;

  // encoder
  int tics_per_rad_;

  // battery voltages
  double batt_volt_left_;
  double batt_volt_right_;

  // wheel speeds
  double wheel_vel_state_[nb_actuators_] = {0.0, 0.0, 0.0, 0.0};
  double wheel_vel_filtered_[nb_actuators_] = {0.0, 0.0, 0.0, 0.0};
  double wheel_pos_[nb_actuators_];

  // move_base
  double dx_wheels_;
  double dy_wheels_;
  double dia_wheels_;
  double max_wheel_rot_vel_;
  double max_ang_vel_;
  double max_lin_vel_;
  double wheel_vel_cmd_[nb_actuators_] = {0.0, 0.0, 0.0, 0.0};
  double vx_cmd_;
  double dtheta_cmd_;

  // publish joint states
  int pub_counter_;

  // IMU Integral
  double omega_imu_;
  double ki_ = 1.0;
  Timer timer_;

  // mutex lock
  std::mutex m_lock_;
};

}  // namespace helmoro_motor_commands
