#include "helmoro_motor_commands/helmoro_motor_commands.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#define CONSTRAIN(VAL, MIN_VAL, MAX_VAL) (VAL < MAX_VAL ? ((VAL > MIN_VAL ? VAL : MIN_VAL)) : MAX_VAL)

namespace helmoro_motor_commands
{
using namespace std::placeholders;

HelmoroMotorCommands::HelmoroMotorCommands(const rclcpp::NodeOptions & options)
  : rclcpp::Node("helmoro_motor_commands", options), motors_left_("/dev/ttyACM1", 115200), motors_right_("/dev/ttyACM0", 115200)
{ 
  // read parameters
  GetParams();

  // Create subscription to let other applications drive the robot
  move_base_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SensorDataQoS(),
    std::bind(&HelmoroMotorCommands::CmdVelocityCallback, this, _1));

  // Create subscription for imu data
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(),
    std::bind(&HelmoroMotorCommands::imuDataCallback, this, _1));

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "helmoro_joint_states", rclcpp::SystemDefaultsQoS());

  if (!is_real_robot_) {
    // Create subscription for imu data
    motor_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "helmoro_motor_states", rclcpp::SensorDataQoS(),
      std::bind(&HelmoroMotorCommands::motorStatesCallback, this, _1));

    motor_commands_pub_ = this->create_publisher<helmoro_msgs::msg::HelmoroJointCommands>(
      "helmoro_motor_commands", rclcpp::SensorDataQoS().reliable());
  }

  // cmd_vel commands
  vx_cmd_ = 0.0;
  dtheta_cmd_ = 0.0;

  wheel_vel_cmd_[0] = 0.0;
  wheel_vel_cmd_[1] = 0.0;
  wheel_vel_cmd_[2] = 0.0;
  wheel_vel_cmd_[3] = 0.0;

  omega_imu_ = 0.0;

  // Create timer to periodically update motor commands
  constexpr auto control_period = std::chrono::duration<double>(1.0 / 40.0);
  control_timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    rclcpp::Duration(control_period),
    std::bind(&HelmoroMotorCommands::update, this));
}

void HelmoroMotorCommands::update()
{
  std::unique_lock<std::mutex> lock(m_lock_);
  getWheelVelocitiesCommand();
  lock.unlock();
  if (is_real_robot_) {
    // read voltages
    // motors_right_.GetMainBatteryVoltage(addr_right_, batt_volt_right_);

    // read wheel positions
    motors_left_.GetM1M2Pos(addr_left_, wheel_pos_[1], wheel_pos_[3]);
    motors_right_.GetM1M2Pos(addr_right_, wheel_pos_[0], wheel_pos_[2]);

    wheel_pos_[0] = wheel_pos_[0] / tics_per_rad_;
    wheel_pos_[1] = wheel_pos_[1] / tics_per_rad_;
    wheel_pos_[1] = wheel_pos_[2] / tics_per_rad_;
    wheel_pos_[3] = wheel_pos_[3] / tics_per_rad_;

    // read wheel speeds
    motors_left_.GetM1SpeedFiltered(addr_left_, wheel_vel_state_[1]);
    motors_left_.GetM2SpeedFiltered(addr_left_, wheel_vel_state_[3]);
    motors_right_.GetM1SpeedFiltered(addr_right_, wheel_vel_state_[0]);
    motors_right_.GetM2SpeedFiltered(addr_right_, wheel_vel_state_[2]);

    wheel_vel_state_[0] = wheel_vel_state_[0] / tics_per_rad_;
    wheel_vel_state_[1] = wheel_vel_state_[1] / tics_per_rad_;
    wheel_vel_state_[2] = wheel_vel_state_[2] / tics_per_rad_;
    wheel_vel_state_[3] = wheel_vel_state_[3] / tics_per_rad_;

    // send wheel commands
    if (!motors_left_.SendSpeedM1M2Command(addr_left_, wheel_vel_cmd_[1] * tics_per_rad_,
                                           wheel_vel_cmd_[3] * tics_per_rad_))
      std::printf("Could not sent speed to left motors \n");
    if (!motors_right_.SendSpeedM1M2Command(addr_right_, wheel_vel_cmd_[0] * tics_per_rad_,
                                            wheel_vel_cmd_[2] * tics_per_rad_))
      std::printf("Could not send speed to right motors \n");
  } else {
    PublishMotorCommands();
  }

  // publish joint states
  PublishJointStates();
}

void HelmoroMotorCommands::CmdVelocityCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(m_lock_);
  vx_cmd_ = msg->linear.x;
  dtheta_cmd_ = msg->angular.z;
}

void HelmoroMotorCommands::imuDataCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  omega_imu_ = msg->angular_velocity.z;
}

void HelmoroMotorCommands::motorStatesCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  for (unsigned int i = 0; i < nb_actuators_; i++) {
    wheel_pos_[i] = msg->position[i];
    wheel_vel_state_[i] = msg->velocity[i];
  }
}

void HelmoroMotorCommands::GetParams()
{
  // Helmoro specs
  if (this->has_parameter("use_sim_time")) {
    is_real_robot_ = false;
    RCLCPP_INFO(this->get_logger(), "[Motor commands]: Commanding in simulation");
  }
  this->declare_parameter("/helmoro_motor_commands/dimensions/wheel_spacing_x", 5.0);
  dx_wheels_ = this->get_parameter("/helmoro_motor_commands/dimensions/wheel_spacing_x").as_double();
  this->declare_parameter("/helmoro_motor_commands/dimensions/wheel_spacing_y", 5.0);
  dy_wheels_ = this->get_parameter("/helmoro_motor_commands/dimensions/wheel_spacing_y").as_double();
  this->declare_parameter("/helmoro_motor_commands/dimensions/wheel_diameter", 5.0);
  dia_wheels_ = this->get_parameter("/helmoro_motor_commands/dimensions/wheel_diameter").as_double();
  this->declare_parameter("/helmoro_motor_commands/actuators/max_wheel_rot_vel", 14.0);
  max_wheel_rot_vel_ = this->get_parameter("/helmoro_motor_commands/actuators/max_wheel_rot_vel").as_double();
  this->declare_parameter("/helmoro_motor_commands/velocities/max_ang_vel", 6.3);
  max_ang_vel_ = this->get_parameter("/helmoro_motor_commands/velocities/max_ang_vel").as_double();
  this->declare_parameter("/helmoro_motor_commands/velocities/max_lin_vel", 1.0);
  max_lin_vel_ = this->get_parameter("/helmoro_motor_commands/velocities/max_lin_vel").as_double();

  // Motor controller ports, addresses & baudrate
  this->declare_parameter("helmoro_motor_commands/address_left", 128);
  addr_left_ = this->get_parameter("helmoro_motor_commands/address_left").as_int();
  this->declare_parameter("helmoro_motor_commands/address_right", 129);
  addr_right_ = this->get_parameter("helmoro_motor_commands/address_right").as_int();
  this->declare_parameter("helmoro_motor_commands/baudrate", 115200);
  baud_ = this->get_parameter("helmoro_motor_commands/baudrate").as_int();
  this->declare_parameter("/helmoro_motor_commands/actuators/encoder_resolution", 2797);
  tics_per_rad_ = this->get_parameter("/helmoro_motor_commands/actuators/encoder_resolution").as_int() / (2 * M_PI);

  // integral factor
  this->declare_parameter("helmoro_motor_commands/integral/ki_factor", 1.0);
  ki_ = this->get_parameter("helmoro_motor_commands/integral/ki_factor").as_double();
}

void HelmoroMotorCommands::getWheelVelocitiesCommand()
{
  // max rot speed for distinct radius
  // double omega_max;
  // double curve_radius;
  double linear_scaling_factor;
  // double velocity_summ;
  double rot_lin_vel;
  double vel_sum_left;
  double vel_sum_right;

  // calculate wheel speed from base rotation
  rot_lin_vel = dtheta_cmd_ * (dy_wheels_ / 2);

  // calculate scaling factor
  if (vx_cmd_ + rot_lin_vel > max_lin_vel_) {
    linear_scaling_factor = max_lin_vel_ / (vx_cmd_ + rot_lin_vel);
  } else {
    linear_scaling_factor = 1;
  }

  // integration term on angular velocity error
  double dt = CONSTRAIN(timer_.ElapsedSeconds(), 0.0, 0.1);
  timer_.Reset();
  static double v_i = 0.0;

  if (vx_cmd_ + rot_lin_vel != 0) {
    v_i += ki_ * (dtheta_cmd_ - omega_imu_) * dt;
    v_i = CONSTRAIN(v_i, -0.15, 0.15);

    rot_lin_vel = rot_lin_vel + v_i;
  }

  vel_sum_left = (vx_cmd_ + rot_lin_vel) * linear_scaling_factor;
  vel_sum_right = (vx_cmd_ - rot_lin_vel) * linear_scaling_factor;

  wheel_vel_cmd_[0] = 2.0 * vel_sum_left / dia_wheels_;
  wheel_vel_cmd_[1] = 2.0 * vel_sum_right / dia_wheels_;
  wheel_vel_cmd_[2] = wheel_vel_cmd_[0];
  wheel_vel_cmd_[3] = wheel_vel_cmd_[1];
}

void HelmoroMotorCommands::PublishJointStates()
{
  sensor_msgs::msg::JointState joint_states;
  joint_states.header.stamp = this->now();
  for (unsigned int i = 0; i < nb_actuators_; i++) {
    joint_states.name.push_back(helmoro_description::HelmoroJointNames::getName(i));
    joint_states.position.push_back(wheel_pos_[i]);
    joint_states.velocity.push_back(wheel_vel_state_[i]);
  }

  joint_state_pub_->publish(joint_states);
}

void HelmoroMotorCommands::PublishMotorCommands()
{
  auto motor_commands_msg = std::make_unique<helmoro_msgs::msg::HelmoroJointCommands>();
  motor_commands_msg->header.stamp = this->now();
  motor_commands_msg->header.frame_id = base_frame_;
  motor_commands_msg->mode = static_cast<int>(helmoro_description::ActuatorModeEnum::MODE_JOINTVELOCITY);
  for (unsigned int i = 0; i < nb_actuators_; i++) {
    motor_commands_msg->command[i] = wheel_vel_cmd_[i];
  }
  motor_commands_pub_->publish(std::move(motor_commands_msg));
}

}  // namespace helmoro_motor_commands

RCLCPP_COMPONENTS_REGISTER_NODE(helmoro_motor_commands::HelmoroMotorCommands)