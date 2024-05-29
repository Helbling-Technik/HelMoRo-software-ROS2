#include "helmoro_gazebo_plugin/helmoro_gazebo_plugin.hpp"

#include <Eigen/Core>


namespace gazebo {

Model_Helmoro::Model_Helmoro()
: last_sim_time_{0},
  last_update_time_{0},
  update_period_ms_{8}
{
}

void Model_Helmoro::Load(physics::ModelPtr model, sdf::ElementPtr sdf){

  model_ = model;
  world_ = model_->GetWorld();
  GZ_ASSERT(world_, "[WHEEL DROP PLUGIN] Invalid world pointer!");

  // Set up ROS node and subscribers and publishers
  ros_node_ = gazebo_ros::Node::Get(sdf);
  RCLCPP_INFO(ros_node_->get_logger(), "Loading HelMoRo Gazebo Plugin");

  // Motor States Publisher
  motor_states_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>("helmoro_motor_states", 10);

  // Motor Commands Subscription with callback
  motor_commands_sub_ = ros_node_->create_subscription<helmoro_msgs::msg::HelmoroJointCommands>(
    "helmoro_motor_commands", 10,
    [ = ](helmoro_msgs::msg::HelmoroJointCommands::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutexActuatorCommandsCallback_);
      for(unsigned int i = 0; i < noActuators_; i++) {
        motor_commands_mode_ = msg->mode;
        motor_commands_[i] = msg->command[i];
      }
    }
  );

  // initialise 
  for(unsigned int i = 0; i < noActuators_; i++){
    motor_positions_[i] = 0.0;
    motor_velocities_[i] = 0.0;
    motor_torques_[i] = 0.0;
    tau_[i] = 0.0;
    motor_commands_[i] = 0.0;
  }

  loopRate_ = 100.0;

  for(unsigned int i=0; i<noActuators_; i++) {
      /// Apply joint torques
      model_->GetJoint(helmoro_description::HelmoroJointNames::getName(i))->SetPosition(0, 0);
  }

  RCLCPP_DEBUG(ros_node_->get_logger(), "Got joints:");

  // Physics Engine
  physics::PhysicsEnginePtr physics = world_->Physics();
  const std::string frictionModel = "cone_model";
  physics->SetParam("friction_model", frictionModel);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&Model_Helmoro::Update, this));

  RCLCPP_INFO(ros_node_->get_logger(), "Finished loading helmoro gazebo plugin");
}

// Called by the world update start event
void Model_Helmoro::Update(){

  std::lock_guard<std::mutex> lock(mutexOnUpdate_);

  auto cur_time = world_->SimTime();
  if (last_sim_time_ == 0) {
    last_sim_time_ = cur_time;
    last_update_time_ = cur_time;
    return;
  }

  // read from simulation
  readSimulation();
  
  // Publish motor states
  auto update_dt = (cur_time - last_update_time_).Double();
  if (update_dt * 1000 >= update_period_ms_) {
    auto motorStatesMsg = sensor_msgs::msg::JointState{};
    motorStatesMsg.header.stamp = ros_node_->now();

    motorStatesMsg.name.resize(noActuators_);
    motorStatesMsg.position.resize(noActuators_);
    motorStatesMsg.velocity.resize(noActuators_);
    for (unsigned int i=0; i<noActuators_; i++) {

        motorStatesMsg.name[i] = helmoro_description::HelmoroJointNames::getName(i);
        motorStatesMsg.position[i] = motor_positions_[i];
        motorStatesMsg.velocity[i] = motor_velocities_[i];
    }
    motor_states_pub_->publish(motorStatesMsg);
    last_update_time_ = cur_time;
  }


  std::lock_guard<std::mutex> lock2(mutexActuatorCommandsCallback_);
  std::lock_guard<std::mutex> lock3(mutexControllerDefinition_);

  // update controller and apply control signal to joints. references are updated in the callback
  writeSimulation();

  last_sim_time_ = cur_time;
}

void Model_Helmoro::readSimulation(){

  //! Read joint states
  // get force, position and velocity of each joint
  for(unsigned int i=0; i<noActuators_; i++) {

      auto joint = model_->GetJoint(helmoro_description::HelmoroJointNames::getName(i));
      if (!joint) {

          RCLCPP_ERROR_STREAM_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 1000, "Joint " +  std::string(helmoro_description::HelmoroJointNames::getName(i)) + " does not exist in Gazebo.");
          return;
      }
      motor_torques_[i] = joint->GetForce(i);
      motor_positions_[i] = joint->Position(i);
      motor_velocities_[i] = joint->GetVelocity(i);
  }
}

void Model_Helmoro::writeSimulation(){

  /// calculate controller outputs
  for(unsigned int i=0; i<noActuators_; i++) {
      model_->GetJoint(helmoro_description::HelmoroJointNames::getName(i))->SetVelocity(0, motor_commands_[i]);
  }
}

GZ_REGISTER_MODEL_PLUGIN(Model_Helmoro)

}  // namespace gazebo
