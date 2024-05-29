// Gazebo includes
#include "gazebo/common/Assert.hh"
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>


// Message includes
#include <sensor_msgs/msg/joint_state.hpp>
#include "helmoro_description/enums/enums.hpp"
#include "helmoro_description/helmoro_names.hpp"
#include <helmoro_msgs/msg/helmoro_joint_commands.hpp>


// System includes
#include <mutex>
#include <string>
#include <vector>

namespace gazebo {

class Model_Helmoro : public ModelPlugin {

 public:
   /// Constructor for Gazebo plugin
   Model_Helmoro();

   /// The load function which is called when opening Gazebo
   void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

 private:
   // Called by the world update start event
   void Update();

   /// Reads Data from Simulation
   void readSimulation();

   // Writes Actuator Commands to simulation
   void writeSimulation();

   static constexpr unsigned int noActuators_ = static_cast<int>(helmoro_description::ActuatorEnum::NrActuators);

   /* Gazebo */
   physics::ModelPtr model_;
   physics::WorldPtr world_;
   sdf::ElementPtr sdf_;

   event::ConnectionPtr updateConnection_;

   common::Time last_sim_time_;
   common::Time last_update_time_;
   double update_period_ms_;
   double loopRate_;


   /* ROS */
   rclcpp::Node::SharedPtr ros_node_;
   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_states_pub_;
   rclcpp::Subscription<helmoro_msgs::msg::HelmoroJointCommands>::SharedPtr motor_commands_sub_;

   /* States */
   //! Joint Torques to apply to simulation
   double tau_[noActuators_];
   //! Joint Forces read from simulation
   double motor_torques_[noActuators_];
   //! Joint Position read from simulation
   double motor_positions_[noActuators_];
   //! Joint Velocities read from simulation
   double motor_velocities_[noActuators_];

   double motor_commands_[noActuators_];
   int motor_commands_mode_;


   //! mutex locks (multiple threads)
   std::mutex mutexActuatorCommandsCallback_;
   std::mutex mutexControllerDefinition_;
   std::mutex mutexLoad_;
   std::mutex mutexOnUpdate_;
};

} // namespace gazebo
