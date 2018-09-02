#ifndef BLUE_HARDWARE_INTERFACE_H
#define BLUE_HARDWARE_INTERFACE_H

#include "blue_controller_manager/blue_hardware_interface.h"

#include <ros/ros.h>
#include <vector>
#include <string>
#include <memory>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <blue_msgs/MotorState.h>

#include <geometry_msgs/Vector3.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/segment.hpp>

#include "blue_hardware_drivers/BLDCDriver.h"

namespace ti = transmission_interface;

class BlueHW: public hardware_interface::RobotHW
{
public:

  BlueHW(ros::NodeHandle &nh);
  void read();
  void write();
  void setControl(bool is_enabled);

private:

  // Setup helpers
  template <typename TParam>
  void getRequiredParam(ros::NodeHandle &nh, const std::string name, TParam &dest);
  void setupMotorComms(std::string &port);
  void buildDynamicChain(KDL::Chain &chain);

  void calibrationStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void setReadGravityVector();
  void computeInverseDynamics();

  struct JointParams
  {
    std::string joint_name;
    hardware_interface::JointHandle joint;
    double id_gain;
    double max_torque;
    double min_torque;
    double max_angle;
    double min_angle;
  };

  bool read_from_motors_;

  // ROS control hardware interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::EffortJointInterface joint_effort_interface_;

  // Comms controller
  BLDCDriver bldc_;

  // Publishers and subscribers
  ros::Publisher motor_state_publisher_;
  ros::Subscriber joint_calibration_sub_;
  ros::Publisher joint_gravity_publisher_;

  // Parameters read in from configuration
  std::vector<std::string> joint_names_;
  std::vector<std::string> motor_names_;
  std::vector<double> gear_ratios_;
  std::vector<double> current_to_torque_ratios_;
  std::vector<int> differential_pairs_;
  std::vector<double> softstop_min_angles_;
  std::vector<double> softstop_max_angles_;
  std::vector<double> joint_torque_directions_;
  std::vector<double> id_gains_;
  double softstop_torque_limit_;
  double softstop_tolerance_;
  std::vector<double> motor_current_limits_;

  // Calibration
  int calibration_counter_;
  bool is_calibrated_;
  std::vector<double> joint_pos_initial_;
  std::vector<double> actuator_pos_initial_;
  int num_joints_;

  // Transmission interfaces
  ti::ActuatorToJointStateInterface actuator_to_joint_interface_;
  ti::JointToActuatorEffortInterface joint_to_actuator_interface_;
  std::vector<std::shared_ptr<ti::SimpleTransmission>> simple_transmissions_;
  std::vector<std::shared_ptr<ti::DifferentialTransmission>> differential_transmissions_;
  int num_diff_actuators_;
  bool has_base_;
  bool has_gripper_;

  // Actuator and joint space data
  std::vector<ti::ActuatorData> actuator_states_;
  std::vector<ti::ActuatorData> actuator_commands_;
  std::vector<ti::JointData> joint_states_;
  std::vector<ti::JointData> joint_commands_;

  // Motor states
  std::vector<comm_id_t> motor_ids_;
  blue_msgs::MotorState motor_states_;
  std::map<comm_id_t, float> motor_commands_;

  // Owned by our ActuatorData and JointData objects
  std::vector<double> actuator_pos_;
  std::vector<double> actuator_vel_;
  std::vector<double> actuator_eff_;
  std::vector<double> actuator_cmd_;
  std::vector<KDL::Vector> actuator_accel_;
  std::vector<KDL::Vector> read_gravity_vector_;
  std::vector<double> joint_pos_;
  std::vector<double> joint_vel_;
  std::vector<double> joint_eff_;
  std::vector<double> joint_cmd_;
  std::vector<double> raw_joint_cmd_;

  // Gravity Compensation
  KDL::Vector gravity_vector_;
  KDL::Chain kdl_chain_;
  KDL::JntArray id_torques_;
  std::vector<std::shared_ptr<JointParams>> joint_params_;
};

#endif // BLUE_HARDWARE_INTERFACE
