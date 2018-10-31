#ifndef BLUE_TRANSMISSIONS_H
#define BLUE_TRANSMISSIONS_H

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
#include <transmission_interface/transmission_interface.h>

namespace ti = transmission_interface;

class BlueTransmissions
{
public:

  BlueTransmissions();

  void init(
      std::vector<std::string> joint_names,
      std::vector<int> differential_pairs,
      std::vector<double> gear_ratios);

  // ROS control hardware interfaces
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::EffortJointInterface joint_effort_interface;

  // Getter functions
  const std::vector<double>& getJointPos();
  const std::vector<double>& getJointVel();

  std::vector<double> getActuatorCommands(
      std::vector<double> feedforward_torques);

private:

  int num_joints_;
  int num_diff_actuators_;

  // Transmission interfaces
  ti::ActuatorToJointStateInterface actuator_to_joint_interface_;
  ti::JointToActuatorEffortInterface joint_to_actuator_interface_;

  // Actuator and joint state data
  std::vector<ti::Transmission *> transmissions_;
  std::vector<ti::ActuatorData> actuator_states_;
  std::vector<ti::ActuatorData> actuator_commands_;
  std::vector<ti::JointData> joint_states_;
  std::vector<ti::JointData> joint_commands_;

  // Data owned by our ActuatorData and JointData objects
  // Also shared by the joint state/effort interfaces
  std::vector<double> actuator_pos_;
  std::vector<double> actuator_vel_;
  std::vector<double> actuator_eff_;
  std::vector<double> actuator_cmd_;
  std::vector<double> joint_pos_;
  std::vector<double> joint_vel_;
  std::vector<double> joint_eff_;
  std::vector<double> joint_cmd_;
  std::vector<double> raw_joint_cmd_;

};

#endif // BLUE_TRANSMISSIONS

