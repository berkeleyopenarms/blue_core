#ifndef BLUE_TRANSMISSIONS_H
#define BLUE_TRANSMISSIONS_H

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
#include <transmission_interface/transmission_interface.h>

#include <kdl/frames.hpp>

namespace ti = transmission_interface;

class BlueKinematics
{
public:

  BlueKinematics();

  void init(
      const std::vector<std::string> &joint_names,
      const std::vector<int> &differential_pairs,
      const std::vector<double> &gear_ratios);

  // ROS control hardware interfaces
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::EffortJointInterface joint_effort_interface;

  // Getter functions
  const std::vector<double>& getJointPos();
  const std::vector<double>& getJointVel();

  // Joint calibration
  void setJointOffsets(
      const std::vector<double> &offsets);

  //  Update internal actuator states
  void setActuatorStates(
      const std::vector<double> &positions,
      const std::vector<double> &velocities,
      const std::vector<double> &efforts,
      const std::vector<double> &accel_x,
      const std::vector<double> &accel_y,
      const std::vector<double> &accel_z);

  // Which way is down??
  std::vector<double> getGravityVector();

  // Get desired actuator commands
  std::vector<double> getActuatorCommands(
      const std::vector<double> &feedforward_torques,
      double softstop_torque_limit, // TODO: clean up softstop code
      const std::vector<double> &softstop_min_angles,
      const std::vector<double> &softstop_max_angles,
      double softstop_tolerance);

private:

  bool is_calibrated_;

  int num_joints_;
  int num_diff_actuators_;
  int num_simple_transmissions_;
  int num_diff_transmissions_;
  int num_transmissions_;

  bool has_base_;
  bool has_gripper_;

  // Transmission interfaces
  ti::ActuatorToJointStateInterface actuator_to_joint_interface_;
  ti::JointToActuatorEffortInterface joint_to_actuator_interface_;

  // Joint offsets
  std::vector<double> actuator_offsets_;
  std::vector<double> joint_offsets_;

  // Actuator and joint state data
  std::vector<ti::Transmission *> transmissions_;
  std::vector<ti::ActuatorData> actuator_states_;
  std::vector<ti::ActuatorData> actuator_commands_;
  std::vector<ti::JointData> joint_states_;
  std::vector<ti::JointData> joint_commands_;

  // Gravity vector
  std::vector<KDL::Vector> accel_vectors_;
  int accel_counter_;

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

