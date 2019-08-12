#ifndef BLUE_TRANSMISSIONS_H
#define BLUE_TRANSMISSIONS_H

#include "blue_msgs/MotorState.h"
#include "blue_msgs/GravityVectorArray.h"

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
#include <transmission_interface/transmission_interface.h>

#include <kdl/frames.hpp>

#include <mutex>

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
  hardware_interface::PositionJointInterface joint_position_interface;

  // Getter functions
  const std::vector<double>& getJointPos();
  const std::vector<double>& getJointVel();
  size_t getJointCount();

  // Joint calibration - snap estimated offsets to closest feasible values
  std::vector<double> snapJointOffsets(
      const std::vector<double> &estimated_joint_offsets,
      const std::vector<double> &actuator_zeros,
      const std::vector<double> &softstop_min_angles,
      const std::vector<double> &softstop_max_angles);

  // Joint calibration
  void setJointOffsets(
      const std::vector<double> &offsets);
  void setGripperPosition(double position);

  //  Update internal actuator states
  void setActuatorStates(
      const blue_msgs::MotorState &motor_msg);

  // Which way is down??
  void getGravityVectors(
      blue_msgs::GravityVectorArray &grav_msg);

  // Get desired actuator commands
  std::vector<double> getActuatorCommands(
      const std::vector<double> &feedforward_torques,
      double softstop_torque_limit, // TODO: clean up softstop code
      const std::vector<double> &softstop_min_angles,
      const std::vector<double> &softstop_max_angles,
      double softstop_tolerance);

  std::vector<double> getPositionActuatorCommands(
      const std::vector<double> &softstop_min_angles,
      const std::vector<double> &softstop_max_angles,
      double softstop_tolerance);

private:

  // Private helpers
  void updateTransmissions(
      const std::vector<double> &actuator_pos,
      const std::vector<double> &actuator_vel);

  void updateAccelerometers(
      const std::vector<double> &accel_x,
      const std::vector<double> &accel_y,
      const std::vector<double> &accel_z);

  // Calibration state (zero commands until arm is calibrated)
  bool is_calibrated_;

  // Counters
  size_t num_joints_;
  size_t num_diff_actuators_;
  size_t num_simple_transmissions_;
  size_t num_diff_transmissions_;
  size_t num_transmissions_;

  // Only one thread should be accessing the robot kinematics at a time
  // TODO: optimize or rethink this?
  std::mutex kinematics_mutex_;

  // Transmission interfaces
  ti::ActuatorToJointStateInterface actuator_to_joint_interface_;
  ti::JointToActuatorEffortInterface joint_to_actuator_interface_;
  ti::JointToActuatorPositionInterface position_joint_to_actuator_interface_;

  // Joint offsets
  std::vector<double> actuator_offsets_;
  std::vector<double> joint_offsets_;

  // Actuator and joint state data
  std::vector<ti::Transmission *> transmissions_;
  std::vector<ti::ActuatorData> actuator_states_;
  std::vector<ti::ActuatorData> actuator_commands_;
  std::vector<ti::ActuatorData> actuator_position_commands_;
  std::vector<ti::JointData> joint_states_;
  std::vector<ti::JointData> joint_commands_;
  std::vector<ti::JointData> joint_position_commands_;

  // Gravity vector
  std::vector<KDL::Vector> accel_vectors_;

  // Data owned by our ActuatorData and JointData objects
  // Also shared by the joint state/effort interfaces
  std::vector<double> actuator_pos_;
  std::vector<double> actuator_vel_;
  std::vector<double> actuator_eff_;
  std::vector<double> actuator_eff_cmd_;
  std::vector<double> actuator_pos_cmd_;
  std::vector<double> joint_pos_;
  std::vector<double> joint_vel_;
  std::vector<double> joint_eff_;
  std::vector<double> joint_eff_cmd_;
  std::vector<double> joint_pos_cmd_;

  std::vector<double> raw_joint_eff_cmd_;
  std::vector<double> raw_joint_pos_cmd_;

};

#endif // BLUE_TRANSMISSIONS

