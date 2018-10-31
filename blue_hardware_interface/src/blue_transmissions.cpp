#include "blue_hardware_interface/blue_hardware_interface.h"
#include "blue_hardware_interface/blue_transmissions.h"

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
#include <transmission_interface/transmission_interface.h>

#include <string.h>

BlueTransmissions::BlueTransmissions() {}

void BlueTransmissions::init(
      std::vector<std::string> joint_names,
      std::vector<int> differential_pairs,
      std::vector<double> gear_ratios) {

  // How many joints do we have?
  int num_joints = joint_names.size();

  // How many transmissions?
  int num_simple_transmissions = num_joints - differential_pairs.size();
  int num_diff_transmissions = differential_pairs.size() / 2;
  int num_transmissions = num_simple_transmissions + num_diff_transmissions;

  // Neat, now let's make the correct amount of space for objects/raw data
  transmissions_.resize(num_transmissions);
  actuator_states_.resize(num_transmissions);
  actuator_commands_.resize(num_transmissions);
  joint_states_.resize(num_transmissions);
  joint_commands_.resize(num_transmissions);

  actuator_pos_.resize(num_joints);
  actuator_vel_.resize(num_joints);
  actuator_eff_.resize(num_joints);
  actuator_cmd_.resize(num_joints);
  joint_pos_.resize(num_joints);
  joint_vel_.resize(num_joints);
  joint_eff_.resize(num_joints);
  joint_cmd_.resize(num_joints);

  raw_joint_cmd_.resize(num_joints);

  // Build each of our transmission objects
  int joint_idx = 0;
  for(int transmission_idx = 0; transmission_idx < num_transmissions; transmission_idx++){
    // How many joints are in this transmission?
    int joints_in_transmission;

    // Is this part of a differential pair?
    auto tmp = std::find(differential_pairs.begin(), differential_pairs.end(), joint_idx);
    if(tmp != differential_pairs.end()){
      // Differential transmissions (ie links, two joints each)
      std::vector<double> transmission_actuator_ratios{-1.0, 1.0};
      std::vector<double> transmission_gear_ratios{
          -gear_ratios[joint_idx],
          -gear_ratios[joint_idx + 1]};

      transmissions_[transmission_idx] = new ti::DifferentialTransmission(
          transmission_actuator_ratios,
          transmission_gear_ratios);
      joints_in_transmission = 2;
    } else {
      // Not differential pair (base or gripper, one joint each)
      transmissions_[transmission_idx] = new ti::SimpleTransmission(
          -gear_ratios[joint_idx],
          0.0);
      joints_in_transmission = 1;
    }

    // Wrap raw data for each joint in the transmission
    for (int i = 0; i < joints_in_transmission; i++) {
      actuator_states_[transmission_idx].position.push_back(&actuator_pos_[joint_idx]);
      actuator_states_[transmission_idx].velocity.push_back(&actuator_vel_[joint_idx]);
      actuator_states_[transmission_idx].effort.push_back(&actuator_eff_[joint_idx]);

      joint_states_[transmission_idx].position.push_back(&joint_pos_[joint_idx]);
      joint_states_[transmission_idx].velocity.push_back(&joint_vel_[joint_idx]);
      joint_states_[transmission_idx].effort.push_back(&joint_eff_[joint_idx]);

      actuator_commands_[transmission_idx].effort.push_back(&actuator_cmd_[joint_idx]);
      joint_commands_[transmission_idx].effort.push_back(&joint_cmd_[joint_idx]);

      joint_idx++;
    }

    // Name the transmission...
    std::ostringstream stream;
    stream << "transmission_" << transmission_idx;
    stream << (joints_in_transmission == 1 ? "_simple" : "_differential");
    std::string transmission_name = stream.str();
    // ...and then register it
    actuator_to_joint_interface_.registerHandle(
        ti::ActuatorToJointStateHandle(transmission_name,
        transmissions_[transmission_idx],
        actuator_states_[transmission_idx],
        joint_states_[transmission_idx]));
    joint_to_actuator_interface_.registerHandle(
        ti::JointToActuatorEffortHandle(transmission_name,
        transmissions_[transmission_idx],
        actuator_commands_[transmission_idx],
        joint_commands_[transmission_idx]));

  }

  // Build interfaces for ros_control
  for (int i = 0; i < num_joints; i++) {
    joint_cmd_[i] = 0.0;
    raw_joint_cmd_[i] = 0.0;
    hardware_interface::JointStateHandle state_handle_a(
        joint_names[i],
        &joint_pos_[i],
        &joint_vel_[i],
        &joint_eff_[i]);
    joint_state_interface.registerHandle(state_handle_a);
  }
  for (int i = 0; i < num_joints; i++) {
    hardware_interface::JointHandle effort_handle_a(
        joint_state_interface.getHandle(joint_names[i]),
        &raw_joint_cmd_[i]);
    joint_effort_interface.registerHandle(effort_handle_a);
  }

}

