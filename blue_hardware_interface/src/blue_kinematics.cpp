#include "blue_hardware_interface/blue_hardware_interface.h"
#include "blue_hardware_interface/blue_kinematics.h"

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
#include <transmission_interface/transmission_interface.h>

#include <string.h>
#include <cmath>

BlueKinematics::BlueKinematics() :
  is_calibrated_(false) {}

void BlueKinematics::init(
      const std::vector<std::string> &joint_names,
      const std::vector<int> &differential_pairs,
      const std::vector<double> &gear_ratios) {

  // How many joints do we have?
  num_joints_ = joint_names.size();

  // How many transmissions?
  num_simple_transmissions_ = num_joints_ - differential_pairs.size();
  num_diff_transmissions_ = differential_pairs.size() / 2;
  num_transmissions_ = num_simple_transmissions_ + num_diff_transmissions_;

  // Neat, now let's make the correct amount of space for objects/raw data
  transmissions_.resize(num_transmissions_);
  actuator_states_.resize(num_transmissions_);
  actuator_commands_.resize(num_transmissions_);
  joint_states_.resize(num_transmissions_);
  joint_commands_.resize(num_transmissions_);

  actuator_pos_.resize(num_joints_);
  actuator_vel_.resize(num_joints_);
  actuator_eff_.resize(num_joints_);
  actuator_cmd_.resize(num_joints_);
  joint_pos_.resize(num_joints_);
  joint_vel_.resize(num_joints_);
  joint_eff_.resize(num_joints_);
  joint_cmd_.resize(num_joints_);

  joint_offsets_.resize(num_joints_, 0.0);
  raw_joint_cmd_.resize(num_joints_, 0.0);

  KDL::Vector zero_vector(0.0, 0.0, 0.0);
  accel_vectors_.resize(num_transmissions_, zero_vector);
  accel_counter_ = 1;

  // Build each of our transmission objects
  int joint_idx = 0;
  for (int transmission_idx = 0; transmission_idx < num_transmissions_; transmission_idx++){
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
  for (int i = 0; i < num_joints_; i++) {
    joint_cmd_[i] = 0.0;
    raw_joint_cmd_[i] = 0.0;
    hardware_interface::JointStateHandle state_handle_a(
        joint_names[i],
        &joint_pos_[i],
        &joint_vel_[i],
        &joint_eff_[i]);
    joint_state_interface.registerHandle(state_handle_a);
  }
  for (int i = 0; i < num_joints_; i++) {
    hardware_interface::JointHandle effort_handle_a(
        joint_state_interface.getHandle(joint_names[i]),
        &raw_joint_cmd_[i]);
    joint_effort_interface.registerHandle(effort_handle_a);
  }

}

void BlueKinematics::setJointOffsets(
    const std::vector<double> &offsets) {
  joint_offsets_ = offsets;
  is_calibrated_ = true;
}

const std::vector<double>& BlueKinematics::getJointPos() {
  return joint_pos_;
}

const std::vector<double>& BlueKinematics::getJointVel() {
  return joint_vel_;
}

void BlueKinematics::setActuatorStates(
    const std::vector<double> &positions,
    const std::vector<double> &velocities,
    const std::vector<double> &efforts,
    const std::vector<double> &accel_x,
    const std::vector<double> &accel_y,
    const std::vector<double> &accel_z) {

  actuator_pos_ = positions;
  actuator_vel_ = velocities;
  actuator_eff_ = efforts;

  // Set actuator offsets so all positions are zero on startup
  while (actuator_offsets_.size() < actuator_pos_.size())
    actuator_offsets_.push_back(-actuator_pos_[actuator_offsets_.size()]);

  // Apply actuator offsets
  for (int i = 0; i < actuator_pos_.size(); i++)
    actuator_pos_[i] += actuator_offsets_[i];

  actuator_to_joint_interface_.propagate();

  // Bias joint positions by calibrated offset
  for (int i = 0; i < num_joints_; i++)
    joint_pos_[i] += joint_offsets_[i];


  // Compute a gravity vector for each link/transmission
  // TODO: fix this, it's pretty hacky
  int actuator_idx = 0;
  for (int transmission_idx = 0; transmission_idx < num_transmissions_; transmission_idx++) {

    KDL::Vector accel_vector;
    accel_vector(0) = accel_x[actuator_idx];
    accel_vector(1) = accel_y[actuator_idx];
    accel_vector(2) = accel_z[actuator_idx];
    accel_vector = accel_vector / accel_vector.Norm() * 9.81;

    if (transmissions_[transmission_idx]->numActuators() == 2) {
      // Differential link

      // Load left reading
      KDL::Vector accel_vector_left;
      accel_vector_left(0) = accel_x[actuator_idx + 1];
      accel_vector_left(1) = accel_y[actuator_idx + 1];
      accel_vector_left(2) = accel_z[actuator_idx + 1];

      // Rotate left reading to match frame of right
      KDL::Rotation left_rot_z;
      left_rot_z.DoRotZ(M_PI);
      accel_vector_left = left_rot_z * accel_vector_left;

      KDL::Rotation left_rot_x;
      left_rot_x.DoRotX(M_PI);
      accel_vector_left = left_rot_x * accel_vector_left;

      // Average left and right readings
      accel_vector = (accel_vector + accel_vector_left) / 2.0;

      // Apply link -> accelerometer transform to averaged reading
      KDL::Rotation link_rot_z;
      link_rot_z.DoRotZ(-0.3378);

      KDL::Rotation link_rot_y;
      link_rot_y.DoRotY(-M_PI / 2.0);

      KDL::Rotation link_rot_z2;
      link_rot_z2.DoRotZ(M_PI / 2.0);

      accel_vector = link_rot_z2 * link_rot_y * link_rot_z * accel_vector;

      // Flip axes
      accel_vector(0) = -accel_vector(0);
      accel_vector(2) = -accel_vector(2);

      actuator_idx += 2;
    } else if (transmission_idx == 0) {
      // Base link
      KDL::Rotation base_rot_z;
      base_rot_z.DoRotZ(-4.301 - M_PI);
      accel_vector = base_rot_z * accel_vector;
      accel_vector(0) = -accel_vector(0);
      accel_vector(1) = -accel_vector(1);

      actuator_idx++;
    } else if (transmission_idx == num_transmissions_ - 1) {
      // Gripper link
      KDL::Rotation grip_rot_z;
      grip_rot_z.DoRotZ(M_PI / 2.0);
      accel_vector = grip_rot_z * accel_vector;
      accel_vector(0) = -accel_vector(0);
      accel_vector(2) = -accel_vector(2);

      actuator_idx++;
    }

    // Exponential decay to smooth values + bias correction
    double alpha = 0.99;
    accel_vectors_[transmission_idx] =
      (accel_vectors_[transmission_idx] * alpha - accel_vector * (1 - alpha))
      / (1 - pow(alpha, accel_counter_));

  }
  accel_counter_++;
}

std::vector<double> BlueKinematics::getGravityVector() {
  KDL::Vector scaled = accel_vectors_[0] / accel_vectors_[0].Norm() * 9.81;
  std::vector<double> output(scaled.data, scaled.data + 3);
  return output;
}

std::vector<double> BlueKinematics::getActuatorCommands(
    const std::vector<double> &feedforward_torques,
    double softstop_torque_limit, // TODO: clean up softstop code
    const std::vector<double> &softstop_min_angles,
    const std::vector<double> &softstop_max_angles,
    double softstop_tolerance) {

  if (!is_calibrated_) {
    // If not calibrated, zero out all actuator commands
    std::fill(actuator_cmd_.begin(), actuator_cmd_.end(), 0);
    return actuator_cmd_;
  }

  // Compute joint commands
  for (int i = 0; i < num_joints_; i++) {
    // Add feedforward if it exists
    if (i < feedforward_torques.size())
      joint_cmd_[i] = raw_joint_cmd_[i] + feedforward_torques[i];
    else
      joint_cmd_[i] = raw_joint_cmd_[i];

    raw_joint_cmd_[i] = 0.0;

    // Soft stops
    // TODO: hacky and temporary
    if(joint_pos_[i] > softstop_max_angles[i] - softstop_tolerance){
      ROS_WARN_THROTTLE(1, "Going over soft stop max, %d", i);
      double offset = joint_pos_[i] - softstop_max_angles[i] + softstop_tolerance;
      joint_cmd_[i] += -1.0 * softstop_torque_limit * pow(offset, 2);
    } else if (joint_pos_[i] < softstop_min_angles[i] + softstop_tolerance){
      ROS_WARN_THROTTLE(1, "Going over soft stop min, %d",i);
      double offset = softstop_min_angles[i] + softstop_tolerance - joint_pos_[i];
      joint_cmd_[i] += softstop_torque_limit * pow(offset, 2);
    }
  }

  // Propagate through transmissions to compute actuator commands
  joint_to_actuator_interface_.propagate();

  return actuator_cmd_;
}
