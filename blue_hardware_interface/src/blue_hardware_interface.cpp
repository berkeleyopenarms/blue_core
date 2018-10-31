#include "blue_hardware_interface/blue_hardware_interface.h"
#include "blue_hardware_interface/blue_transmissions.h"
#include "blue_msgs/MotorState.h"

#include <ros/assert.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

namespace ti = transmission_interface;

BlueHW::BlueHW(ros::NodeHandle &nh) : nh_(nh) {

  // Read robot parameters
  loadParams();

  // Motor driver bringup
  motor_driver_.init(
      params_.serial_port,
      params_.motor_ids);

  // Build helper for robot dynamics
  // (Used to compute gravity compensation torques)
  dynamics_.init(
      params_.robot_description,
      params_.baselink,
      params_.endlink);

  // Set up transmissions
  transmissions_.init(
      params_.joint_names,
      params_.differential_pairs,
      params_.gear_ratios);

  // Register joint interfaces with the controller manager
  registerInterface(&transmissions_.joint_state_interface);
  registerInterface(&transmissions_.joint_effort_interface);

  // ROS communications setup
  motor_states_.name = params_.motor_names;
  motor_state_publisher_ = nh.advertise<blue_msgs::MotorState>(
      "blue_hardware/motor_states", 1);

  // Wait for the dust to settle a bit before we actually start :)
  ros::Duration(0.444).sleep();
}

void BlueHW::read() {
  // Motor communication! Simultaneously write commands and read state
  motor_driver_.update(motor_commands_, motor_states_);

  // Publish the motor states, in case anybody's listening
  motor_states_.header.stamp = ros::Time::now();
  motor_state_publisher_.publish(motor_states_);


}

void BlueHW::write() {
  // Compute gravity compensation
  auto gravity_comp_torques = dynamics_.computeGravityComp(
      transmissions_.getJointPos(),
      transmissions_.getJointVel());

  // Set gripper gravity compensation torque to 0
  gravity_comp_torques.back() = 0.0;

  // Compute motor commands from actuator commands
  auto actuator_commands = transmissions_.getActuatorCommands(gravity_comp_torques);

  // Post-process motor commands
  for (int i = 0; i < actuator_commands.size(); i++) {
    // Convert torque to current
    actuator_commands[i] = actuator_commands[i] * params_.current_to_torque_ratios[i];

    // Apply current limit
    actuator_commands[i] = std::max(
        std::min(actuator_commands[i], params_.motor_current_limits[i]),
        -params_.motor_current_limits[i]);

    // Update our command map
    motor_commands_[params_.motor_ids[i]] = actuator_commands[i];
  }
}

template <typename TParam>
void BlueHW::getParam(const std::string name, TParam& dest) {
  ROS_ASSERT_MSG(
    nh_.getParam(name, dest),
    "Could not find %s parameter in namespace %s",
    name.c_str(),
    nh_.getNamespace().c_str()
  );
}

void BlueHW::loadParams() {
  // Motor driver stuff
  getParam("blue_hardware/serial_port", params_.serial_port);
  getParam("blue_hardware/motor_names", params_.motor_names);
  std::vector<int> temp_motor_ids;
  getParam("blue_hardware/motor_ids", temp_motor_ids);
  for (auto id : temp_motor_ids)
    params_.motor_ids.push_back(id);

  // Parameters for parsing URDF
  getParam("robot_description", params_.robot_description);
  getParam("baselink", params_.baselink);
  getParam("endlink", params_.endlink);

  // Read data needed for transmissions
  getParam("blue_hardware/joint_names", params_.joint_names);
  getParam("blue_hardware/gear_ratios", params_.gear_ratios);
  getParam("blue_hardware/differential_pairs", params_.differential_pairs);

  // Torque => current conversion stuff
  getParam("blue_hardware/motor_current_limits", params_.motor_current_limits);
  getParam("blue_hardware/current_to_torque_ratios", params_.current_to_torque_ratios);
}
