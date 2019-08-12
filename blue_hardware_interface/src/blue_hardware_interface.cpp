#include "blue_hardware_interface/blue_hardware_interface.h"
#include "blue_hardware_interface/blue_kinematics.h"

#include <ros/assert.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

namespace ti = transmission_interface;

BlueHW::BlueHW(ros::NodeHandle &nh) : nh_(nh) {
  // Read robot parameters
  loadParams();

  // Acquire the motor_driver_ lock
  // std::lock_guard<std::mutex> lock(motor_driver_mutex_);

  // Motor driver bringup
  motor_driver_.init(
      params_.serial_port,
      params_.motor_ids);

  use_hardware_position_control = false;
  // motor_driver_.set_control_current_controller();
  // blue_hardware_interface::comm_ctrl_mode_t control_mode

  // Build helper for robot dynamics
  // (Used to compute gravity compensation torques)
  dynamics_.init(
      params_.robot_description,
      params_.baselink,
      params_.endlink);

  // Set up kinematics
  kinematics_.init(
      params_.joint_names,
      params_.differential_pairs,
      params_.gear_ratios);

  // Register joint interfaces with the controller manager
  registerInterface(&kinematics_.joint_state_interface);
  registerInterface(&kinematics_.joint_effort_interface);
  registerInterface(&kinematics_.joint_position_interface);

  // ROS publishers
  motor_states_msg_.name = params_.motor_names;
  motor_state_publisher_ = nh.advertise<blue_msgs::MotorState>(
      "blue_hardware/motor_states", 1);

  gravity_vectors_msg_.frame_ids = params_.accel_links;
  gravity_vector_publisher_ = nh.advertise<blue_msgs::GravityVectorArray>(
      "blue_hardware/gravity_vectors", 1);

  // Initialize motor commands
  for (auto id : params_.motor_ids) {
    motor_commands_[id] = 0.0;
    motor_pos_commands_[id] = 0.0;
  }

  bool success = false;
  while (!success && ros::ok()) {
    try {
      read();
      success = true;
    } catch (blue_hardware_drivers::comms_error e) {
      ROS_WARN("Failed to read motor states for initialization, retrying...");
      ros::Duration(0.5).sleep();
    }
  }

  // Calibration service
  joint_startup_calibration_service_ = nh.advertiseService(
      "blue_hardware/joint_startup_calibration",
      &BlueHW::jointStartupCalibration,
      this);
  gripper_position_calibration_service_ = nh.advertiseService(
      "blue_hardware/gripper_position_calibration",
      &BlueHW::gripperPositionCalibration,
      this);
}

void BlueHW::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                      const std::list<hardware_interface::ControllerInfo>& stop_list) {

  // TODO read these mappings in as a parameter
  std::vector<std::string> joint_hardware_effort_controllers{
    "effort_controllers/JointTrajectoryController",
    "effort_controllers/JointGroupEffortController",
    "blue_controllers/BlueJointGroupPositionController",
    "blue_controllers/BlueJointGroupCTC",
  };
  std::vector<std::string> joint_hardware_position_controllers{
    "position_controllers/JointGroupPositionController",
  };

  std::vector<std::string> gripper_hardware_effort_controllers{
    "effort_controllers/GripperActionController",
    "effort_controllers/JointGroupEffortController",
  };
  std::vector<std::string> gripper_hardware_position_controllers{
  };

  // Acquire the motor_driver lock
  std::lock_guard<std::mutex> lock(motor_driver_mutex_);


  for (std::list<hardware_interface::ControllerInfo>::const_iterator it=stop_list.begin(); it != stop_list.end(); ++it) {
    // if stopping a joint position hardware control mode, then switch back to current control mode for gravity comp
    if (std::find(
          std::begin(joint_hardware_position_controllers),
          std::end(joint_hardware_position_controllers),
          it->type) != std::end(joint_hardware_position_controllers)) {

      // do this for all except the last
      for (int i = 0; i < kinematics_.num_joints_ - 1; i++)
        motor_driver_.setControlMode(i, blue_hardware_drivers::COMM_CTRL_MODE_CURRENT);
    }

    if (std::find(
          std::begin(gripper_hardware_position_controllers),
          std::end(gripper_hardware_position_controllers),
          it->type) != std::end(gripper_hardware_position_controllers)) {
      motor_driver_.setControlMode(kinematics_.num_joints_ - 1, blue_hardware_drivers::COMM_CTRL_MODE_CURRENT);
    }
  }

  for (std::list<hardware_interface::ControllerInfo>::const_iterator it=start_list.begin(); it != start_list.end(); ++it) {
    if (std::find(
          std::begin(joint_hardware_position_controllers),
          std::end(joint_hardware_position_controllers),
          it->type) != std::end(joint_hardware_position_controllers)) {

      // do this for all except the last
      for (int i = 0; i < kinematics_.num_joints_ - 1; i++) {
        motor_driver_.setControlMode(i, blue_hardware_drivers::COMM_CTRL_MODE_POS_FF);
      }
    }

    if (std::find(
          std::begin(gripper_hardware_position_controllers),
          std::end(gripper_hardware_position_controllers),
          it->type) != std::end(gripper_hardware_position_controllers)) {
      motor_driver_.setControlMode(kinematics_.num_joints_ - 1, blue_hardware_drivers::COMM_CTRL_MODE_POS_FF);
    }
  }

}

void BlueHW::read() {
  // Motor communication! Simultaneously write commands and read state
  // motor_driver_.update(motor_commands_, motor_states_msg_);

  // Acquire the motor_driver_ lock
  std::lock_guard<std::mutex> lock(motor_driver_mutex_);


  motor_driver_.update(motor_pos_commands_, motor_commands_, motor_states_msg_);

  // Publish the motor states, in case anybody's listening
  motor_states_msg_.header.stamp = ros::Time::now();
  motor_state_publisher_.publish(motor_states_msg_);

  // Update kinematics with motor states
  kinematics_.setActuatorStates(motor_states_msg_);

  // Update orientation for gravity compensation
  kinematics_.getGravityVectors(gravity_vectors_msg_);
  std::array<double, 3> gravity_vector = {
    gravity_vectors_msg_.vectors[0].x,
    gravity_vectors_msg_.vectors[0].y,
    gravity_vectors_msg_.vectors[0].z
  };
  dynamics_.setGravityVector(gravity_vector);

  // Publish gravity vectors
  gravity_vectors_msg_.header.stamp = ros::Time::now();
  gravity_vector_publisher_.publish(gravity_vectors_msg_);
}

void BlueHW::write() {
  // Compute gravity compensation
  auto feedforward_torques = dynamics_.computeGravityComp(
      kinematics_.getJointPos(),
      kinematics_.getJointVel());

  // Apply gravity compensation fine tuning terms
  for (int i = 0; i < feedforward_torques.size(); i++)
    feedforward_torques[i] *= params_.id_torque_gains[i];

  // Get actuator commands, using the gravity comp torques as a feedforward
  // // pseudo code
  // if (position control interface is active) {
  //   // set commands to be torque commands 0
  //   for (int i = 0; i < raw_joint_cmd_.size(); i++)
  //     raw_joint_cmd_ = 0.0
  // }

  auto position_actuator_commands = kinematics_.getPositionActuatorCommands(
      params_.softstop_min_angles,
      params_.softstop_max_angles,
      params_.softstop_tolerance);

  auto actuator_commands = kinematics_.getActuatorCommands(
      feedforward_torques,
      params_.softstop_torque_limit, // TODO: clean up softstop code
      params_.softstop_min_angles,
      params_.softstop_max_angles,
      params_.softstop_tolerance);

  // Post-process motor commands
  for (int i = 0; i < actuator_commands.size(); i++) {
    // Convert torque to current
    // TODO: use driver internal torque control mode
    actuator_commands[i] = actuator_commands[i] * params_.current_to_torque_ratios[i];

    // Apply current limit
    // TODO: limit currents more if in position control mode
    actuator_commands[i] = std::max(
        std::min(actuator_commands[i], params_.motor_current_limits[i]),
        -params_.motor_current_limits[i]);

    // Update our command map
    motor_commands_[params_.motor_ids[i]] = actuator_commands[i];
    motor_pos_commands_[params_.motor_ids[i]] = position_actuator_commands[i];
  }
}

template <typename TParam>
void BlueHW::getParam(const std::string name, TParam& dest) {
  // Try to find a parameter and explode if it doesn't exist
  ROS_ASSERT_MSG(
      nh_.getParam(name, dest),
      "Could not find %s parameter in namespace %s",
      name.c_str(),
      nh_.getNamespace().c_str());
}

bool BlueHW::jointStartupCalibration(
    blue_msgs::JointStartupCalibration::Request &request,
    blue_msgs::JointStartupCalibration::Response &response
) {

  if (request.disable_snap) {
    // Set joint offsets naively
    kinematics_.setJointOffsets(request.joint_positions);
  } else {
    // Set joint offsets to closest feasible values, based on actuator zeros
    //
    // This relies on the fact that we have a ~7:1 reduction, and absolute position
    // sensors on the actuators
    kinematics_.setJointOffsets(
        kinematics_.snapJointOffsets(
            request.joint_positions,
            params_.actuator_zeros,
            params_.softstop_min_angles,
            params_.softstop_max_angles));
  }

  response.success = true;
  return true;
}

bool BlueHW::gripperPositionCalibration(
    blue_msgs::GripperPositionCalibration::Request &request,
    blue_msgs::GripperPositionCalibration::Response &response
) {
  // Set the current position of the gripper
  kinematics_.setGripperPosition(request.position);

  response.success = true;
  return true;
}

void BlueHW::loadParams() {
  // Motor driver stuff
  getParam("blue_hardware/serial_port", params_.serial_port);
  getParam("blue_hardware/motor_names", params_.motor_names);
  std::vector<int> temp_motor_ids;
  getParam("blue_hardware/motor_ids", temp_motor_ids);
  for (int id : temp_motor_ids)
    params_.motor_ids.push_back(id);

  // Parameters for parsing URDF
  getParam("robot_description", params_.robot_description);
  getParam("blue_hardware/baselink", params_.baselink);
  getParam("blue_hardware/endlink", params_.endlink);

  // Read data needed for kinematics
  getParam("blue_hardware/joint_names", params_.joint_names);
  getParam("blue_hardware/gear_ratios", params_.gear_ratios);
  getParam("blue_hardware/differential_pairs", params_.differential_pairs);

  // Torque => current conversion stuff
  getParam("blue_hardware/motor_current_limits", params_.motor_current_limits);
  getParam("blue_hardware/current_to_torque_ratios", params_.current_to_torque_ratios);

  // Gravity compensation tuning
  getParam("blue_hardware/id_torque_gains", params_.id_torque_gains);

  // Soft stops
  // TODO: hacky and temporary
  getParam("blue_hardware/softstop_torque_limit", params_.softstop_torque_limit);
  getParam("blue_hardware/softstop_min_angles", params_.softstop_min_angles);
  getParam("blue_hardware/softstop_max_angles", params_.softstop_max_angles);
  getParam("blue_hardware/softstop_tolerance", params_.softstop_tolerance);

  // Links to attach accelerometer measurements to
  getParam("blue_hardware/accel_links", params_.accel_links);

  // Calibration
  getParam("blue_hardware/actuator_zeros", params_.actuator_zeros);
}
