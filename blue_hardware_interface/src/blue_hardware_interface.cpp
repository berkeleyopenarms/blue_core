#include "blue_hardware_interface/blue_hardware_interface.h"
#include "blue_hardware_interface/blue_transmissions.h"
#include "blue_msgs/MotorState.h"

#include <ros/assert.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

namespace ti = transmission_interface;

struct Params{
  std::string serial_port;
  std::vector<uint8_t> motor_ids;

  std::string robot_description;

  std::vector<std::string> joint_names;
  std::vector<int> differential_pairs;
  std::vector<double> gear_ratios;
};

void BlueHW::loadParams(Params &params) {
  // Read the serial port path, eg /dev/ttyUSB0
  getParam("blue_hardware/serial_port", params.serial_port);

  // Read the motor ids, which need to be cast int->uint8_t
  std::vector<int> temp_motor_ids;
  getParam("blue_hardware/motor_ids", temp_motor_ids);
  for (auto id : temp_motor_ids)
    params.motor_ids.push_back(id);

  // Read URDF
  getParam("robot_description", params.robot_description);

  // Read data needed for transmissions
  getParam("blue_hardware/joint_names", params.joint_names);
  getParam("blue_hardware/gear_ratios", params.gear_ratios);
  getParam("blue_hardware/differential_pairs", params.differential_pairs);

}

BlueHW::BlueHW(ros::NodeHandle &nh) : nh_(nh) {

  // Read robot parameters
  Params params;
  loadParams(params);

  // Motor driver bringup
  motor_driver_.init(
      params.serial_port,
      params.motor_ids);

  // Build robot dynamics helper
  dynamics_.init(params.robot_description);

  // Set up transmissions
  transmissions_.init(
      params.joint_names,
      params.differential_pairs,
      params.gear_ratios);

  // Register joint state/effort handles
  registerInterface(&transmissions_.joint_state_interface);
  registerInterface(&transmissions_.joint_effort_interface);

  // Set up publishers and subscribers
}

void BlueHW::read() {
  // Read from motors

}

void BlueHW::write() {
  // Compute gravity compensation

  // Propagate actuator torques to motor torques[

  // Update motor commands
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

