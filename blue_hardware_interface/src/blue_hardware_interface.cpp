#include "blue_hardware_interface/blue_hardware_interface.h"
#include "blue_msgs/MotorState.h"

#include <ros/assert.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

namespace ti = transmission_interface;

BlueHW::BlueHW(ros::NodeHandle &nh) : nh_(nh) {

  // Read the motor ids
  std::vector<uint8_t> motor_ids;
  std::vector<int> temp_motor_ids;
  getParam("blue_hardware/motor_ids", temp_motor_ids);
  for (auto id : temp_motor_ids)
    motor_ids.push_back(id);

  blue_msgs::MotorState motor_states_;

  // Bring up motors
  std::string serial_port;
  getParam("blue_hardware/serial_port", serial_port);
  motor_driver.init(motor_ids, serial_port);

  // ROS_ASSERT(nh.getParam("blue_hardware/serial_port", ));

  // Set up transmissions

  // Gravity Compensation

  // Register joint state/effort handles

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
TParam BlueHW::getParam(const std::string name, TParam& output) {
  ROS_ASSERT_MSG(
    nh_.getParam(name, output),
    "Could not find %s parameter in namespace %s",
    name.c_str(),
    nh_.getNamespace().c_str()
  );
}

