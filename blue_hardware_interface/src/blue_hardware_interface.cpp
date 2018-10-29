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
};

void BlueHW::loadParams(Params &params) {
  // Read the serial port path, eg /dev/ttyUSB0
  getParam("blue_hardware/serial_port", params.serial_port);

  // Read the motor ids, which need to be cast int->uint8_t
  std::vector<int> temp_motor_ids;
  getParam("blue_hardware/motor_ids", temp_motor_ids);
  for (auto id : temp_motor_ids)
    params.motor_ids.push_back(id);

  //
}

BlueHW::BlueHW(ros::NodeHandle &nh) : nh_(nh) {

  // Read robot parameters
  Params params;
  loadParams(params);

  // Motor driver bringup
  motor_driver.init(
      params.serial_port,
      params.motor_ids);

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
void BlueHW::getParam(const std::string name, TParam& dest) {
  ROS_ASSERT_MSG(
    nh_.getParam(name, dest),
    "Could not find %s parameter in namespace %s",
    name.c_str(),
    nh_.getNamespace().c_str()
  );
}

