#ifndef BLUE_HARDWARE_INTERFACE_H
#define BLUE_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <geometry_msgs/Vector3.h>

#include "blue_hardware_interface/blue_transmissions.h"

#include "blue_hardware_drivers/BLDCDriver.h"
#include "blue_msgs/JointStartupCalibration.h"

namespace ti = transmission_interface;

class BlueHW: public hardware_interface::RobotHW
{
public:

  BlueHW(ros::NodeHandle &nh);
  void read();
  void write();

private:
  ros::NodeHandle nh_;

  // Motor driver interface
  blue_hardware_drivers::BLDCDriver motor_driver;

  template <typename TParam>
  TParam getParam(const std::string name, TParam& output);

};

#endif // BLUE_HARDWARE_INTERFACE
