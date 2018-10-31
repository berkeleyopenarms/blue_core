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
#include "blue_hardware_interface/blue_dynamics.h"

#include "blue_hardware_drivers/BLDCDriver.h"
#include "blue_msgs/JointStartupCalibration.h"

typedef struct Params Params;

class BlueHW: public hardware_interface::RobotHW
{
public:

  BlueHW(ros::NodeHandle &nh);
  void read();
  void write();

private:
  ros::NodeHandle nh_;

  // Motor driver interface
  blue_hardware_drivers::BLDCDriver motor_driver_;

  // Transmission abstraction layer (actuator <-> joint)
  BlueTransmissions transmissions_;

  // Robot dynamics helper
  BlueDynamics dynamics_;

  template <typename TParam>
  void getParam(const std::string name, TParam& dest);
  void loadParams(Params &params);

};

#endif // BLUE_HARDWARE_INTERFACE
