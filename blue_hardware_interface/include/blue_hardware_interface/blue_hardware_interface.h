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

typedef struct {
  // Motor driver stuff
  std::string serial_port;
  std::vector<uint8_t> motor_ids;
  std::vector<std::string> motor_names;

  // Parameters for parsing URDF
  std::string robot_description;
  std::string baselink;
  std::string endlink;

  // Read data needed for transmissions
  std::vector<std::string> joint_names;
  std::vector<int> differential_pairs;
  std::vector<double> gear_ratios;

  // Torque => current conversion stuff
  std::vector<double> current_to_torque_ratios;
  std::vector<double> motor_current_limits;
} Params;

class BlueHW: public hardware_interface::RobotHW
{
public:

  BlueHW(ros::NodeHandle &nh);
  void read();
  void write();

private:
  ros::NodeHandle nh_;

  // Configuration from parameter server
  Params params_;

  // Motor driver interface
  blue_hardware_drivers::BLDCDriver motor_driver_;
  std::map<uint8_t, float> motor_commands_;

  // Transmission abstraction layer (actuator <-> joint)
  BlueTransmissions transmissions_;

  // Robot dynamics helper
  BlueDynamics dynamics_;

  // ROS stuff
  blue_msgs::MotorState motor_states_;
  ros::Publisher motor_state_publisher_;

  // Helpers for reading params
  template <typename TParam>
  void getParam(const std::string name, TParam& dest);
  void loadParams();

};

#endif // BLUE_HARDWARE_INTERFACE
