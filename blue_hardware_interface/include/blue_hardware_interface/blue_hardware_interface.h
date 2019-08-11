#ifndef BLUE_HARDWARE_INTERFACE_H
#define BLUE_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <geometry_msgs/Vector3.h>

#include "blue_hardware_interface/blue_kinematics.h"
#include "blue_hardware_interface/blue_dynamics.h"

#include "blue_hardware_drivers/BLDCDriver.h"
#include "blue_msgs/JointStartupCalibration.h"
#include "blue_msgs/GripperPositionCalibration.h"
#include "blue_msgs/MotorState.h"
#include "blue_msgs/GravityVectorArray.h"

typedef struct {
  // Motor driver stuff
  std::string serial_port;
  std::vector<uint8_t> motor_ids;
  std::vector<std::string> motor_names;

  // Parameters for parsing URDF
  std::string robot_description;
  std::string baselink;
  std::string endlink;

  // Read data needed for kinematics
  std::vector<std::string> joint_names;
  std::vector<int> differential_pairs;
  std::vector<double> gear_ratios;

  // Torque => current conversion stuff
  std::vector<double> current_to_torque_ratios;
  std::vector<double> motor_current_limits;

  // Gains for our inverse dynamics torques (gravity compensation tuning)
  std::vector<double> id_torque_gains;

  // Soft stops
  // TODO: hacky and temporary
  double softstop_torque_limit;
  std::vector<double> softstop_min_angles;
  std::vector<double> softstop_max_angles;
  double softstop_tolerance;

  // Links to attach accelerometer measurements to
  std::vector<std::string> accel_links;

  // Calibration parameters
  std::vector<double> actuator_zeros;
} Params;

class BlueHW: public hardware_interface::RobotHW
{
public:

  BlueHW(ros::NodeHandle &nh);
  void read();
  void write();
  void doSwitch(const std::list<ControllerInfo>& start_list,
                const std::list<ControllerInfo>& stop_list);

private:
  ros::NodeHandle nh_;

  // Configuration from parameter server
  Params params_;

  // Motor driver interface
  blue_hardware_drivers::BLDCDriver motor_driver_;
  std::unordered_map<uint8_t, float> motor_commands_;

  // Kinematics abstraction layer (actuator <-> joint)
  BlueKinematics kinematics_;

  // Robot dynamics helper
  BlueDynamics dynamics_;

  // Publishers
  blue_msgs::MotorState motor_states_msg_;
  ros::Publisher motor_state_publisher_;
  blue_msgs::GravityVectorArray gravity_vectors_msg_;
  ros::Publisher gravity_vector_publisher_;

  // Calibration services
  bool jointStartupCalibration(
    blue_msgs::JointStartupCalibration::Request &request,
    blue_msgs::JointStartupCalibration::Response &response
  );
  bool gripperPositionCalibration(
    blue_msgs::GripperPositionCalibration::Request &request,
    blue_msgs::GripperPositionCalibration::Response &response
  );
  ros::ServiceServer joint_startup_calibration_service_;
  ros::ServiceServer gripper_position_calibration_service_;

  // Helpers for reading params
  template <typename TParam>
  void getParam(const std::string name, TParam& dest);
  void loadParams();

};

#endif // BLUE_HARDWARE_INTERFACE
