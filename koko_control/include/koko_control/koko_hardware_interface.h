#include <ros/ros.h>
#include <vector>
#include <string>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <sensor_msgs/JointState.h>
#include <koko_hardware_drivers/MotorState.h>

using namespace transmission_interface;

class KokoHW: public hardware_interface::RobotHW
{

public:
  KokoHW(ros::NodeHandle &nh);

  void UpdateMotorState(const koko_hardware_drivers::MotorState::ConstPtr& msg);
  void CalibrateJointState(const sensor_msgs::JointState::ConstPtr& msg);
  double convertMotorTorqueToCurrent(double motor_torque, int index);
  ros::Time get_time();
  ros::Duration get_period();
  void read();
  void write();
  void PublishJointCommand();
  const int getPositionRead();

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_effort_interface;

  ros::Subscriber motor_state_subscriber;
  std::vector<ros::Publisher> motor_cmd_publishers;
  ros::Publisher debug_adv;
  ros::Subscriber jnt_state_tracker_subscriber;

  ros::Time last_time;
  std::vector<std::string> joint_names;
  std::vector<std::string> motor_names;

  std::vector<double> gear_ratios;
  std::vector<double> cmd;
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> eff;

  std::vector<double> current_slope;
  std::vector<double> current_offset;
  std::vector<int> paired_constraints;

  std::vector<double> motor_pos;
  std::vector<double> motor_vel;

  int position_read;
  int calibrated;
  std::vector<double> joint_state_initial;
  std::vector<double> joint_torque_directions;
  double hardstop_torque_limit;
  double i_to_T_slope;
  double i_to_T_intercept;
  int calibration_num;
  std::vector<double> angle_after_calibration;
  int is_calibrated;
  int prev_is_calibrated;
  double hardstop_eps;
  int num_joints;

  // adding transmissions
  // Transmission interfaces
  ActuatorToJointStateInterface act_to_jnt_state; // For motor to joint state
  JointToActuatorEffortInterface jnt_to_act_eff; // For joint eff to actuator

  //For arbitrary length
  std::vector<SimpleTransmission*> simple_transmissions;
  std::vector<DifferentialTransmission*> differential_transmissions;

  // Transmissions
  SimpleTransmission *base_trans;
  DifferentialTransmission *shoulder_trans;
  DifferentialTransmission *upper_arm_trans;
  DifferentialTransmission *wrist_trans;

  //Actuator and joint space variables
  std::vector<ActuatorData> a_state_data_vect;
  std::vector<ActuatorData> a_cmd_data_vect;

  std::vector<JointData> j_state_data_vect;
  std::vector<JointData> j_cmd_data_vect;

  // Actuator and joint variables
  std::vector<double> a_curr_pos_vect;
  std::vector<double> a_curr_vel_vect;
  std::vector<double> a_curr_eff_vect;
  std::vector<double> a_cmd_eff_vect;

  std::vector<double> j_curr_pos_vect;
  std::vector<double> j_curr_vel_vect;
  std::vector<double> j_curr_eff_vect;
  std::vector<double> j_cmd_eff_vect;

  std::vector<double> min_angles;
  std::vector<double> max_angles;
};
