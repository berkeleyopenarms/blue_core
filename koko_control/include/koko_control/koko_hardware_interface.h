#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <ros/node_handle.h>
#include <vector>
#include <string>
#include <math.h>
#include <koko_hardware_drivers/BLDCDriver.h>

class KokoHW: public hardware_interface::RobotHW
{

public:
  KokoHW(ros::NodeHandle &nh)
  {

    if (!nh.getParam("joint_names", joint_names)) {
      ROS_ERROR("No joint_names given (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!nh.getParam("motor_names", motor_names)) {
      ROS_ERROR("No motor_names given (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!nh.getParam("gear_ratio", gear_ratios)) {
      ROS_ERROR("No gear_ratio given (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!nh.getParam("directions", directions)) {
      ROS_ERROR("No gear_ratio given (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!nh.getParam("torque_directions", torque_directions)) {
      ROS_ERROR("No gear_ratio given (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!nh.getParam("current_slope", current_slope)) {
      ROS_ERROR("No current_slope given (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!nh.getParam("current_offset", current_offset)) {
      ROS_ERROR("No current_offset given (namespace: %s)", nh.getNamespace().c_str());
    }

    if (!nh.getParam("simple_controller/paired_constraints", paired_constraints)) {
      ROS_ERROR("No simple_controller/paired_constraints given (namespace: %s", nh.getNamespace().c_str());
    }

    if (paired_constraints.size() % 2 != 0) {
      ROS_ERROR("Paired_constraints length must be even");
    }
    num_joints = joint_names.size();
    motor_pos.resize(num_joints, 0.0);
    motor_vel.resize(num_joints, 0.0);
    motor_eff.resize(num_joints, 0.0);
    motor_cmd.resize(num_joints, 0.0);
    cmd.resize(num_joints, 0.0);
    pos.resize(num_joints, 0.0);
    vel.resize(num_joints, 0.0);
    eff.resize(num_joints, 0.0);
    joint_state_initial.resize(num_joints, 0.0);
    angle_after_calibration.resize(num_joints, 0.0);



    for (int i = 0; i < num_joints; i++) {
      cmd[i] = 0.0;
      hardware_interface::JointStateHandle state_handle_a(joint_names[i], &pos[i], &vel[i], &eff[i]);
      //ROS_INFO("joint %s", joint_names[i].c_str());
      jnt_state_interface.registerHandle(state_handle_a);
    }
    ROS_ERROR("c1");
    motor_driver.init( &motor_pos, &motor_vel, &motor_eff, &motor_cmd );
    ROS_ERROR("c2");

    registerInterface(&jnt_state_interface);

    for (int i = 0; i < num_joints; i++) {
      hardware_interface::JointHandle effort_handle_a(jnt_state_interface.getHandle(joint_names[i]), &cmd[i]);
      jnt_effort_interface.registerHandle(effort_handle_a);
    }
    registerInterface(&jnt_effort_interface);
    ROS_ERROR("c3");

    position_read = 0;
    calibrated = 0;
    prev_is_calibrated = 0;
    is_calibrated = 0;

    for (int i = 0; i < num_joints; i++) {
      joint_state_initial[i] = 0.0;
    }

    calibration_num = 100;

    jnt_state_tracker_subscriber = nh.subscribe("joint_state_tracker", 1000, &KokoHW::CalibrateJointState, this);
  }

  void CalibrateJointState(const sensor_msgs::JointState msg) {
    if (calibrated == calibration_num)
    {
      for (int i = 0; i < num_joints; i++) {
        joint_state_initial[i] = msg.position[i];
        pos[i] = joint_state_initial[i];
        ROS_ERROR("calibrated initial joint state %f", joint_state_initial[i]);
      }
      calibrated++;
      is_calibrated = 1; 
      ROS_ERROR("Finished Calibrating Joint States");
    }
    else if(calibrated != calibration_num + 1)
    {
      for (int i = 0; i < num_joints; i++) {
        joint_state_initial[i] = msg.position[i];
        pos[i] = joint_state_initial[i];
        // ROS_ERROR("calibrated initial joint state %f", joint_state_initial[i]);
      }
      calibrated++;
      // ROS_ERROR("calibrated index %d", calibrated);
    }
  }

  double convertMotorTorqueToCurrent(double motor_torque, int index) {
    return current_slope[index] * motor_torque + current_offset[index];
  }
  
  ros::Time get_time() {
    return ros::Time::now();
  } 

  ros::Duration get_period() {
    ros::Time current_time = ros::Time::now();
    ros::Duration period = current_time - last_time; 
    last_time = current_time;
    return period; 
  }

  void read() {
      motor_driver.read();
      update_joints_from_motors();
  }

  void update_joints_from_motors(){
    if (is_calibrated && (!prev_is_calibrated)) { 
      // ROS_ERROR("d1");
      for(int i = 0; i < num_joints; i++){
        angle_after_calibration[i] = motor_pos[i];
      }
      prev_is_calibrated = 1;
      // ROS_ERROR("d2");

    } 
    if (!is_calibrated) {
      // ROS_ERROR("d3");
      return;
    }
    ROS_ERROR("update_joints_from_motors");
    std::vector<double> pre_joint_pos(num_joints);
    std::vector<double> pre_joint_vel(num_joints);
    // ROS_ERROR("d4");

    for(int i = 0; i < num_joints; i++){
      pre_joint_pos[i] = motor_pos[i] - angle_after_calibration[i];
      pre_joint_vel[i] = motor_vel[i];
    }
    // ROS_ERROR("d5");

    for(int i = 0; i < num_joints; i++){
      if(std::find(paired_constraints.begin(), paired_constraints.end(), i) != paired_constraints.end()) {
        int a = std::find(paired_constraints.begin(), paired_constraints.end(), i) - paired_constraints.begin();
        //trying to set index a
        // ROS_ERROR("in differential, %d", i);
        if (a % 2 == 0) {
          int b = a + 1;
          //Might have to change signs
          pre_joint_pos[i] = -.5 * motor_pos[paired_constraints[a]] + .5 * motor_pos[paired_constraints[b]];
          pre_joint_vel[i] = -.5 * motor_vel[paired_constraints[a]] + .5 * motor_vel[paired_constraints[b]];

        } else {
          int b = a - 1;
          pre_joint_pos[i] = .5 * motor_pos[paired_constraints[b]] + .5 * motor_pos[paired_constraints[a]];
          pre_joint_vel[i] = .5 * motor_vel[paired_constraints[b]] + .5 * motor_vel[paired_constraints[a]];
        }
      }
    } 
    // ROS_ERROR("d6");
    for(int i = 0; i < num_joints; i++){
      pos[i] = directions[i] * (pre_joint_pos[i] / gear_ratios[i]) + joint_state_initial[i];
      vel[i] = directions[i] * pre_joint_vel[i] / gear_ratios[i];  
      eff[i] = 0.0; // TODO: should be populated from driver current readouts
    }
    // ROS_ERROR("d7");
    position_read = 1;
    
  }

  void write() {
    // ROS_ERROR("f1");
    update_motor_currents_from_joint_cmd();
    // ROS_ERROR("f2");
    //motor_driver.write();
    // ROS_ERROR("f3");
  }

  void update_motor_currents_from_joint_cmd() {
    if (!is_calibrated) {
      // ROS_ERROR("g1");
      return;
    }
    // ROS_ERROR("g2");

    std::vector<double> post_motor_torque(num_joints);
    std::vector<double> cmd_oriented(num_joints);
    
    for (int i = 0; i < num_joints; i++) {
      post_motor_torque[i] = cmd[i];
      cmd_oriented[i] = torque_directions[i] * cmd[i];
    }
    // ROS_ERROR("g3");

    for (int j = 0; j < paired_constraints.size(); j = j + 2) {
      int index_a = paired_constraints[j];
      int index_b = paired_constraints[j + 1];
      post_motor_torque[index_a] = -0.5 * cmd_oriented[index_a] +  0.5 * cmd_oriented[index_b];
      post_motor_torque[index_b] =  0.5 * cmd_oriented[index_a] +  0.5 * cmd_oriented[index_b];
    }    
    // ROS_ERROR("g4");

    for (int i = 0; i < num_joints; i++) {
      double motor_torque =  post_motor_torque[i] / gear_ratios[i];
      double motor_current = convertMotorTorqueToCurrent(motor_torque, i);
      motor_cmd[i] = motor_current; 
    }
    // ROS_ERROR("g5");

  }
  
  
  const int getPositionRead() {
    return position_read;
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_effort_interface;
  BLDCDriver motor_driver;
  int num_joints;
  ros::Subscriber jnt_state_tracker_subscriber;
  ros::Time last_time;
  std::vector<std::string> joint_names;
  std::vector<std::string> motor_names;
  std::vector<double> gear_ratios;

  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> eff;
  std::vector<double> cmd;

  std::vector<double> current_slope;
  std::vector<double> current_offset;
  std::vector<int> paired_constraints;

  std::vector<double> motor_pos;
  std::vector<double> motor_vel;
  std::vector<double> motor_eff;
  std::vector<double> motor_cmd;

  int position_read;
  int calibrated;
  std::vector<double> joint_state_initial;
  std::vector<double> directions;
  std::vector<double> torque_directions;
  double i_to_T_slope;
  double i_to_T_intercept;
  int calibration_num;
  std::vector<double> angle_after_calibration;
  int is_calibrated;
  int prev_is_calibrated;
};
