#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
//#include <motor_state_msg/MotorState.h>
#include <urdf/model.h>
#include <math.h>

#include "blue_controller_manager/blue_hardware_interface.h"

namespace ti = transmission_interface;

BlueHW::BlueHW(ros::NodeHandle &nh)
{
  std::string baselink;
  std::string endlink;
  std::string robot_desc_string;

  // load in robot parameters from parameter server
  getRequiredParam(nh, "robot_description", robot_desc_string);
  getRequiredParam(nh, "blue_hardware/joint_names", joint_names_);
  getRequiredParam(nh, "blue_hardware/motor_names", motor_names_);
  getRequiredParam(nh, "blue_hardware/motor_ids", motor_ids_);
  getRequiredParam(nh, "blue_hardware/gear_ratios", gear_ratios_);
  getRequiredParam(nh, "blue_hardware/joint_torque_directions", joint_torque_directions_);
  getRequiredParam(nh, "blue_hardware/current_to_torque_ratios", current_to_torque_ratios_);
  getRequiredParam(nh, "blue_hardware/differential_pairs", differential_pairs_);
  getRequiredParam(nh, "blue_hardware/softstop_torque_limit", softstop_torque_limit_);
  getRequiredParam(nh, "blue_hardware/softstop_min_angles", softstop_min_angles_);
  getRequiredParam(nh, "blue_hardware/softstop_max_angles", softstop_max_angles_);
  getRequiredParam(nh, "blue_hardware/softstop_tolerance", softstop_tolerance_);
  getRequiredParam(nh, "blue_hardware/motor_current_limits", motor_current_limits_);
  getRequiredParam(nh, "blue_hardware/id_torque_gains", id_gains_);
  getRequiredParam(nh, "blue_hardware/baselink", baselink);
  getRequiredParam(nh, "blue_hardware/endlink", endlink);

  // configure robot
  num_joints_ = joint_names_.size();
  ROS_INFO("Robot has %d joints", num_joints_);

  if (differential_pairs_.size() % 2 != 0) {
    ROS_FATAL("Paired_constraints length must be even");
    ros::shutdown();
    exit(1);
  }
  is_base_ = false;
  if (!(std::find(differential_pairs_.begin(), differential_pairs_.end(), 0) != differential_pairs_.end())){
    is_base_ = true;
    ROS_INFO("Robot Configured with Base");
  } else {
    ROS_INFO("Robot Configured with No Base");
  }

  is_gripper_ = false;
  if (!(std::find(differential_pairs_.begin(), differential_pairs_.end(), num_joints_ - 1) != differential_pairs_.end())){
    is_gripper_ = true;
    ROS_INFO("Robot Configured with Gripper");
  } else {
    ROS_INFO("Robot Configured with No Gripper");
  }
  int num_simple_actuators = num_joints_ - differential_pairs_.size();
  num_diff_actuators_ = differential_pairs_.size() / 2;
  int num_actuators = num_simple_actuators + num_diff_actuators_;
  ROS_INFO("%d simple actuators and %d differential actuators", num_simple_actuators, num_diff_actuators_);

  // load in URDF and build kinematic chain
  KDL::Tree my_tree;
  if(!kdl_parser::treeFromString(robot_desc_string, my_tree)){
    ROS_FATAL("Failed to contruct kdl tree");
    ros::shutdown();
    exit(1);
  }
  KDL::Chain chain;
  if (!my_tree.getChain(baselink, endlink, chain)) {
    ROS_FATAL("Failed to construct kdl chain");
    ros::shutdown();
    exit(1);
  }
  buildDynamicChain(chain);

  // initalizing variables

  // misc
  read_from_motors_ = false;
  is_hardstop_calibrate_ = 0.0;
  id_torques_ = KDL::JntArray(chain.getNrOfJoints());
  joint_pos_initial_.resize(num_joints_, 0.0);
  KDL::Vector zero_vect(0.0, 0.0, 0.0);
  actuator_accel_.resize(num_joints_, zero_vect);
  read_gravity_vector_.resize(num_diff_actuators_ + num_simple_actuators);

  // transmission vectors
  actuator_pos_initial_.resize(num_joints_, 0.0);
  actuator_revolution_constant_.resize(num_joints_, 0.0);
  motor_cmd_publishers_.resize(num_joints_);
  actuator_states_.resize(num_actuators);
  actuator_commands_.resize(num_actuators);
  joint_states_.resize(num_actuators);
  joint_commands_.resize(num_actuators);
  simple_transmissions_.resize(num_simple_actuators);
  differential_transmissions_.resize(num_diff_actuators_);
  actuator_pos_.resize(num_joints_);
  actuator_vel_.resize(num_joints_);
  actuator_eff_.resize(num_joints_);
  actuator_cmd_.resize(num_joints_);
  joint_pos_.resize(num_joints_);
  joint_vel_.resize(num_joints_);
  joint_eff_.resize(num_joints_);
  raw_joint_cmd_.resize(num_joints_);
  joint_cmd_.resize(num_joints_);
  calibration_counter_ = 0;
  is_calibrated_ = false;
  gravity_vector_.data[0] = 0;
  gravity_vector_.data[1] = 0;
  gravity_vector_.data[2] = -9.81;
  for (int i = 0; i < num_joints_; i++) {
    joint_pos_initial_[i] = 0.0;
  }

  // register join state handle and effort handle
  for (int i = 0; i < num_joints_; i++) {
    joint_cmd_[i] = 0.0;
    raw_joint_cmd_[i] = 0.0;
    hardware_interface::JointStateHandle state_handle_a(joint_names_[i], &joint_pos_[i], &joint_vel_[i], &joint_eff_[i]);
    joint_state_interface_.registerHandle(state_handle_a);
  }
  registerInterface(&joint_state_interface_);

  for (int i = 0; i < num_joints_; i++) {
    hardware_interface::JointHandle effort_handle_a(joint_state_interface_.getHandle(joint_names_[i]), &raw_joint_cmd_[i]);
    joint_effort_interface_.registerHandle(effort_handle_a);
  }
  registerInterface(&joint_effort_interface_);


  // building the transmissions for the entire robot is general to any configuration of
  // 0 or 1 base, N x Differentials, 0 or 1 gripper
  int simple_idx = 0;
  int differential_idx = 0;
  for(int j_idx = 0; j_idx < num_joints_; j_idx += 0){
    if(std::find(differential_pairs_.begin(), differential_pairs_.end(), j_idx) != differential_pairs_.end()){
      // a differential transmission if in paired constraints
      std::vector<double> gear_ratios_temp(2, 1.0);
      std::vector<double> actuator_ratios(2, 1.0);
      actuator_ratios[0] = -1.0;

      ROS_INFO("Joint Index %d", j_idx);
      gear_ratios_temp[0] = -gear_ratios_[j_idx];
      j_idx++;
      ROS_INFO("Joint Index %d", j_idx);
      gear_ratios_temp[1] = -gear_ratios_[j_idx];
      j_idx++;

      differential_transmissions_[differential_idx] = new ti::DifferentialTransmission(actuator_ratios, gear_ratios_temp);
      differential_idx++;

    } else {
      // a simple transmission because not in paired constraints
      ROS_INFO("Joint Index %d", j_idx);
      simple_transmissions_[simple_idx] = new ti::SimpleTransmission(-gear_ratios_[j_idx], 0.0);
      simple_idx++;
      j_idx++;
    }
  }

  simple_idx = 0;
  differential_idx = 0;
  int actuator_idx = 0;
  for(int j_idx = 0; j_idx < num_joints_; j_idx += 0){
    if(std::find(differential_pairs_.begin(), differential_pairs_.end(), j_idx) != differential_pairs_.end()){
      // a differential transmission if in paired constraints
      // Wrap differential transmission raw data - current state

      ROS_INFO("Joint Index %d", j_idx);
      actuator_states_[actuator_idx].position.push_back(&actuator_pos_[j_idx]);
      actuator_states_[actuator_idx].position.push_back(&actuator_pos_[j_idx + 1]);
      actuator_states_[actuator_idx].velocity.push_back(&actuator_vel_[j_idx]);
      actuator_states_[actuator_idx].velocity.push_back(&actuator_vel_[j_idx + 1]);
      actuator_states_[actuator_idx].effort.push_back(&actuator_eff_[j_idx]);
      actuator_states_[actuator_idx].effort.push_back(&actuator_eff_[j_idx + 1]);

      joint_states_[actuator_idx].position.push_back(&joint_pos_[j_idx]);
      joint_states_[actuator_idx].position.push_back(&joint_pos_[j_idx + 1]);
      joint_states_[actuator_idx].velocity.push_back(&joint_vel_[j_idx]);
      joint_states_[actuator_idx].velocity.push_back(&joint_vel_[j_idx + 1]);
      joint_states_[actuator_idx].effort.push_back(&joint_eff_[j_idx]);
      joint_states_[actuator_idx].effort.push_back(&joint_eff_[j_idx + 1]);

      // Wrap differential transmission raw data - effort command
      actuator_commands_[actuator_idx].effort.push_back(&actuator_cmd_[j_idx]);
      actuator_commands_[actuator_idx].effort.push_back(&actuator_cmd_[j_idx + 1]);
      joint_commands_[actuator_idx].effort.push_back(&joint_cmd_[j_idx]);
      joint_commands_[actuator_idx].effort.push_back(&joint_cmd_[j_idx + 1]);

      j_idx += 2;

      differential_idx++;

    } else {
      // a simple transmission because not in paired constraints
      // Wrap base simple transmission raw data - current state
      ROS_INFO("Joint Index %d", j_idx);
      actuator_states_[actuator_idx].position.push_back(&actuator_pos_[j_idx]);
      actuator_states_[actuator_idx].velocity.push_back(&actuator_vel_[j_idx]);
      actuator_states_[actuator_idx].effort.push_back(&actuator_eff_[j_idx]);
      joint_states_[actuator_idx].position.push_back(&joint_pos_[j_idx]);
      joint_states_[actuator_idx].velocity.push_back(&joint_vel_[j_idx]);
      joint_states_[actuator_idx].effort.push_back(&joint_eff_[j_idx]);

      // Wrap simple transmission raw data - effort command
      actuator_commands_[actuator_idx].effort.push_back(&actuator_cmd_[j_idx]);
      joint_commands_[actuator_idx].effort.push_back(&joint_cmd_[j_idx]);

      simple_idx++;
      j_idx++;
    }
    actuator_idx++;
  }
  // ...once the raw data has been wrapped, the rest is straightforward

  // Register transmissions to each interface
  simple_idx = 0;
  differential_idx = 0;
  int joint_index = 0;
  for(int a_idx = 0; a_idx < num_actuators; a_idx += 1){
    std::ostringstream oss;
    oss << a_idx;
    if(std::find(differential_pairs_.begin(), differential_pairs_.end(), joint_index) != differential_pairs_.end()){
      // a differential transmission if in paired constraints

      actuator_to_joint_interface_.registerHandle(ti::ActuatorToJointStateHandle("differential_trans" + oss.str(),
            differential_transmissions_[differential_idx],
            actuator_states_[a_idx],
            joint_states_[a_idx]));
      joint_to_actuator_interface_.registerHandle(ti::JointToActuatorEffortHandle("differential_trans" + oss.str(),
            differential_transmissions_[differential_idx],
            actuator_commands_[a_idx],
            joint_commands_[a_idx]));
      joint_index += 2;
      differential_idx++;
    } else {
      // a simple transmission because not in paired constraints
      actuator_to_joint_interface_.registerHandle(ti::ActuatorToJointStateHandle("simple_trans" + oss.str(),
            simple_transmissions_[simple_idx],
            actuator_states_[a_idx],
            joint_states_[a_idx]));
      joint_to_actuator_interface_.registerHandle(ti::JointToActuatorEffortHandle("simple_trans" + oss.str(),
            simple_transmissions_[simple_idx],
            actuator_commands_[a_idx],
            joint_commands_[a_idx]));
      joint_index += 1;
      simple_idx++;
    }
  }
  ROS_INFO("Finished setting up transmissions");

  joint_state_tracker_sub_ = nh.subscribe("joint_state_tracker", 1, &BlueHW::calibrationStateCallback, this);
  for (int i = 0; i < motor_names_.size(); i++) {
    motor_cmd_publishers_[i] = nh.advertise<std_msgs::Float64>("blue_hardware/" + motor_names_[i] + "_cmd", 1);
    motor_state_publishers_[i] = nh.advertise<std_msgs::Float64>("blue_hardware/" + motor_names_[i] + "_state", 1);
  }
  ROS_INFO("Finished setting up subscribers and publisher");

  for(auto id : motor_ids_) {
    boards_.push_back(id);
  }
  bldc_.init(boards_, &states_);
  read_from_motors_ = true;
  ROS_INFO("Finished setting up Serial Driver");

  ros::Duration(0.444).sleep();
}

void BlueHW::setReadGravityVector() {
  // function that updates gravity vector of robot (including base)
  int start_ind = 0;
  int error_val = 0;
  if (is_base_ == true) {
    // Apply base gravity transform (rotates base gravtiy vector to be axis aligned)
    KDL::Rotation base_rot_z;
    base_rot_z.DoRotZ(-4.301 - M_PI);
    KDL::Vector corrected_base;
    // scale gravity vector and rotate
    corrected_base = base_rot_z * actuator_accel_[0] * 9.81 / actuator_accel_[0].Norm();
    corrected_base.data[0] = -corrected_base.data[0];
    corrected_base.data[1] = -corrected_base.data[1];
    corrected_base.data[2] = corrected_base.data[2];
    read_gravity_vector_[start_ind] = corrected_base;
    start_ind = 1;
  }
  for (int i = 0; i < num_diff_actuators_; i++) {
    KDL::Vector raw_right;
    KDL::Vector raw_left;
    KDL::Vector raw;

    // reading raw accelerometer values
    raw_right = actuator_accel_.at(start_ind + 2*i);
    raw_left = actuator_accel_.at(start_ind + 2*i + 1);

    // Rotate Left accel vector into Right accel Frame
    KDL::Rotation left_rot_z;
    left_rot_z.DoRotZ(M_PI);
    raw_left = left_rot_z * raw_left;
    KDL::Rotation left_rot_x;
    left_rot_x.DoRotX(M_PI);
    raw_left = left_rot_x * raw_left;

    // Average the KDL Vector accelerations
    raw = (raw_right + raw_left) / 2.0;

    // Apply found link transmission to raw reading
    KDL::Vector x_vect(0.26860026, -0.96283056, -0.02848168);
    KDL::Vector y_vect(0.96299097,  0.2690981,  -0.01531682);
    KDL::Vector z_vect(0.02241186, -0.0233135,   0.99947696);
    KDL::Rotation transform(x_vect, y_vect, z_vect);
    raw = transform * raw;
    KDL::Rotation raw_rot_x;
    raw_rot_x.DoRotX(M_PI / 2);
    raw = raw_rot_x * raw;
    KDL::Rotation raw_rot_z;
    raw_rot_z.DoRotZ(M_PI);
    raw = raw_rot_z * raw;
    raw.data[2] = -raw.data[2];

    read_gravity_vector_[start_ind + i] = raw * 9.81 / raw.Norm();
  }
  // TODO: Find Gripper link transform
  if(is_gripper_) {
    // apply gripper rotation
    KDL::Rotation grip_rot_z;
    grip_rot_z.DoRotZ(2 * M_PI);
    KDL::Vector corrected_grip;
    corrected_grip = grip_rot_z * actuator_accel_[start_ind + 2*num_diff_actuators_] / actuator_accel_[start_ind + 2*num_diff_actuators_].Norm() * 9.81;
    corrected_grip.data[2] = -corrected_grip.data[2];
    read_gravity_vector_[start_ind + num_diff_actuators_] = corrected_grip;
  }

  // accumulate gravity vector and take moving average
  double a = 0.992;
  gravity_vector_[0] = a * gravity_vector_[0] - (1.0 - a) * read_gravity_vector_[0][0];
  gravity_vector_[1] = a * gravity_vector_[1] - (1.0 - a) * read_gravity_vector_[0][1];
  gravity_vector_[2] = a * gravity_vector_[2] - (1.0 - a) * read_gravity_vector_[0][2];
}


template <typename TParam>
void BlueHW::getRequiredParam(ros::NodeHandle &nh, const std::string name, TParam &dest) {
  if(!nh.getParam(name, dest)) {
    ROS_FATAL("Could not find %s parameter in namespace %s", name.c_str(), nh.getNamespace().c_str());
    ros::shutdown();
    exit(1);
  }
}

void BlueHW::read() {
  if (read_from_motors_) 
    motorStateCallback();
}

void BlueHW::write() {
  std::vector<float> commands;
  if (is_calibrated_) {
    // load in inverse dynamics values
    computeInverseDynamics();
    for (int i = 0; i < num_joints_; i++) {
      // add inverse dynamics joint gain for non gripper joints
      if ( !(is_gripper_ && i == num_joints_ - 1) ) {
        joint_cmd_[i] = raw_joint_cmd_[i] + id_torques_(i) * joint_params_[i]->id_gain;
      } else {
        joint_cmd_[i] = raw_joint_cmd_[i];
      }
      // Prevent raw joint commands from persisting, so no torque is
      // applied if a controller is stopped
      raw_joint_cmd_[i] = 0.0;

      // checking joint limits and publish counter torque if near the limit
      if(joint_pos_[i] > softstop_max_angles_[i] - softstop_tolerance_){
        ROS_WARN_THROTTLE(1, "Going over soft stop max, %d", i);
        double del = joint_pos_[i] - softstop_max_angles_[i] + softstop_tolerance_;
        joint_cmd_[i] += -1.0 * softstop_torque_limit_ * del * del;
      } else if (joint_pos_[i] < softstop_min_angles_[i] + softstop_tolerance_){
        ROS_WARN_THROTTLE(1, "Going over soft stop min, %d",i);
        double del = softstop_min_angles_[i] + softstop_tolerance_ - joint_pos_[i];
        joint_cmd_[i] += softstop_torque_limit_ * del * del;
      }
    }

    //propagate requested joint torques through transmission
    joint_to_actuator_interface_.propagate();

    for (int i = 0; i < num_joints_; i++) {
      actuator_cmd_[i] = joint_torque_directions_[i] * actuator_cmd_[i];
      double motor_torque = actuator_cmd_[i];
      // convert torque to current
      double motor_current = current_to_torque_ratios_[i] * motor_torque;

      // threshold current to current limit
      if (std::abs(motor_current) > motor_current_limits_[i]){
        if (motor_current > 0){
          motor_current = motor_current_limits_[i];
        } else {
          motor_current = -motor_current_limits_[i];
        }
      }

      std_msgs::Float64 commandMsg;
      commandMsg.data = motor_current;
      motor_cmd_publishers_[i].publish(commandMsg);
      commands.push_back(motor_current);
    }
  } else {
    for (auto id : boards_) {
      commands.push_back(0.0);
    }
  }

  int index = 0;
  for (auto id : boards_) {
    commands_[id] = commands[index++];
  }
}

void BlueHW::updateComms() {
  read_from_motors_ = true;
  try {
    bldc_.update(commands_);
  } catch (comms_error e) {
    ROS_ERROR(e.what());
    read_from_motors_ = false;
  }
}

void BlueHW::buildDynamicChain(KDL::Chain &chain){
  int ns = chain.getNrOfSegments();
  for(int i = 0; i < ns; i++){
    KDL::Segment seg = chain.segments[i];

    if (seg.getJoint().getType() != 8)
    {
      JointParams* jointParam = new JointParams();
      std::string jointName = seg.getJoint().getName();
      kdl_chain_.addSegment(seg);
      double id_gain = id_gains_[i];
      jointParam->id_gain = id_gains_[i];
      jointParam->joint_name = jointName;
      joint_params_.push_back(jointParam);

      ROS_INFO("Joint %s, has inverse dynamics gain of: %f", jointName.c_str(), id_gain);
    }
  }
  ROS_INFO("Finished constructing chain");
}

void BlueHW::computeInverseDynamics() {
  int id_joints = num_joints_;
  if (is_gripper_) {
    id_joints -= 1;
  }
  KDL::JntArray jointPositions(id_joints);
  KDL::JntArray jointVelocities(id_joints);
  KDL::JntArray jointAccelerations(id_joints);
  KDL::Wrenches f_ext;

  for (int i = 0; i < id_joints; i++) {
    jointPositions(i) = joint_pos_[i];
    jointVelocities(i) = joint_vel_[i];
    jointAccelerations(i) = 0.0;
    f_ext.push_back(KDL::Wrench());
  }

  KDL::ChainIdSolver_RNE chainIdSolver(kdl_chain_, gravity_vector_);
  int statusID = chainIdSolver.CartToJnt(jointPositions, jointVelocities, jointAccelerations, f_ext, id_torques_);
}

void BlueHW::motorStateCallback() {

  int index = 0;
  for (auto id : boards_) {
    if (is_calibrated_ != 1 || !read_from_motors_) {
      actuator_pos_initial_[index] = states_[id].position;
      KDL::Vector accel_vect;
      accel_vect.data[0] = states_[id].acc_x;
      accel_vect.data[1] = states_[id].acc_y;
      accel_vect.data[2] = states_[id].acc_z;
      actuator_accel_.at(index) = accel_vect;
    } else if (is_calibrated_ == 1){
      // add calibration angle to raw actuator reading
      actuator_pos_[index] = states_[id].position - actuator_pos_initial_[index] * is_hardstop_calibrate_ + 2.0 * M_PI * actuator_revolution_constant_[index];
      KDL::Vector accel_vect;
      accel_vect.data[0] = states_[id].acc_x;
      accel_vect.data[1] = states_[id].acc_y;
      accel_vect.data[2] = states_[id].acc_z;
      actuator_accel_.at(index) = accel_vect;
      actuator_vel_[index] = states_[id].velocity;
      actuator_eff_[index] = states_[id].di;
    }
    index++;
  }

  // Propagate actuator information to joint information
  actuator_to_joint_interface_.propagate();

  for(int i = 0; i < num_joints_; i++) {
    joint_pos_[i] = joint_pos_[i] + joint_pos_initial_[i];
  }
  read_from_motors_ = true;
  setReadGravityVector();
}

void BlueHW::calibrationStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  if (is_calibrated_)
    return;
  if (calibration_counter_ >= 10) {
    // waits for 10 calibration messages before finalizing calibration
    for (int i = 0; i < joint_pos_initial_.size(); i++) {
      joint_pos_initial_[i] = msg->position[i];
      joint_pos_[i] = joint_pos_initial_[i];
      ROS_INFO("Calibrated joint %d to state %f", i, joint_pos_initial_[i]);
    }
    calibration_counter_++;
    is_hardstop_calibrate_ = 1.0;
    is_calibrated_ = true;
    ROS_INFO("Finished Calibrating Joint States, counter: %d", calibration_counter_);
  } else {
    for (int i = 0; i < joint_pos_initial_.size(); i++) {
      joint_pos_initial_[i] = msg->position[i];
      joint_pos_[i] = joint_pos_initial_[i];
    }
    calibration_counter_++;
  }
}
