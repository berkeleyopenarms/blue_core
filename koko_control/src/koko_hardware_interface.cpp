#include "koko_control/koko_hardware_interface.h"
#include <std_msgs/Float64.h>
#include <urdf/model.h>
#include <math.h>

namespace ti = transmission_interface;

KokoHW::KokoHW(ros::NodeHandle &nh)
{
  if (!nh.getParam("koko_hardware/joint_names", joint_names_)) {
    ROS_ERROR("No koko_hardware/joint_names given (namespace: %s)", nh.getNamespace().c_str());
  }
  if (!nh.getParam("koko_hardware/motor_names", motor_names_)) {
    ROS_ERROR("No koko_hardware/motor_names given (namespace: %s)", nh.getNamespace().c_str());
  }
  if (!nh.getParam("koko_hardware/gear_ratio", gear_ratios_)) {
    ROS_ERROR("No koko_hardware/gear_ratio given (namespace: %s)", nh.getNamespace().c_str());
  }
  if (!nh.getParam("koko_hardware/joint_torque_directions", joint_torque_directions_)) {
    ROS_ERROR("No koko_hardware/joint_torque_directions given (namespace: %s)", nh.getNamespace().c_str());
  }
  if (!nh.getParam("koko_hardware/current_slope", current_slope_)) {
    ROS_ERROR("No koko_hardware/current_slope given (namespace: %s)", nh.getNamespace().c_str());
  }
  if (!nh.getParam("koko_hardware/paired_constraints", paired_constraints_)) {
    ROS_ERROR("No koko_hardware/paired_constraints given (namespace: %s", nh.getNamespace().c_str());
  }
  if (paired_constraints_.size() % 2 != 0) {
    ROS_ERROR("Paired_constraints length must be even");
  }
  if (!nh.getParam("koko_hardware/softstop_torque_limit", softstop_torque_limit_)) {
    ROS_ERROR("No koko_hardware/softstop_torque_limit given (namespace: %s)", nh.getNamespace().c_str());
  }
  if (!nh.getParam("koko_hardware/min_angles", min_angles_)) {
    ROS_ERROR("No koko_hardware/min_angles given (namespace: %s)", nh.getNamespace().c_str());
  }
  if (!nh.getParam("koko_hardware/max_angles", max_angles_)) {
    ROS_ERROR("No koko_hardware/max_angles given (namespace: %s)", nh.getNamespace().c_str());
  }
  if (!nh.getParam("koko_hardware/softstop_tolerance", softstop_tolerance_)) {
    ROS_ERROR("No koko_hardware/softstop_tolerance given (namespace: %s)", nh.getNamespace().c_str());
  }

  std::string robot_desc_string;
  if (!nh.getParam("robot_dyn_description", robot_desc_string)) {
    ROS_ERROR("No robot_dyn_description given, %s", nh.getNamespace().c_str());
  }

  std::string endlink;
  if (!nh.getParam("koko_hardware/endlink", endlink)) {
    ROS_ERROR("No endlink given node namespace %s", nh.getNamespace().c_str());
  }

  KDL::Tree my_tree;
  if(!kdl_parser::treeFromString(robot_desc_string, my_tree)){
    ROS_ERROR("Failed to contruct kdl tree");
  }

  KDL::Chain chain;
  if (!my_tree.getChain("base_link", endlink, chain)) {
    ROS_ERROR("Failed to construct kdl chain");
  }
  buildDynamicChain(chain);
  id_torques = KDL::JntArray(chain.getNrOfJoints());

  num_joints_ = joint_names_.size();
  read_from_motors_ = false;

  cmd.resize(num_joints_, 0.0);
  pos.resize(num_joints_, 0.0);
  vel.resize(num_joints_, 0.0);
  eff.resize(num_joints_, 0.0);
  joint_pos_initial_.resize(num_joints_, 0.0);
  actuator_pos_initial_.resize(num_joints_, 0.0);

  motor_cmd_publishers_.resize(num_joints_);

  int num_simple_actuators = num_joints_ - paired_constraints_.size();
  int num_differential_actuators = paired_constraints_.size() / 2;
  int num_actuators = num_simple_actuators + num_differential_actuators;

  actuator_states_.resize(num_actuators);
  actuator_commands_.resize(num_actuators);
  joint_states_.resize(num_actuators);
  joint_commands_.resize(num_actuators);
  simple_transmissions_.resize(num_simple_actuators);
  differential_transmissions_.resize(num_differential_actuators);

  actuator_pos_.resize(num_joints_);
  actuator_vel_.resize(num_joints_);
  actuator_eff_.resize(num_joints_);
  actuator_cmd_.resize(num_joints_);

  joint_pos_.resize(num_joints_);
  joint_vel_.resize(num_joints_);
  joint_eff_.resize(num_joints_);
  joint_cmd_.resize(num_joints_);


  for (int i = 0; i < num_joints_; i++) {
    cmd[i] = 0.0;
    hardware_interface::JointStateHandle state_handle_a(joint_names_[i], &pos[i], &vel[i], &eff[i]);
    joint_state_interface_.registerHandle(state_handle_a);
  }

  registerInterface(&joint_state_interface_);

  for (int i = 0; i < num_joints_; i++) {
    hardware_interface::JointHandle effort_handle_a(joint_state_interface_.getHandle(joint_names_[i]), &cmd[i]);
    joint_effort_interface_.registerHandle(effort_handle_a);
  }
  registerInterface(&joint_effort_interface_);

  calibration_counter_ = 0;
  is_calibrated_ = false;


  for (int i = 0; i < num_joints_; i++) {
    joint_pos_initial_[i] = 0.0;
  }

  gravity_vector_.data[0] = 0;
  gravity_vector_.data[1] = 0;
  gravity_vector_.data[2] = -9.81;

  gravity_vector_sub_ = nh.subscribe( "/koko_hardware/gravity_vector_", 1, &KokoHW::gravityVectorCallback, this);
  joint_state_tracker_sub = nh.subscribe("joint_state_tracker", 1, &KokoHW::calibrationStateCallback, this);
  motor_state_sub_ = nh.subscribe("koko_hardware/motor_states", 1, &KokoHW::motorStateCallback, this);

  for (int i = 0; i < motor_names_.size(); i++) {
    motor_cmd_publishers_[i] = nh.advertise<std_msgs::Float64>("koko_hardware/" + motor_names_[i] + "_cmd", 1);
  }

  int simple_idx = 0;
  int differential_idx = 0;
  for(int j_idx = 0; j_idx < num_joints_; j_idx += 0){
    if(std::find(paired_constraints_.begin(), paired_constraints_.end(), j_idx) != paired_constraints_.end()){
      // a differential transmission if in paired constraints
      std::vector<double> gear_ratios_temp(2, 1.0);
      std::vector<double> actuator_ratios(2, 1.0);
      actuator_ratios[0] = -1.0;

      gear_ratios_temp[0] = -gear_ratios_[j_idx];
      j_idx++;
      gear_ratios_temp[1] = -gear_ratios_[j_idx];
      j_idx++;

      differential_transmissions_[differential_idx] = new ti::DifferentialTransmission(actuator_ratios, gear_ratios_temp);
      differential_idx++;

    } else {
      // a simple transmission because not in paired constraints
      simple_transmissions_[simple_idx] = new ti::SimpleTransmission(-gear_ratios_[j_idx], 0.0);
      simple_idx++;
      j_idx++;
    }
  }

  simple_idx = 0;
  differential_idx = 0;
  int actuator_idx = 0;
  for(int j_idx = 0; j_idx < num_joints_; j_idx += 0){
    if(std::find(paired_constraints_.begin(), paired_constraints_.end(), j_idx) != paired_constraints_.end()){
      // a differential transmission if in paired constraints
      // Wrap differential transmission raw data - current state

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
  // ...once the raw data has been wrapped, the rest is straightforward ///

  // Register transmissions to each interface
  simple_idx = 0;
  differential_idx = 0;
  for(int a_idx = 0; a_idx < num_actuators; a_idx += 1){
    std::ostringstream oss;
    oss << a_idx;
    if(std::find(paired_constraints_.begin(), paired_constraints_.end(), a_idx) != paired_constraints_.end()){
      // a differential transmission if in paired constraints

      actuator_to_joint_interface_.registerHandle(ti::ActuatorToJointStateHandle("differential_trans" + oss.str(),
            differential_transmissions_[differential_idx],
            actuator_states_[a_idx],
            joint_states_[a_idx]));
      joint_to_actuator_interface_.registerHandle(ti::JointToActuatorEffortHandle("differential_trans" + oss.str(),
            differential_transmissions_[differential_idx],
            actuator_commands_[a_idx],
            joint_commands_[a_idx]));
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
      simple_idx++;
    }
  }
  ROS_INFO("Finished setting up transmissions");

  //accelerometerCalibrate(num_differential_actuators);
}


void KokoHW::accelerometerCalibrate(int num_diff_actuators) {
  //unsigned int nj = kdl_chain_.getNrOfJoints();
  //KDL::ChainFkSolverPos_recursive fksolver(chain);

  //// calibrate base
  //KDL::Frame base_frame;
  //KDL::JntArray jointPositions = KDL::JntArray(nj);
  //std::vector<double> error_base(8, 0.0);

  //KDL::Vector gravity_base;
  //gravity_base.data[0] = gravity_vector_[0];
  //gravity_base.data[1] = gravity_vector_[1];
  //gravity_base.data[2] = gravity_vector_[2];
  //for(int k = 0; k < 8; k ++){
  //  actuator_pos_[0] = actuator_pos_initial_[0] + 2.0 * M_PI * k;
  //  // add 2kpi to motor position
  //  // propogate to joints
  //  actuator_to_joint_interface_.propagate();
  //  for (int j = 0; j < nj; i++) {
  //    jointPositions(j) = joint_pos_[j];
  //  }

  //  int status = fksolver.JntToCart(jointPositions, base_frame, 1);
  //  KDL::Vector expected_gravity_vect = base_frame * gravity_base;

  //  // gravity vector measured minus gravity vector in this joint position
  //  // TODO read gravity vector from link
  //  //error_base[k] = expect_gravity_vect[0] -

  //  // error will be appended to a list of errors

  //}
  //// choose the k with minimum error
  //int arg_min = std::min_element( error_base.begin(), error_base.end() );
  //actuator_pos_[0] = actuator_pos_initial_[0] + 2.0 * M_PI * arg_min;

  //std::vector<double> error_link(8*8, 0.0);
  //for(int i = 1; i< num_diff_actuators+1; i++ ){

  //  for(int m1 = 0; m1 < 8; m1 ++){
  //    for(int m2 = 0; m2 < 8 ; m2 ++){
  //      actuator_pos_[i*2 - 1] = actuator_pos_initial_[i*2 - 1] + 2.0 * M_PI * (double) m1;
  //      actuator_pos_[i*2] = actuator_pos_initial_[i*2] + 2.0 * M_PI * (double) m2;
  //      // add 2kpi to motor position
  //      // propogate to joints
  //      actuator_to_joint_interface_.propagate();
  //      for (int i = 0; i < nj; i++) {
  //        jointPositions(i) = joint_pos_[i];
  //      }

  //      int status = fksolver.JntToCart(jointPositions, base_frame, 2*i+1);
  //      KDL::Vector expected_gravity_vect = base_frame * gravity_base;

  //      // gravity vector measured minus gravity vector in this joint position
  //      // TODO read gravity vector from link
  //      //error_link[m1 + m2*8] = expect_gravity_vect[0] -
  //      // error will be appended to a list of errors
  //    }
  //  }
  //  // choose the k with minimum error
  //  min_ele = std::min_element( error_link.begin(), error_link.end() );
  //  int argmin_m1 = min_ele%8;
  //  actuator_pos_[i*2 - 1] = actuator_pos_initial_[i*2 - 1] + 2.0 * M_PI * (double) argmin_m1;
  //  int argmin_m2 = min_ele / 8;
  //  actuator_pos_[i*2] = actuator_pos_initial_[i*2] + 2.0 * M_PI * (double) argmin_m2;

  //}
  //is_calibrated_ = true;

}

void KokoHW::read() {
}

void KokoHW::write() {
  if(!read_from_motors_)
    return;

  if (is_calibrated_) {
    computeInverseDynamics();
    // added for using transmission interface
    for (int i = 0; i < num_joints_; i++){
      // TODO
      joint_cmd_[i] = cmd[i] + id_torques(i) * joint_params_[i]->id_gain;
      // checking joint limits and publish counter torque if near
      if(pos[i] > max_angles_[i] - softstop_tolerance_){
        double del = pos[i] - max_angles_[i] + softstop_tolerance_;
        joint_cmd_[i] += -1.0 * softstop_torque_limit_ * del * del;
      } else if (pos[i] < min_angles_[i] + softstop_tolerance_){
        double del = min_angles_[i] + softstop_tolerance_ - pos[i];
        joint_cmd_[i] += softstop_torque_limit_ * del * del;
      }
    }
    //propagate through transmission
    joint_to_actuator_interface_.propagate();
    for (int i = 0; i < num_joints_; i++) {
      actuator_cmd_[i] = joint_torque_directions_[i] * actuator_cmd_[i];
      double motor_torque = actuator_cmd_[i];
      double motor_current = current_slope_[i] * motor_torque;
      std_msgs::Float64 commandMsg;

      commandMsg.data =  motor_current;
      motor_cmd_publishers_[i].publish(commandMsg);
    }
  }
}


void KokoHW::gravityVectorCallback(const geometry_msgs::Vector3ConstPtr& grav) {
  gravity_vector_[0] = grav->x;
  gravity_vector_[1] = grav->y;
  gravity_vector_[2] = grav->z;
}


void KokoHW::buildDynamicChain(KDL::Chain &chain){
  int ns = chain.getNrOfSegments();
  ROS_ERROR
  for(int i = 0; i < ns; i++){
    KDL::Segment seg = chain.segments[i];

    if (seg.getJoint().getType() != 8)
    {
      JointParams* jointParam = new JointParams();
      std::string jointName = seg.getJoint().getName();
      kdl_chain_.addSegment(seg);
      double id_gain;
      jointParam->id_gain = 1.0;
      jointParam->joint_name = jointName;
      joint_params_.push_back(jointParam);

      ROS_INFO("Joint %s, has inverse dynamics gain of: %f", jointName.c_str(), id_gain);
    }
  }
  ROS_INFO("Finished Constructing Chain");
}

void KokoHW::computeInverseDynamics()
{
  unsigned int nj = kdl_chain_.getNrOfJoints();
  KDL::JntArray jointPositions(nj);
  KDL::JntArray jointVelocities(nj);
  KDL::JntArray jointAccelerations(nj);
  KDL::Wrenches f_ext;

  for (int i = 0; i < nj; i++) {
    jointPositions(i) = pos[i];
    jointVelocities(i) = vel[i];
    jointAccelerations(i) = 0.0;
    f_ext.push_back(KDL::Wrench());
  }

  KDL::ChainIdSolver_RNE chainIdSolver(kdl_chain_, gravity_vector_);
  int statusID = chainIdSolver.CartToJnt(jointPositions, jointVelocities, jointAccelerations, f_ext, id_torques);
  // ROS_INFO("status: %d", statusID);
  // ROS_INFO("pos vel =  %f, %f", pos[0], vel[0]);
}

void KokoHW::motorStateCallback(const koko_hardware_drivers::MotorState::ConstPtr& msg) {
  read_from_motors_ = true;
  for (int i = 0; i < msg->name.size(); i++) {
    int index = -1;

    for (int j = 0; j < motor_names_.size(); j++) {
      if (msg->name[i].compare(motor_names_[j]) == 0){
        index = j;
      }
    }

    if (index == -1){
      ROS_ERROR("Some Joint koko_hwi error, msg name %s, with %d joints", msg->name[i].c_str(), num_joints_);
    }

    if (is_calibrated_ != 1) {
      actuator_pos_initial_[index] = msg->position[i];
    } else if (is_calibrated_ == 1){
      actuator_pos_[index] = msg->position[i] - actuator_pos_initial_[index];
      actuator_vel_[index] = msg->velocity[i];
      //actuator_eff[index] = msg->effort[i];
      // update state of all motors
    }
  }
  // propgates actuator information to joint information
  actuator_to_joint_interface_.propagate();
  for(int i = 0; i < num_joints_; i++) {
    pos[i] = joint_pos_[i] + joint_pos_initial_[i];
    vel[i] = joint_vel_[i];
    eff[i] = joint_eff_[i];
  }
}

void KokoHW::calibrationStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  if (is_calibrated_)
    return;
  if (calibration_counter_ >= 10) {
    for (int i = 0; i < joint_pos_initial_.size(); i++) {
      joint_pos_initial_[i] = msg->position[i];
      pos[i] = joint_pos_initial_[i];
      ROS_INFO("Calibrated joint %d to state %f", i, joint_pos_initial_[i]);
    }
    calibration_counter_++;
    is_calibrated_ = true;
    ROS_INFO("Finished Calibrating Joint States, counter: %d", calibration_counter_);
  } else {
    for (int i = 0; i < joint_pos_initial_.size(); i++) {
      joint_pos_initial_[i] = msg->position[i];
      pos[i] = joint_pos_initial_[i];
    }
    calibration_counter_++;
  }
}
