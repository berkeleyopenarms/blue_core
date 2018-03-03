#include "koko_control/koko_hardware_interface.h"
#include <std_msgs/Float64.h>
#include <urdf/model.h>

using namespace transmission_interface;

KokoHW::KokoHW(ros::NodeHandle &nh)
{
   if (!nh.getParam("koko_hardware/joint_names", joint_names)) {
      ROS_INFO("No koko_hardware/joint_names given (namespace: %s)", nh.getNamespace().c_str());
   }
   if (!nh.getParam("koko_hardware/motor_names", motor_names)) {
      ROS_INFO("No koko_hardware/motor_names given (namespace: %s)", nh.getNamespace().c_str());
   }
   if (!nh.getParam("koko_hardware/gear_ratio", gear_ratios)) {
      ROS_INFO("No koko_hardware/gear_ratio given (namespace: %s)", nh.getNamespace().c_str());
   }
   if (!nh.getParam("koko_hardware/joint_torque_directions", joint_torque_directions)) {
      ROS_INFO("No koko_hardware/joint_torque_directions given (namespace: %s)", nh.getNamespace().c_str());
   }
   if (!nh.getParam("koko_hardware/current_slope", current_slope)) {
      ROS_INFO("No koko_hardware/current_slope given (namespace: %s)", nh.getNamespace().c_str());
   }
   if (!nh.getParam("koko_hardware/current_offset", current_offset)) {
      ROS_INFO("No koko_hardware/current_offset given (namespace: %s)", nh.getNamespace().c_str());
   }
   if (!nh.getParam("koko_hardware/paired_constraints", paired_constraints)) {
      ROS_INFO("No koko_hardware/paired_constraints given (namespace: %s", nh.getNamespace().c_str());
   }
   if (paired_constraints.size() % 2 != 0) {
      ROS_INFO("Paired_constraints length must be even");
   }
   if (!nh.getParam("koko_hardware/hardstop_torque_limit", hardstop_torque_limit)) {
      ROS_INFO("No koko_hardware/hardstop_torque_limit given (namespace: %s)", nh.getNamespace().c_str());
   }

   if (!nh.getParam("koko_hardware/min_angles", min_angles)) {
      ROS_INFO("No koko_hardware/min_angles given (namespace: %s)", nh.getNamespace().c_str());
   }

   if (!nh.getParam("koko_hardware/max_angles", max_angles)) {
      ROS_INFO("No koko_hardware/max_angles given (namespace: %s)", nh.getNamespace().c_str());
   }
   if (!nh.getParam("koko_hardware/softstop_tolerance", hardstop_eps)) {
      ROS_INFO("No koko_hardware/softstop_tolerance given (namespace: %s)", nh.getNamespace().c_str());
   }

   std::string robot_desc_string;
   if (!nh.getParam("robot_dyn_description", robot_desc_string)) {
      ROS_ERROR("No robot_dyn_description given, %s", nh.getNamespace().c_str());
   }

   num_joints = joint_names.size();
   motor_pos.resize(num_joints, 0.0);
   motor_vel.resize(num_joints, 0.0);
   cmd.resize(num_joints, 0.0);
   pos.resize(num_joints, 0.0);
   vel.resize(num_joints, 0.0);
   eff.resize(num_joints, 0.0);
   joint_state_initial.resize(num_joints, 0.0);
   angle_after_calibration.resize(num_joints, 0.0);

   motor_cmd_publishers.resize(num_joints);



   int num_simple_actuators = num_joints - paired_constraints.size();
   int num_differential_actuators = paired_constraints.size() / 2;
   int num_actuators = num_simple_actuators + num_differential_actuators;

   a_state_data_vect.resize(num_actuators);
   a_cmd_data_vect.resize(num_actuators);
   j_state_data_vect.resize(num_actuators);
   j_cmd_data_vect.resize(num_actuators);
   simple_transmissions.resize(num_simple_actuators);
   differential_transmissions.resize(num_differential_actuators);

   a_curr_pos_vect.resize(num_joints);
   a_curr_vel_vect.resize(num_joints);
   a_curr_eff_vect.resize(num_joints);
   a_cmd_eff_vect.resize(num_joints);

   j_curr_pos_vect.resize(num_joints);
   j_curr_vel_vect.resize(num_joints);
   j_curr_eff_vect.resize(num_joints);
   j_cmd_eff_vect.resize(num_joints);

   ///////////////////////////////////////////////////////////////////////////////////////////////
   // add for Joint limits reading from urdf
   // loading in joint limits
   //https://github.com/ros-controls/ros_control/wiki/joint_limits_interface
   // max_angles.resize(num_joints);
   // min_angles.resize(num_joints);

   // urdf::Model model;
   // if (!model.initFile(robot_desc_string)){
   //  ROS_ERROR("Failed to parse urdf file");
   // }
   // ROS_INFO("Successfully parsed urdf file");

   // boost::shared_ptr<urdf::ModelInterface> koko_urdf = rdf_loader.getURDF();
   // joint_limits_interface::JointLimits limits;
   // ROS_ERROR("getting joint limits");
   // for (int j; j< num_joints; j ++){
   //  boost::shared_ptr<const urdf::Joint> urdf_joint = koko_urdf->getJoint(joint_names[j]);
   //  const bool urdf_limits_ok = getJointLimits(urdf_joint, limits);
   //  min_angles[j] = limits.min_position;
   //  max_angles[j] = limits.max_position;
   //  ROS_ERROR("min: %f, max: %f", min_angles[j], max_angles[j]);
   // }
   /////////////////////////////////////////////////////////////////////////////////////////////

   for (int i = 0; i < num_joints; i++) {
      cmd[i] = 0.0;
      hardware_interface::JointStateHandle state_handle_a(joint_names[i], &pos[i], &vel[i], &eff[i]);
      jnt_state_interface.registerHandle(state_handle_a);
   }

   registerInterface(&jnt_state_interface);

   for (int i = 0; i < num_joints; i++) {
      hardware_interface::JointHandle effort_handle_a(jnt_state_interface.getHandle(joint_names[i]), &cmd[i]);
      jnt_effort_interface.registerHandle(effort_handle_a);
   }
   registerInterface(&jnt_effort_interface);

   position_read = 0;
   calibrated = 0;
   prev_is_calibrated = 0;
   is_calibrated = 0;

   for (int i = 0; i < num_joints; i++) {
      joint_state_initial[i] = 0.0;
   }

   calibration_num = 10;

   jnt_state_tracker_subscriber = nh.subscribe("joint_state_tracker", 1000, &KokoHW::CalibrateJointState, this);

   motor_state_subscriber = nh.subscribe("koko_hardware/motor_states", 1000, &KokoHW::UpdateMotorState, this);

   for (int i = 0; i < motor_names.size(); i++) {
      motor_cmd_publishers[i] = nh.advertise<std_msgs::Float64>("koko_hardware/" + motor_names[i] + "_cmd", 1000);
      ROS_INFO("Publishers %s", motor_names[i].c_str());
   }
   debug_adv = nh.advertise<std_msgs::Float64>("koko_hardware/joints_pos_update", 1000);

   //*******************************************************************************************************************// added for hardware interaface

   int simple_idx = 0;
   int differential_idx = 0;
   for(int j_idx = 0; j_idx < num_joints; j_idx += 0){
      if(std::find(paired_constraints.begin(), paired_constraints.end(), j_idx) != paired_constraints.end()){
        // a differential transmission if in paired constraints
        std::vector<double> gear_ratios_temp(2, 1.0);
        std::vector<double> actuator_ratios(2, 1.0);
        actuator_ratios[0] = -1.0;

        gear_ratios_temp[0] = -gear_ratios[j_idx];
        j_idx ++;
        gear_ratios_temp[1] = -gear_ratios[j_idx];
        j_idx ++;

        differential_transmissions[differential_idx] = new DifferentialTransmission(actuator_ratios, gear_ratios_temp);
        differential_idx ++;

      } else {
        // a simple transmission because not in paired constraints
        simple_transmissions[simple_idx] = new SimpleTransmission(-gear_ratios[j_idx], 0.0);
        simple_idx ++;
        j_idx ++;
      }
   }

   simple_idx = 0;
   differential_idx = 0;
   int actuator_idx = 0;
   for(int j_idx = 0; j_idx < num_joints; j_idx += 0){
      if(std::find(paired_constraints.begin(), paired_constraints.end(), j_idx) != paired_constraints.end()){
        // a differential transmission if in paired constraints
        // Wrap differential transmission raw data - current state

        a_state_data_vect[actuator_idx].position.push_back(&a_curr_pos_vect[j_idx]);
        a_state_data_vect[actuator_idx].position.push_back(&a_curr_pos_vect[j_idx + 1]);
        a_state_data_vect[actuator_idx].velocity.push_back(&a_curr_vel_vect[j_idx]);
        a_state_data_vect[actuator_idx].velocity.push_back(&a_curr_vel_vect[j_idx + 1]);
        a_state_data_vect[actuator_idx].effort.push_back(&a_curr_eff_vect[j_idx]);
        a_state_data_vect[actuator_idx].effort.push_back(&a_curr_eff_vect[j_idx + 1]);

        j_state_data_vect[actuator_idx].position.push_back(&j_curr_pos_vect[j_idx]);
        j_state_data_vect[actuator_idx].position.push_back(&j_curr_pos_vect[j_idx + 1]);
        j_state_data_vect[actuator_idx].velocity.push_back(&j_curr_vel_vect[j_idx]);
        j_state_data_vect[actuator_idx].velocity.push_back(&j_curr_vel_vect[j_idx + 1]);
        j_state_data_vect[actuator_idx].effort.push_back(&j_curr_eff_vect[j_idx]);
        j_state_data_vect[actuator_idx].effort.push_back(&j_curr_eff_vect[j_idx + 1]);

        // Wrap differential transmission raw data - effort command
        a_cmd_data_vect[actuator_idx].effort.push_back(&a_cmd_eff_vect[j_idx]);
        a_cmd_data_vect[actuator_idx].effort.push_back(&a_cmd_eff_vect[j_idx + 1]);
        j_cmd_data_vect[actuator_idx].effort.push_back(&j_cmd_eff_vect[j_idx]);
        j_cmd_data_vect[actuator_idx].effort.push_back(&j_cmd_eff_vect[j_idx + 1]);

        j_idx ++;
        j_idx ++;

        differential_idx ++;

      } else {
        // a simple transmission because not in paired constraints
        // Wrap base simple transmission raw data - current state
        a_state_data_vect[actuator_idx].position.push_back(&a_curr_pos_vect[j_idx]);
        a_state_data_vect[actuator_idx].velocity.push_back(&a_curr_vel_vect[j_idx]);
        a_state_data_vect[actuator_idx].effort.push_back(&a_curr_eff_vect[j_idx]);
        j_state_data_vect[actuator_idx].position.push_back(&j_curr_pos_vect[j_idx]);
        j_state_data_vect[actuator_idx].velocity.push_back(&j_curr_vel_vect[j_idx]);
        j_state_data_vect[actuator_idx].effort.push_back(&j_curr_eff_vect[j_idx]);

        // Wrap simple transmission raw data - effort command
        a_cmd_data_vect[actuator_idx].effort.push_back(&a_cmd_eff_vect[j_idx]);
        j_cmd_data_vect[actuator_idx].effort.push_back(&j_cmd_eff_vect[j_idx]);

        simple_idx ++;
        j_idx ++;
      }
      actuator_idx ++;
   }
   // ...once the raw data has been wrapped, the rest is straightforward ///

   // Register transmissions to each interface
   simple_idx = 0;
   differential_idx = 0;
   for(int a_idx = 0; a_idx < num_actuators; a_idx += 1){
      std::ostringstream oss;
      oss << a_idx;
      if(std::find(paired_constraints.begin(), paired_constraints.end(), a_idx) != paired_constraints.end()){
        // a differential transmission if in paired constraints

        act_to_jnt_state.registerHandle(ActuatorToJointStateHandle("differential_trans" + oss.str(),
                differential_transmissions[differential_idx],
                a_state_data_vect[a_idx],
                j_state_data_vect[a_idx]));
        jnt_to_act_eff.registerHandle(JointToActuatorEffortHandle("differential_trans" + oss.str(),
                differential_transmissions[differential_idx],
                a_cmd_data_vect[a_idx],
                j_cmd_data_vect[a_idx]));
        differential_idx ++;
      } else {
        // a simple transmission because not in paired constraints
        act_to_jnt_state.registerHandle(ActuatorToJointStateHandle("simple_trans" + oss.str(),
                simple_transmissions[simple_idx],
                a_state_data_vect[a_idx],
                j_state_data_vect[a_idx]));
        jnt_to_act_eff.registerHandle(JointToActuatorEffortHandle("simple_trans" + oss.str(),
                simple_transmissions[simple_idx],
                a_cmd_data_vect[a_idx],
                j_cmd_data_vect[a_idx]));
        simple_idx ++;
      }
   }
}

void KokoHW::UpdateMotorState(const koko_hardware_drivers::MotorState::ConstPtr& msg) {
   std_msgs::Float64 debug_msg;
   debug_msg.data = 1.0;
   debug_adv.publish(debug_msg);

   for (int i = 0; i < msg->name.size(); i++) {
      int index = -1;

      for (int j = 0; j < motor_names.size(); j++) {
        if (msg->name[i].compare(motor_names[j]) == 0){
           index = j;
        }
      }

      if (index == -1){
        ROS_ERROR("Some Joint koko_hwi error, msg name %s, with %d joints", msg->name[i].c_str(), num_joints);
      }

      if (is_calibrated != 1) {
        angle_after_calibration[index] = msg->position[i];
      } else if (is_calibrated == 1){

        position_read = 1;
        /////////////////////////////////////////////////////////////////////////
        // added for using transmission interface
        a_curr_pos_vect[index] = msg->position[i] - angle_after_calibration[index];
        a_curr_vel_vect[index] = msg->velocity[i];
        //a_curr_eff[index] = msg->effort[i];
        // update state of all motors
      }
   }
   ///////////////////////////////////////////
   // added for using transmission interface
   act_to_jnt_state.propagate();
   for(int i = 0; i < num_joints; i ++) {
      //TODO add back in when confiremted that this works
      pos[i] = j_curr_pos_vect[i] + joint_state_initial[i];
      vel[i] = j_curr_vel_vect[i];
      eff[i] = j_curr_eff_vect[i];
   }
   //  update all positions of joints
}

void KokoHW::CalibrateJointState(const sensor_msgs::JointState::ConstPtr& msg) {
   if (calibrated == calibration_num)
   {
      for (int i = 0; i < joint_state_initial.size(); i++) {
        joint_state_initial[i] = msg->position[i];
        pos[i] = joint_state_initial[i];
        ROS_INFO("Calibrated joint %d to state %f", i, joint_state_initial[i]);
      }
      calibrated++;
      is_calibrated = 1;
      ROS_INFO("Finished Calibrating Joint States");
   }
   else if(calibrated != calibration_num + 1)
   {
      for (int i = 0; i < joint_state_initial.size(); i++) {
        joint_state_initial[i] = msg->position[i];
        pos[i] = joint_state_initial[i];
      }
      calibrated++;

      if (calibrated % (calibration_num / 5) == 0) {
        ROS_INFO("Calibrating - received message %d/%d", calibrated, calibration_num);
      }
   }
}

double KokoHW::convertMotorTorqueToCurrent(double motor_torque, int index) {
   return current_slope[index] * motor_torque + current_offset[index];
}

ros::Time KokoHW::get_time() {
   return ros::Time::now();
}

ros::Duration KokoHW::get_period() {
   ros::Time current_time = ros::Time::now();
   ros::Duration period = current_time - last_time;
   last_time = current_time;
   return period;
}

void KokoHW::read() {
}

void KokoHW::write() {
   PublishJointCommand();
}

void KokoHW::PublishJointCommand() {

   if (is_calibrated) {
      std::vector<double> pre(num_joints);
      std::vector<double> cmd_oriented(num_joints);
      // added for using transmission interface
      for (int i = 0; i < num_joints; i++){
        j_cmd_eff_vect[i] = cmd[i];
        // checking joint limits and publish counter torque if near
        if(pos[i] > max_angles[i] - hardstop_eps){
           double del = pos[i] - max_angles[i] + hardstop_eps;
           j_cmd_eff_vect[i] += -1.0 * hardstop_torque_limit * del * del;
        } else if (pos[i] < min_angles[i] + hardstop_eps){
           double del = min_angles[i] + hardstop_eps - pos[i];
           j_cmd_eff_vect[i] += hardstop_torque_limit * del * del;
        }
      }
      //propagate through transmission
      jnt_to_act_eff.propagate();
      for (int i = 0; i < num_joints; i++) {
        a_cmd_eff_vect[i] = joint_torque_directions[i] * a_cmd_eff_vect[i];
        double motor_torque = a_cmd_eff_vect[i];
        double motor_current = convertMotorTorqueToCurrent(motor_torque, i);
        std_msgs::Float64 commandMsg;

        commandMsg.data =  motor_current;
        motor_cmd_publishers[i].publish(commandMsg);
      }
   }
}

const int KokoHW::getPositionRead() {
   return position_read;
}