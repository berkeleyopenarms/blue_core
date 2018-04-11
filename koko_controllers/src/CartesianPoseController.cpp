#include <koko_controllers/CartesianPoseController.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <kdl/frames.hpp>
#include <vector>
#include <iterator>
#include <unistd.h>
#include <algorithm>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <numeric>

#include <koko_controllers/pseudoinverse.h>


namespace koko_controllers{
  bool CartesianPoseController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {

    ros::NodeHandle node;
    std::string robot_desc_string;
    if (!node.getParam("robot_dyn_description", robot_desc_string)) {
      ROS_ERROR("No robot_dyn_description given, %s", node.getNamespace().c_str());
    }

    KDL::Tree my_tree;

    if(!kdl_parser::treeFromString(robot_desc_string, my_tree)) {
      ROS_ERROR("Failed to contruct kdl tree");
      return false;
    }

    std::string endlink;
    if (!n.getParam("endlink", endlink)) {
      ROS_ERROR("No endlink given node namespace %s", n.getNamespace().c_str());
    }

    posture_control_ = false;
    if (!n.getParam("posture_control", posture_control_)) {
      ROS_ERROR("No posture_control given node namespace %s", n.getNamespace().c_str());
    }

    if (!n.getParam("posture_target", posture_target_)) {
      ROS_ERROR("No posture_target given node namespace %s", n.getNamespace().c_str());
    }

    if (!n.getParam("posture_gain", posture_gain_)) {
      ROS_ERROR("No posture_gain given node namespace %s", n.getNamespace().c_str());
    }

    if (!n.getParam("paired_constraints_", paired_constraints_)) {
      ROS_ERROR("No paired_constraint given node namespace %s", n.getNamespace().c_str());
    }

    if (!n.getParam("filter_length", filter_length_)) {
      ROS_ERROR("No filter_length given node namespace %s", n.getNamespace().c_str());
    }

    if (paired_constraints_.size() % 2 != 0) {
      ROS_ERROR("Paired_constraints length must be even");
    }

    // Load PID parameters
    pid_controllers_.resize(6);

    std::string pid_prefix = "pid";
    char dimension_names_pos[] = "xyz";
    char dimension_names_rot[] = "rpy";

    for (int i = 0; i < 3; i++) {
      pid_controllers_[i].init(ros::NodeHandle(n, pid_prefix + "_pos_" + dimension_names_pos[i]));
    }
    for (int i = 0; i < 3; i++) {
      pid_controllers_[i+3].init(ros::NodeHandle(n, pid_prefix + "_rot_" + dimension_names_rot[i]));
    }

    // Get URDF

    KDL::Chain dummyChain;
    if (!my_tree.getChain("base_link", endlink, dummyChain)) {
      ROS_ERROR("Failed to construct kdl kdl_chain_");
      return false;
    }

    int ns = dummyChain.getNrOfSegments();
    ROS_INFO("ns = %d", ns);
    for(int i = 0; i < ns; i++) {
      KDL::Segment seg = dummyChain.segments[i];

      if (seg.getJoint().getType() != 8)
      {
        JointSettings jointPD;
        std::string jointName = seg.getJoint().getName();
        joint_names_.push_back(jointName);
        kdl_chain_.addSegment(seg);
        jointPD.joint = robot->getHandle(jointName);

        double max_torque;
        double min_torque;
        double min_angle;
        double max_angle;
        double pos_mult;
        double rot_mult;
        double d_gain;

        if (!n.getParam(jointName + "/max_torque", max_torque)) {
          ROS_ERROR("No %s/max_torque given (namespace: %s)", jointName.c_str(), n.getNamespace().c_str());
          return false;
        }
        if (!n.getParam(jointName + "/min_torque", min_torque)) {
          ROS_ERROR("No %s/min_torque given (namespace: %s)", jointName.c_str(), n.getNamespace().c_str());
          return false;
        }
        if (!n.getParam(jointName + "/min_angle", min_angle)) {
          ROS_ERROR("No %s/min_angle given (namespace: %s)", jointName.c_str(), n.getNamespace().c_str());
          return false;
        }
        if (!n.getParam(jointName + "/max_angle", max_angle)) {
          ROS_ERROR("No %s/max_angle given (namespace: %s)", jointName.c_str(), n.getNamespace().c_str());
          return false;
        }
        if (!n.getParam(jointName + "/d_gain", d_gain)) {
          ROS_ERROR("No %s/d_gain given (namespace: %s)", jointName.c_str(), n.getNamespace().c_str());
          return false;
        }

        if (!n.getParam(jointName + "/pos_mult", pos_mult)) {
          ROS_ERROR("No %s/pos_mult given (namespace: %s)", jointName.c_str(), n.getNamespace().c_str());
          return false;
        }
        if (!n.getParam(jointName + "/rot_mult", rot_mult)) {
          ROS_ERROR("No %s/rot_mult given (namespace: %s)", jointName.c_str(), n.getNamespace().c_str());
          return false;
        }

        jointPD.pos_mult = pos_mult;
        jointPD.rot_mult = rot_mult;

        jointPD.max_torque = max_torque;
        jointPD.min_torque = min_torque;
        jointPD.max_angle = max_angle;
        jointPD.min_angle = min_angle;
        jointPD.d_gain = d_gain;
        jointPD.joint_name = jointName;
        joint_vector_.push_back(jointPD);

      }
    }

    std::string root_name;
    if (!n.getParam("root_name", root_name)) {
      ROS_ERROR("No root_name given (namespace: %s)", n.getNamespace().c_str());
      return false;
    }

    sub_command_ = n.subscribe("command", 1, &CartesianPoseController::controllerPoseCallback, this);

    ROS_INFO("nj %d", kdl_chain_.getNrOfJoints());

    pub_command_ = node.advertise<std_msgs::Float64MultiArray>("pub_command_msg", 1);
    pub_delta_ = node.advertise<std_msgs::Float64MultiArray>("delta_msg", 1);

    for (int i = 0; i < 6; i++) {
      std::vector<double> err_dot_history;
      d_error_history_.push_back(err_dot_history);
    }

    return true;
  }

  void CartesianPoseController::starting(const ros::Time& time)
  {
    unsigned int nj = kdl_chain_.getNrOfJoints();

    KDL::ChainFkSolverPos_recursive fksolver(kdl_chain_);
    KDL::Frame cartpos;

    KDL::JntArray jnt_pos_ = KDL::JntArray(nj);
    for (int i = 0; i <nj; i++) {
      jnt_pos_(i) = joint_vector_[i].joint.getPosition();
    }
    int status = fksolver.JntToCart(jnt_pos_, cartpos);
    command_pose_.position.x = cartpos.p.data[0];
    command_pose_.position.y = cartpos.p.data[1];
    command_pose_.position.z = cartpos.p.data[2];
    cartpos.M.GetQuaternion(command_pose_.orientation.x, command_pose_.orientation.y,
      command_pose_.orientation.z, command_pose_.orientation.w);

  }

  void CartesianPoseController::controllerPoseCallback(const geometry_msgs::PoseStamped msg)
  {
    command_pose_ = msg.pose;
  }

  void CartesianPoseController::publishCommandMsg(KDL::Vector desired_position, KDL::Rotation desired_rotation) {
    double r; double p; double y;
    desired_rotation.GetRPY(r, p, y);
    std_msgs::Float64MultiArray command_msg;
    command_msg.data.push_back(desired_position.data[0]);
    command_msg.data.push_back(desired_position.data[1]);
    command_msg.data.push_back(desired_position.data[2]);
    command_msg.data.push_back(r);
    command_msg.data.push_back(p);
    command_msg.data.push_back(y);
    pub_command_.publish(command_msg);
  }

  void CartesianPoseController::publishDeltaMsg(KDL::Twist twist_error) {
    std_msgs::Float64MultiArray delta_msg;
    delta_msg.data.push_back(twist_error.vel.data[0]);
    delta_msg.data.push_back(twist_error.vel.data[1]);
    delta_msg.data.push_back(twist_error.vel.data[2]);
    delta_msg.data.push_back(twist_error.rot.data[0]);
    delta_msg.data.push_back(twist_error.rot.data[1]);
    delta_msg.data.push_back(twist_error.rot.data[2]);
    pub_delta_.publish(delta_msg);
  }

  void CartesianPoseController::update(const ros::Time& time, const ros::Duration& period)
  {

    unsigned int nj = kdl_chain_.getNrOfJoints();
    std::vector<double> commands(nj);

    KDL::Vector desired_position(command_pose_.position.x, command_pose_.position.y, command_pose_.position.z);
    KDL::Rotation desired_rotation = KDL::Rotation::Quaternion(command_pose_.orientation.x, command_pose_.orientation.y,
                                                             command_pose_.orientation.z, command_pose_.orientation.w);
    publishCommandMsg(desired_position, desired_rotation);

    KDL::Frame pose_desired = KDL::Frame(desired_rotation, desired_position);

    KDL::JntArray jnt_pos_ = KDL::JntArray(nj);
    Eigen::Matrix<double, Eigen::Dynamic, 1> jnt_vel_(nj, 1);
    // KDL::JntArray jnt_vel_ = KDL::JntArray(nj);
    for (int i = 0; i <nj; i++) {
      jnt_pos_(i) = joint_vector_[i].joint.getPosition();
      jnt_vel_(i) = joint_vector_[i].joint.getVelocity();
    }
    KDL::ChainFkSolverPos_recursive jnt_to_pose_solver(kdl_chain_);
    KDL::Frame pose_current;
    jnt_to_pose_solver.JntToCart(jnt_pos_, pose_current);

    KDL::Twist twist_error = KDL::diff(pose_current, pose_desired);

    /*if (twist_error.vel.Norm() > 1.0) {
      ROS_INFO("twist err mag %f)", twist_error.vel.Norm());
      twist_error.vel.Normalize() * 1.0;
    } */

    KDL::Jacobian jacobian(nj);
    KDL::ChainJntToJacSolver jac_solver(kdl_chain_);
    jac_solver.JntToJac(jnt_pos_, jacobian, -1);

    // Eigen::Matrix<double, Eigen::Dynamic, 1> joint_vel(nj, 1);
    // for(int i = 0; o < nj; i++){
      // joint_vel(i, 1) = joint_vector_[i].joint.getVelocity()
    // }

    Eigen::Matrix<double, Eigen::Dynamic, 1> twist_error_dot(nj, 1);
    twist_error_dot = jacobian.data * jnt_vel_ ;
    KDL::Twist twist_error_dot_;
    for(int i = 0; i < 6; i++){
      twist_error_dot_(i) = -twist_error_dot[i];
    }

    KDL::Wrench wrench_desi;
    for (unsigned int i=0; i<6; i++)
      wrench_desi(i) = computeCommand(twist_error(i), twist_error_dot_(i), period, i);



    for (unsigned int i = 0; i < nj; i++) {
      commands[i] = 0;
      for (unsigned int j=0; j<6; j++) {


        if(j == 0 || j == 1 || j == 2) {
          // position coordinates of jacobian
          commands[i] += (jacobian(j,i) * wrench_desi(j)) * joint_vector_[i].pos_mult;
          if (i == 3 && j == 2){
            // ROS_ERROR("command pos %d, %f", i, commands[i]);
          }
        } else {
          // rotation coordinates of jacobian
          commands[i] += (jacobian(j,i) * wrench_desi(j)) * joint_vector_[i].rot_mult;
        }
      }

    }
    // ROS_INFO_THROTTLE(3, "Jac col 5 %f, %f, %f, %f, %f, %f", jacobian(0,5), jacobian(1,5), jacobian(2,5), jacobian(3,5), jacobian(4,5), jacobian(5,5));
    // ROS_INFO_THROTTLE(3, "Jac col 6 %f, %f, %f, %f, %f, %f", jacobian(0,6), jacobian(1,6), jacobian(2,6), jacobian(3,6), jacobian(4,6), jacobian(5,6));
    // d gain joint portion
    for(int i = 0; i < nj; i++) {
      commands[i] += -joint_vector_[i].d_gain * jnt_vel_(i);
    }


    // null space posture control
    if (posture_control_) {
      Eigen::Matrix<double, Eigen::Dynamic, 1>  posture_error(nj,1);
      for (int i = 0; i < nj; i++) {
        posture_error(i, 0) = posture_target_[i] - jnt_pos_(i);
      }
      Eigen::Matrix<double, 6, Eigen::Dynamic>  jacobian_eig(6, nj);

      for (int i = 0; i < 6; i++) {
        for (int j = 0; j < nj; j++) {
          jacobian_eig(i, j) = jacobian(i, j);
        }
      }

      Eigen::Matrix<double, Eigen::Dynamic, 6>  jacobian_pinv = pseudoinverse(jacobian_eig, 0.00000001);

      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>  I_nj(nj, nj);
      I_nj.setIdentity();
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>  nullspace_proj = I_nj - jacobian_pinv * jacobian_eig;

      Eigen::Matrix<double, Eigen::Dynamic, 1> posture_error_proj = nullspace_proj * (posture_gain_ * posture_error);


      for(int i = 0; i < nj; i++) {
        commands[i] += posture_error_proj(i, 0);
      }


    }


    for(int i = 0; i < nj; i++) {
      if(std::find(paired_constraints_.begin(), paired_constraints_.end(), i) == paired_constraints_.end()) {
        commands[i] = std::min(std::max(commands[i], joint_vector_[i].min_torque), joint_vector_[i].max_torque);
      }
    }

    for (int i = 0; i < paired_constraints_.size(); i = i + 2) {
      int lift_index = paired_constraints_[i];
      int roll_index = paired_constraints_[i+1];
      double lift = commands[lift_index];
      double roll = commands[roll_index];
      //ROS_INFO("lift: %f, roll: %f", commands[lift_index],  commands[roll_index]);

      double max_effort = joint_vector_[lift_index].max_torque + joint_vector_[roll_index].max_torque;
      double scalar = 1;
      if (lift >= 0 && roll >= 0 && (lift + roll) > max_effort) {
        scalar = max_effort / (lift + roll);
      } else if (lift < 0 && roll >= 0 && (roll - lift) > max_effort) {
        scalar = max_effort / (roll - lift);
      } else if (lift < 0 && roll < 0 && (-roll - lift) > max_effort) {
        scalar = max_effort / (-roll - lift);
      } else if (lift >= 0 && roll < 0 && (lift - roll) > max_effort) {
        scalar = max_effort / (lift - roll);
      }
      commands[lift_index] *= scalar;
      commands[roll_index] *= scalar;
    }
    for (int i = 0; i < joint_vector_.size(); i++) {
      //ROS_INFO("command %d: %f", i, commands[i]);
      joint_vector_[i].joint.setCommand(commands[i]);
    }
  }

  double CartesianPoseController::computeCommand(double error, double error_dot, const ros::Duration& dt, int index)
  {
    if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error))
      return 0.0;

    if (std::isnan(error_dot) || std::isinf(error_dot))
      return 0.0;

    d_error_history_[index].push_back(error_dot);
    int size = d_error_history_[index].size();

    if (d_error_history_[index].size() > filter_length_) {
      d_error_history_[index].erase(d_error_history_[index].begin());
    }

    double error_dot_average = std::accumulate(d_error_history_[index].begin(), d_error_history_[index].end(),
                              0.0) / d_error_history_[index].size();

    if (index == 3){
      // ROS_ERROR("command error%d, %f", index, error);
    }
    return pid_controllers_[index].computeCommand(error, error_dot_average, dt);
  }

  geometry_msgs::Pose CartesianPoseController::enforceJointLimits(geometry_msgs::Pose command_pose_)
  {

    ROS_ERROR("command_pose_ before %f %f %f", command_pose_.position.x, command_pose_.position.y, command_pose_.position.z);
    unsigned int nj = kdl_chain_.getNrOfJoints();
    KDL::JntArray joint_positions = KDL::JntArray(nj);

    for (int i =0; i < nj; i++) {
      joint_positions(i) =  joint_vector_[i].joint.getPosition();
    }

    KDL::Frame cartpos;
    KDL::ChainFkSolverPos_recursive fksolver(kdl_chain_);
    int status = fksolver.JntToCart(joint_positions, cartpos);

    KDL::Jacobian jacobian(nj);
    KDL::ChainJntToJacSolver jac_solver(kdl_chain_);
    jac_solver.JntToJac(joint_positions, jacobian, -1);

    KDL::JntArray joint_inverse_kin = KDL::JntArray(nj);
    for (int i = 0; i < nj; i++) {
      joint_inverse_kin(i) = joint_positions(i);
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic> ee_pose_desired(3,1);
    ee_pose_desired(0, 0) = command_pose_.position.x;
    ee_pose_desired(1, 0) = command_pose_.position.y;
    ee_pose_desired(2, 0) = command_pose_.position.z;

    KDL::Rotation desired_rotation = KDL::Rotation::Quaternion(command_pose_.orientation.x,
                                                               command_pose_.orientation.y,
                                                               command_pose_.orientation.z,
                                                               command_pose_.orientation.w
                                                               );

    //ROS_ERROR("desired position: %f, %f, %f", ee_pose_desired(0), ee_pose_desired(1), ee_pose_desired(2));

    for (int j = 0; j < 1000; j++)
    {
      int status = fksolver.JntToCart(joint_inverse_kin, cartpos);
      //ROS_INFO("cartpos: %f, %f, %f", cartpos.p.data[0], cartpos.p.data[1], cartpos.p.data[2]);
      jac_solver.JntToJac(joint_inverse_kin, jacobian, -1);

      Eigen::Matrix<double,6,Eigen::Dynamic> jacPos(6,nj);

      for (unsigned int joint = 0; joint < nj; joint++) {
        for (unsigned int index = 0; index < 6; index ++) {
          jacPos(index,joint) = jacobian(index,joint);
          //ROS_INFO("jacobian %d, %d = %f", index, joint, jacPos(index,joint));
        }
      }

      Eigen::Matrix<double,3, Eigen::Dynamic> ee_pose_cur(3,1);
      for (int i = 0; i < 3; i ++) {
        ee_pose_cur(i, 0) = cartpos.p.data[i];
      }

      KDL::Rotation rotation_difference = desired_rotation * cartpos.M.Inverse();
      KDL::Vector rotation_difference_vec = rotation_difference.GetRot();

      Eigen::Matrix<double, 3, Eigen::Dynamic> pose_difference = ee_pose_desired - ee_pose_cur;

      Eigen::Matrix<double, 6, Eigen::Dynamic> deltaX(6,1);

      deltaX(0, 0) = pose_difference(0, 0);
      deltaX(1, 0) = pose_difference(1, 0);
      deltaX(2, 0) = pose_difference(2, 0);
      deltaX(3, 0) = rotation_difference_vec(0);
      deltaX(4, 0) = rotation_difference_vec(1);
      deltaX(5, 0) = rotation_difference_vec(2);
      ROS_INFO("delta X %f, %f, %f, %f, %f, %f", deltaX(0, 0), deltaX(1, 0),  deltaX(2, 0),
        deltaX(3, 0),  deltaX(4, 0),  deltaX(5, 0));


      Eigen::MatrixXd deltaJoint = jacPos.transpose() * deltaX;
      ROS_INFO("delta joint %f, %f, %f, %f, %f, %f",deltaJoint(0,0), deltaJoint(1,0), deltaJoint(2,0),
        deltaJoint(3,0), deltaJoint(4,0), deltaJoint(5,0));

      double alpha = 1;
      for (int k = 0; k < nj; k++) {
        joint_inverse_kin(k) = alpha * deltaJoint(k,0) + joint_inverse_kin(k);
      }
    }

    for (int i = 0; i < nj; i++)  {

      joint_inverse_kin(i) = fmod(joint_inverse_kin(i) + M_PI, 2 * M_PI);
      if (joint_inverse_kin(i) < 0) joint_inverse_kin(i) += 2 * M_PI;
      joint_inverse_kin(i) = joint_inverse_kin(i) - M_PI;
      joint_inverse_kin(i) = std::min(std::max(joint_inverse_kin(i), joint_vector_[i].min_angle), joint_vector_[i].max_angle);
    }

    fksolver.JntToCart(joint_inverse_kin, cartpos);
    geometry_msgs::Point point;
    point.x = cartpos.p.data[0];
    point.y = cartpos.p.data[1];
    point.z = cartpos.p.data[2];
    geometry_msgs::Quaternion rot;
    cartpos.M.GetQuaternion(rot.x, rot.y, rot.z, rot.w);
    geometry_msgs::Pose confined_pose;
    confined_pose.position = point;
    confined_pose.orientation = rot;
    // ROS_ERROR("command_pose_ after %f %f %f", confined_pose.position.x, confined_pose.position.y, confined_pose.position.z);

    return confined_pose;
  }


}


PLUGINLIB_EXPORT_CLASS(koko_controllers::CartesianPoseController, controller_interface::ControllerBase)
