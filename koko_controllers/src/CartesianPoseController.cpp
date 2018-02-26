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

#include <koko_misc/pseudoinverse.h>


namespace koko_controllers{
  bool CartesianPoseController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {

    ros::NodeHandle node;
    std::string robot_desc_string;
    if (!node.getParam("robot_dyn_description", robot_desc_string)) {
      ROS_ERROR("No robot_dyn_description given, %s", node.getNamespace().c_str());
    }

    KDL::Tree my_tree;

    if(!kdl_parser::treeFromString(robot_desc_string, my_tree)){
      ROS_ERROR("Failed to contruct kdl tree");
      return false; 
    }

    std::string endlink;
    if (!n.getParam("endlink", endlink)) {
      ROS_ERROR("No endlink given node namespace %s", n.getNamespace().c_str());
    }

    posture_control = false;
    if (!n.getParam("posture_control", posture_control)) {
      ROS_ERROR("No posture_control given node namespace %s", n.getNamespace().c_str());
    }

    if (!n.getParam("posture_target", posture_target)) {
      ROS_ERROR("No posture_target given node namespace %s", n.getNamespace().c_str());
    }

    if (!n.getParam("posture_gain", posture_gain)) {
      ROS_ERROR("No posture_gain given node namespace %s", n.getNamespace().c_str());
    }

    if (!n.getParam("paired_constraints", paired_constraints)) {
      ROS_ERROR("No paired_constraint given node namespace %s", n.getNamespace().c_str());
    }

    if (!n.getParam("p_gains", p_gains)) {
      ROS_ERROR("No p_gains given node namespace %s", n.getNamespace().c_str());
    }

    if (!n.getParam("d_gains", d_gains)) {
      ROS_ERROR("No d_gains given node namespace %s", n.getNamespace().c_str());
    }

    if (!n.getParam("z_offset_controller", z_offset_controller)) {
      ROS_ERROR("No z_offset_controller given node namespace %s", n.getNamespace().c_str());
    }

    if (!n.getParam("filter_length", filter_length)) {
      ROS_ERROR("No filter_length given node namespace %s", n.getNamespace().c_str());
    }

    if (paired_constraints.size() % 2 != 0) {
      ROS_ERROR("Paired_constraints length must be even");
    }

    KDL::Chain dummyChain;

    if (!my_tree.getChain("base_link", endlink, dummyChain)) {
      ROS_ERROR("Failed to construct kdl chain");
      return false;
    }

    int ns = dummyChain.getNrOfSegments();
    ROS_INFO("ns = %d", ns);
    for(int i = 0; i < ns; i++){
      KDL::Segment seg = dummyChain.segments[i];

      if (seg.getJoint().getType() != 8)
      {
        JointPD jointPD;
        std::string jointName = seg.getJoint().getName();
        joint_names.push_back(jointName);
        chain.addSegment(seg);
        jointPD.joint = robot->getHandle(jointName);

        double max_torque;
        double min_torque;
        double min_angle;
        double max_angle;
        double id_gain;
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
        if (!n.getParam(jointName + "/id", id_gain)) {
          ROS_ERROR("No %s/id gain given (namespace: %s)", jointName.c_str(), n.getNamespace().c_str());
          return false;
        }

        jointPD.max_torque = max_torque;
        jointPD.min_torque = min_torque;
        jointPD.max_angle = max_angle;
        jointPD.min_angle = min_angle;
        jointPD.id_gain = id_gain;
        jointPD.d_gain = d_gain;
        jointPD.joint_name = jointName;
        joint_vector.push_back(jointPD);

      }
    }

    std::string root_name;
    if (!n.getParam("root_name", root_name)) {
      ROS_ERROR("No root_name given (namespace: %s)", n.getNamespace().c_str());
      return false;
    }

    subController = n.subscribe("command", 1, &CartesianPoseController::controllerPoseCallback, this);
    subCommand = n.subscribe("command_label", 1, &CartesianPoseController::commandCallback, this);

    sub_grav = n.subscribe( "/koko_hardware/gravity", 1, &CartesianPoseController::gravCallback, this);
    sub_joint = n.subscribe("/" + root_name  + "/joint_states", 1, &CartesianPoseController::jointCallback, this);

    ROS_INFO("nj %d", chain.getNrOfJoints());

    id_torques = KDL::JntArray(chain.getNrOfJoints());
    p_error_last.resize(6);
    d_error.resize(6);
    visualizer = "simple_6dof_MOVE_ROTATE_3D";
    command_label = 0;

    commandPub = node.advertise<std_msgs::Float64MultiArray>("/commandPub", 1000);
    deltaPub = node.advertise<std_msgs::Float64MultiArray>("/deltaPub", 1000);
    inverseDynamicsPub = node.advertise<std_msgs::Float64MultiArray>("/inverseDynamicsPub", 1000);

    for (int i = 0; i < 6; i++) {
      std::vector<double> err_dot_history;
      err_dot_histories.push_back(err_dot_history);
    }

    return true;
  }

  void CartesianPoseController::starting(const ros::Time& time)
  {
    unsigned int nj = chain.getNrOfJoints();

    KDL::ChainFkSolverPos_recursive fksolver1(chain);
    KDL::Frame cartpos;

    KDL::JntArray jnt_pos_ = KDL::JntArray(nj);
    for (int i = 0; i <nj; i++) {
      jnt_pos_(i) = joint_vector[i].joint.getPosition();
    }
    int status = fksolver1.JntToCart(jnt_pos_, cartpos);
    commandPose.position.x = cartpos.p.data[0];
    commandPose.position.y = cartpos.p.data[1];
    commandPose.position.z = cartpos.p.data[2];
    cartpos.M.GetQuaternion(commandPose.orientation.x, commandPose.orientation.y, 
      commandPose.orientation.z, commandPose.orientation.w);

  }

  void CartesianPoseController::commandCallback(const std_msgs::Int32 msg) 
  {
    command_label = msg.data;
  }

  void CartesianPoseController::controllerPoseCallback(const geometry_msgs::PoseStamped msg) 
  {
    if (command_label == 25) {
      commandPose = msg.pose;
    }
  }

  void CartesianPoseController::publishCommandMsg(KDL::Vector desired_position, KDL::Rotation desired_rotation) {
    double r; double p; double y;
    desired_rotation.GetRPY(r, p, y);
    std_msgs::Float64MultiArray commandMsg;
    commandMsg.data.push_back(desired_position.data[0]);
    commandMsg.data.push_back(desired_position.data[1]);
    commandMsg.data.push_back(desired_position.data[2]);
    commandMsg.data.push_back(r);
    commandMsg.data.push_back(p);
    commandMsg.data.push_back(y);
    //commandPub.publish(commandMsg);
  }

  void CartesianPoseController::publishDeltaMsg(KDL::Twist twist_error) {
    std_msgs::Float64MultiArray deltaMsg;
    deltaMsg.data.push_back(twist_error.vel.data[0]);
    deltaMsg.data.push_back(twist_error.vel.data[1]);
    deltaMsg.data.push_back(twist_error.vel.data[2]);
    deltaMsg.data.push_back(twist_error.rot.data[0]);
    deltaMsg.data.push_back(twist_error.rot.data[1]);
    deltaMsg.data.push_back(twist_error.rot.data[2]);
    deltaPub.publish(deltaMsg);
  }

  void CartesianPoseController::publishInverseDynamicsMsg() {
    std_msgs::Float64MultiArray inverseDynamicsMsg;
    for (int i = 0; i < joint_vector.size(); i++) {
      inverseDynamicsMsg.data.push_back(id_torques(i));
    }
    //inverseDynamicsPub.publish(inverseDynamicsMsg);
  }

  void CartesianPoseController::update(const ros::Time& time, const ros::Duration& period)
  { 

    unsigned int nj = chain.getNrOfJoints();
    std::vector<double> commands(nj);

    KDL::Vector desired_position(commandPose.position.x, commandPose.position.y, commandPose.position.z);
    KDL::Rotation desired_rotation = KDL::Rotation::Quaternion(commandPose.orientation.x, commandPose.orientation.y,
                                                             commandPose.orientation.z, commandPose.orientation.w);
    publishCommandMsg(desired_position, desired_rotation);

    KDL::Frame pose_desired = KDL::Frame(desired_rotation, desired_position);

    KDL::JntArray jnt_pos_ = KDL::JntArray(nj);
    KDL::JntArray jnt_vel_ = KDL::JntArray(nj);
    for (int i = 0; i <nj; i++) {
      jnt_pos_(i) = joint_vector[i].joint.getPosition();
      jnt_vel_(i) = joint_vector[i].joint.getVelocity();
    }
    KDL::ChainFkSolverPos_recursive jnt_to_pose_solver(chain);
    KDL::Frame pose_current;
    jnt_to_pose_solver.JntToCart(jnt_pos_, pose_current);

    KDL::Twist twist_error = KDL::diff(pose_current, pose_desired);

    /*if (twist_error.vel.Norm() > 1.0) {
      ROS_INFO("twist err mag %f)", twist_error.vel.Norm());
      twist_error.vel.Normalize() * 1.0;
    } */

    publishDeltaMsg(twist_error);
    KDL::Wrench wrench_desi;
    for (unsigned int i=0; i<6; i++)
      wrench_desi(i) = computeCommand(twist_error(i), period, i);

    KDL::Jacobian jacobian(nj);
    KDL::ChainJntToJacSolver jacSolver(chain);
    jacSolver.JntToJac(jnt_pos_, jacobian, -1);
    for (unsigned int i = 0; i < nj; i++){
      commands[i] = 0;
      for (unsigned int j=0; j<6; j++) {
        commands[i] += (jacobian(j,i) * wrench_desi(j));
      }

    }
    // ROS_INFO_THROTTLE(3, "Jac col 5 %f, %f, %f, %f, %f, %f", jacobian(0,5), jacobian(1,5), jacobian(2,5), jacobian(3,5), jacobian(4,5), jacobian(5,5));
    // ROS_INFO_THROTTLE(3, "Jac col 6 %f, %f, %f, %f, %f, %f", jacobian(0,6), jacobian(1,6), jacobian(2,6), jacobian(3,6), jacobian(4,6), jacobian(5,6));
    // d gain joint portion
    for(int i = 0; i < nj; i++) {
      commands[i] += -joint_vector[i].d_gain * jnt_vel_(i);
    }

    // id portion
    for(int i = 0; i < nj; i++) {
      commands[i] += joint_vector[i].id_gain * id_torques(i);
    }

    // null space posture control
    if (posture_control) {
      Eigen::Matrix<double, Eigen::Dynamic, 1>  posture_error(nj,1);
      for (int i = 0; i < nj; i++) {
        posture_error(i, 0) = posture_target[i] - jnt_pos_(i);
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

      Eigen::Matrix<double, Eigen::Dynamic, 1> posture_error_proj = nullspace_proj * (posture_gain * posture_error);


      for(int i = 0; i < nj; i++) {
        commands[i] += posture_error_proj(i, 0);
      }


    }


    for(int i = 0; i < nj; i++) {
      if(std::find(paired_constraints.begin(), paired_constraints.end(), i) == paired_constraints.end()) {
        commands[i] = std::min(std::max(commands[i], joint_vector[i].min_torque), joint_vector[i].max_torque);
      }  
    }

    for (int i = 0; i < paired_constraints.size(); i = i + 2) {
      int lift_index = paired_constraints[i];
      int roll_index = paired_constraints[i+1];
      double lift = commands[lift_index];
      double roll = commands[roll_index];
      //ROS_INFO("lift: %f, roll: %f", commands[lift_index],  commands[roll_index]);

      double max_effort = joint_vector[lift_index].max_torque + joint_vector[roll_index].max_torque;
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
    for (int i = 0; i < joint_vector.size(); i++) {
      //ROS_INFO("command %d: %f", i, commands[i]);
      joint_vector[i].joint.setCommand(commands[i]);
    }
  }

  double CartesianPoseController::computeCommand(double error, ros::Duration dt, int index)
  {
    if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error))
      return 0.0;
    double error_dot = d_error[index];
    if (dt.toSec() > 0.0)  {
      error_dot = (error - p_error_last[index]) /dt.toSec();
      p_error_last[index] = error;
    }
    if (std::isnan(error_dot) || std::isinf(error_dot))
      return 0.0;
    double p_term, d_term;
    d_error[index] = error_dot;

    err_dot_histories[index].push_back(error_dot);
    int size = err_dot_histories[index].size();

    if (err_dot_histories[index].size() > filter_length) {
      err_dot_histories[index].erase(err_dot_histories[index].begin());
    }

    double err_dot_average = std::accumulate(err_dot_histories[index].begin(), err_dot_histories[index].end(),
                              0.0) / err_dot_histories[index].size();
    p_term = p_gains[index] * error;
    d_term = d_gains[index] * err_dot_average;

    return p_term + d_term;
  }

  void CartesianPoseController::gravCallback(const geometry_msgs::Vector3ConstPtr& grav) {
    gravity[0] = grav->x;
    gravity[1] = grav->y;
    gravity[2] = grav->z;
  }

  void CartesianPoseController::jointCallback(const sensor_msgs::JointState msg)
  {
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointPositions(nj);
    KDL::JntArray jointVelocities(nj);
    KDL::JntArray jointAccelerations(nj);
    KDL::Wrenches f_ext;
    for (int i = 0; i < nj; i++) {
      for (int index = 0; index < nj; index++) {
        if (msg.name[i].compare(joint_names[index]) == 0) {
          jointPositions(index) = msg.position[i];
          jointVelocities(index) = msg.velocity[i];
          jointAccelerations(index) = 0.0;
          f_ext.push_back(KDL::Wrench());

          break;
        } else if (index == nj - 1){ 
           ROS_ERROR("No joint %s for controller", msg.name[i].c_str());
        }
      }
    }

    KDL::ChainIdSolver_RNE chainIdSolver(chain, gravity);
    int statusID = chainIdSolver.CartToJnt(jointPositions, jointVelocities, jointAccelerations, f_ext, id_torques); 
    //ROS_INFO("status: %d", statusID);
    //ROS_INFO("pos vel =  %f, %f", msg.position[0], msg.velocity[0]); 

    publishInverseDynamicsMsg();
    //ROS_INFO("nr_segments = %d", ns);
    /**for(int i = 0; i < ns; i++){
      KDL::Segment seg = chain.segments[i];
      ROS_INFO("seg %d name: %s", i, seg.getName().c_str()); 
      ROS_INFO("joint name: %s type: %d", seg.getJoint().getName().c_str(), seg.getJoint().getType()); 
    } **/
    //ROS_INFO("id torques =  %f", id_torques(0)); 
  }

  geometry_msgs::Pose CartesianPoseController::enforceJointLimits(geometry_msgs::Pose commandPose)
  {

    ROS_ERROR("commandPose before %f %f %f", commandPose.position.x, commandPose.position.y, commandPose.position.z);
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointPositions = KDL::JntArray(nj);

    for (int i =0; i < nj; i++) {
      jointPositions(i) =  joint_vector[i].joint.getPosition();
    }

    KDL::Frame cartpos;
    KDL::ChainFkSolverPos_recursive fksolver1(chain);
    int status = fksolver1.JntToCart(jointPositions, cartpos);

    KDL::Jacobian jacobian(nj);
    KDL::ChainJntToJacSolver jacSolver(chain);
    jacSolver.JntToJac(jointPositions, jacobian, -1);

    KDL::JntArray jointInverseKin = KDL::JntArray(nj);
    for (int i = 0; i < nj; i++) {
      jointInverseKin(i) = jointPositions(i);
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic> ee_pose_desired(3,1);
    ee_pose_desired(0, 0) = commandPose.position.x;
    ee_pose_desired(1, 0) = commandPose.position.y;
    ee_pose_desired(2, 0) = commandPose.position.z;

    KDL::Rotation desired_rotation = KDL::Rotation::Quaternion(commandPose.orientation.x,
                                                               commandPose.orientation.y,
                                                               commandPose.orientation.z,
                                                               commandPose.orientation.w
                                                               );

    //ROS_ERROR("desired position: %f, %f, %f", ee_pose_desired(0), ee_pose_desired(1), ee_pose_desired(2));

    for (int j = 0; j < 1000; j++)
    {
      int status = fksolver1.JntToCart(jointInverseKin, cartpos);
      //ROS_INFO("cartpos: %f, %f, %f", cartpos.p.data[0], cartpos.p.data[1], cartpos.p.data[2]);
      jacSolver.JntToJac(jointInverseKin, jacobian, -1);

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
      ROS_INFO("delta joint %f, %f, %f, %f, %f, %f",deltaJoint(0,0), deltaJoint(1,0), deltaJoint(2,0)
      ,deltaJoint(3,0), deltaJoint(4,0), deltaJoint(5,0));

      double alpha = 1; 
      for (int k = 0; k < nj; k++) {
        jointInverseKin(k) = alpha * deltaJoint(k,0) + jointInverseKin(k);
      }
    }

    for (int i = 0; i < nj; i++)  {

      jointInverseKin(i) = fmod(jointInverseKin(i) + M_PI, 2 * M_PI);
      if (jointInverseKin(i) < 0) jointInverseKin(i) += 2 * M_PI;
      jointInverseKin(i) = jointInverseKin(i) - M_PI;
      jointInverseKin(i) = std::min(std::max(jointInverseKin(i), joint_vector[i].min_angle), joint_vector[i].max_angle);
    }

    fksolver1.JntToCart(jointInverseKin, cartpos);
    geometry_msgs::Point point;
    point.x = cartpos.p.data[0];
    point.y = cartpos.p.data[1];
    point.z = cartpos.p.data[2];
    geometry_msgs::Quaternion rot;
    cartpos.M.GetQuaternion(rot.x, rot.y, rot.z, rot.w);
    geometry_msgs::Pose confined_pose;
    confined_pose.position = point;
    confined_pose.orientation = rot;
    // ROS_ERROR("commandPose after %f %f %f", confined_pose.position.x, confined_pose.position.y, confined_pose.position.z);

    return confined_pose;
  }


}


PLUGINLIB_EXPORT_CLASS(koko_controllers::CartesianPoseController, controller_interface::ControllerBase)
