#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include <vector>
#include <kdl/frames.hpp>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

std::string visualizer = "simple_6dof_MOVE_ROTATE_3D";
class Arm
{
public:


  Arm(int index, KDL::Tree tree, KDL::Chain chainz)
  {
    my_tree = tree;
    receivedCommandYet = false;
    chain = chainz; 
    jointJacIndex = index; 
    commandPub = n_.advertise<std_msgs::Float64MultiArray>("/3dof/joint_position_controller/command", 1000);
    subJoint = n_.subscribe("/3dof/joint_states", 1000, &Arm::jointCallback, this);
    subVisual = n_.subscribe("/basic_controls/feedback", 1000, &Arm::visualCallback, this);
    statePub = n_.advertise<std_msgs::Float64MultiArray>("/3dof/joint_states_wrap", 0);
    deltaPub = n_.advertise<std_msgs::Float64MultiArray>("/3dof/delta_pose", 0);

    arrowPub = n_.advertise<visualization_msgs::Marker>("arrow_marker", 0);

    subFKIndex = n_.subscribe("/fk_index", 1000, &Arm::fkindexCallback, this);

    int ns = chain.getNrOfSegments();

    ROS_INFO("chain segment size = %i",ns);

    for(int i = 0; i < ns; i++){
      KDL::Segment seg = chain.segments[i];
      ROS_INFO("seg %d name: %s", i, seg.getName().c_str()); 
      ROS_INFO("joint name: %s type: %d", seg.getJoint().getName().c_str(), seg.getJoint().getType()); 
    } 

    for(int i = 0; i < ns; i++){
      KDL::Segment seg = chain.segments[i];
      if (seg.getJoint().getType() != 8)
      { 
        std::string jointName = seg.getJoint().getName();
        joint_names.push_back(jointName);
      }
    }


    fk_index = -1;

    commandPose.position.x = 0.5;
    commandPose.position.y = -0.38;
    commandPose.position.z = 1.9;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.03;
    marker.scale.z = 0;
    marker.color.a = 1.0; 
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    geometry_msgs::Point p2;
    p2.x = 1;
    p2.y = 0;
    p2.z = 0;
    marker.points.push_back(p);
    marker.points.push_back(p2);
    markerRot.header.frame_id = "world";
    markerRot.header.stamp = ros::Time();
    markerRot.ns = "my_namespace";
    markerRot.id = 1;
    markerRot.type = visualization_msgs::Marker::ARROW;
    markerRot.action = visualization_msgs::Marker::ADD;
    markerRot.scale.x = 0.01;
    markerRot.scale.y = 0.03;
    markerRot.scale.z = 0;
    markerRot.color.a = 1.0;
    markerRot.color.r = 0.0;
    markerRot.color.g = 1.0;
    markerRot.color.b = 1.0;
    geometry_msgs::Point p3;
    p3.x = 0;
    p3.y = 0;
    p3.z = 0;
    geometry_msgs::Point p4;
    p4.x = 1;
    p4.y = 0;
    p4.z = 0;
    markerRot.points.push_back(p3);
    markerRot.points.push_back(p4);
    initialized = 0;

  }
  
  void jointCallback(const sensor_msgs::JointState msg)
  {

    unsigned int nj = my_tree.getNrOfJoints();
    KDL::JntArray jointPositions = KDL::JntArray(nj);




    std_msgs::Float64MultiArray wrapMsg;

    for (int i = 0; i < nj; i++) {
      double wraparound = msg.position[i];
      wraparound = std::fmod(wraparound + 3.14159265359, 6.28318530718);
      if (wraparound< 0) wraparound += 6.28318530718;
      wraparound = wraparound - 3.1415926535;
      wrapMsg.data.push_back(wraparound);
    }
    statePub.publish(wrapMsg);


    for (int i = 0; i < nj; i++) {
      for (int index = 0; index < nj; index++) {
        if (msg.name[i].compare(joint_names[index]) == 0) {
          jointPositions(index) = msg.position[i];
          break;
        } else if (index == nj - 1){ 
           ROS_ERROR("No joint %s for controller", msg.name[i].c_str());
        }
      }
    } 

    //ROS_INFO("current joint pos %f, %f, %f", msg.position[0], msg.position[1], msg.position[2]);

    KDL::JntArray jointOut = KDL::JntArray(nj);

    KDL::ChainFkSolverPos_recursive fksolver1(chain);
  

   // ROS_INFO("desired commandpos %f, %f, %f", commandPose.position.x, commandPose.position.y, commandPose.position.z);



    KDL::Frame cartpos;
    int status = fksolver1.JntToCart(jointPositions, cartpos);


    KDL::Jacobian jacobian(nj);
    KDL::ChainJntToJacSolver jacSolver(chain);
    jacSolver.JntToJac(jointPositions, jacobian, -1);
    //ROS_INFO("j_x %f", jacobian(0,0));
    //ROS_INFO("j_y %f", jacobian(1,0));
    //ROS_INFO("j_z %f", jacobian(2,0));
    marker.points[0].x = cartpos.p.data[0];
    marker.points[0].y = cartpos.p.data[1];
    marker.points[0].z = cartpos.p.data[2];
    marker.points[1].x = cartpos.p.data[0] + jacobian(0,0);
    marker.points[1].y = cartpos.p.data[1] + jacobian(1,0);
    marker.points[1].z = cartpos.p.data[2] + jacobian(2,0);
    
    markerRot.points[0].x = cartpos.p.data[0];
    markerRot.points[0].y = cartpos.p.data[1];
    markerRot.points[0].z = cartpos.p.data[2];
    markerRot.points[1].x = cartpos.p.data[0] + jacobian(3,0);
    markerRot.points[1].y = cartpos.p.data[1] + jacobian(4,0);
    markerRot.points[1].z = cartpos.p.data[2] + jacobian(5,0);

    //ROS_INFO("j_rx %f", jacobian(3,0));

    arrowPub.publish(marker);
    arrowPub.publish(markerRot);




    if (initialized == 0) {
      jointInverseKin = KDL::JntArray(nj);
      for (int i = 0; i < nj; i++) {
        jointInverseKin(i) = jointPositions(i);
      }
      initialized = 1;
    }
    if(!receivedCommandYet){
        return;
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

    for (int j = 0; j < 1; j++)
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
      //deltaX(0, 0) = 0;
      //deltaX(1, 0) = 0;
      //deltaX(2, 0) = 0;
      deltaX(0, 0) = pose_difference(0);
      deltaX(1, 0) = pose_difference(1);
      deltaX(2, 0) = pose_difference(2);
      deltaX(3, 0) = 0;
      deltaX(4, 0) = 0;
      deltaX(5, 0) = 0;
      //deltaX(3, 0) = rotation_difference_vec(0);
      //deltaX(4, 0) = rotation_difference_vec(1);
      //deltaX(5, 0) = rotation_difference_vec(2);


      std_msgs::Float64MultiArray deltaMsg;

      for (int i = 0; i < 6; i++) {
        deltaMsg.data.push_back(deltaX(i,0));
      }
      deltaPub.publish(deltaMsg);


      /*Eigen::MatrixXd jacobianPosition(3, nj);
      Eigen::MatrixXd jacobianRotation(3, nj);

      for (int row = 0; row < 3; row ++) {
        for (int col = 0; col < nj; col ++) {
          jacobianPosition(row, col) = jacPos(row, col);
          jacobianRotation(row, col) = jacPos(row + 3, col);
        }
      }  

      Eigen::MatrixXd jacobianPosition_psuedo = pseudoinverse(jacobianPosition, 0.0001);
      Eigen::MatrixXd jacobianRotation_psuedo = pseudoinverse(jacobianRotation, 0.0001);


      Eigen::Matrix<double, 3, Eigen::Dynamic> deltaXPos(3,1);
      deltaXPos(0, 0) = deltaX(0, 0); 
      deltaXPos(1, 0) = deltaX(1, 0); 
      deltaXPos(2, 0) = deltaX(2, 0); 
     
      Eigen::Matrix<double, 3, Eigen::Dynamic> deltaXRot(3,1);
      deltaXRot(0, 0) = deltaX(3, 0); 
      deltaXRot(1, 0) = deltaX(4, 0); 
      deltaXRot(2, 0) = deltaX(5, 0); 


      Eigen::MatrixXd posA = jacobianPosition_psuedo * deltaXPos;
      Eigen::MatrixXd rotA = jacobianRotation_psuedo * deltaXRot;

      ROS_INFO("posA %f, %f, %f, %f, %f, %f", posA(0,0), posA(1,0), posA(2,0), posA(3,0), posA(4,0), posA(5,0) );


      Eigen::MatrixXd null_rot_posA = posA - jacobianRotation_psuedo * (jacobianRotation * posA);
      Eigen::MatrixXd null_pos_rotA = rotA - jacobianPosition_psuedo * (jacobianPosition * rotA);

      ROS_INFO("null_rot_posA %f, %f, %f, %f, %f, %f", null_rot_posA(0,0), null_rot_posA(1,0), null_rot_posA(2,0), null_rot_posA(3,0), null_rot_posA(4,0), null_rot_posA(5,0));
      */

      Eigen::MatrixXd deltaJoint = jacPos.transpose() * deltaX;

      for (int k = 0; k < nj; k++) {
        //jointInverseKin(k) = null_pos_rotA(k,0) + null_rot_posA(k,0) + jointInverseKin(k);
        jointInverseKin(k) = deltaJoint(k,0) + jointInverseKin(k);
      }


    }



    std_msgs::Float64MultiArray commandMsg;
    if (receivedCommandYet){
      for (int i = 0; i < nj; i++) {
          double wraparound = jointInverseKin(i);
          wraparound = fmod(wraparound + 3.14159265359, 6.28318530718);
          if (wraparound< 0) wraparound += 6.28318530718;
          wraparound = wraparound - 3.1415926535;

          commandMsg.data.push_back(wraparound);
        }
      commandPub.publish(commandMsg);
    }

  }



 Eigen::MatrixXd  pseudoinverse(const Eigen::MatrixXd &mat, double tolerance) // choose appropriately
  {
      Eigen::JacobiSVD<Eigen::MatrixXd> svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
      const Eigen::MatrixXd &singularValues = svd.singularValues();
      Eigen::Matrix<double, Eigen::MatrixXd::ColsAtCompileTime, Eigen::MatrixXd::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
      singularValuesInv.setZero();
      for (unsigned int i = 0; i < singularValues.size(); ++i) {
          if (singularValues(i) > tolerance)
          {
              singularValuesInv(i, i) = 1 / singularValues(i);
          }
          else
          {
              singularValuesInv(i, i) = 0;
          }
      }
      return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
  }

  void visualCallback(const visualization_msgs::InteractiveMarkerFeedback msg) 
  {
    if (strcmp(msg.marker_name.c_str(), visualizer.c_str()) == 0) {
      if(!receivedCommandYet){
        receivedCommandYet = true;
      }
      commandPose = msg.pose;
    }
  }

  void fkindexCallback(const std_msgs::Int32 msg) 
  {
    fk_index = (int)msg.data;
  }

private:
  ros::NodeHandle n_;
  ros::Publisher commandPub;
  ros::Publisher arrowPub;
  ros::Publisher statePub;
  ros::Publisher deltaPub;

  ros::Subscriber subJoint;
  ros::Subscriber subVisual; 
  ros::Subscriber subFKIndex; 
  geometry_msgs::Pose commandPose;
  visualization_msgs::Marker marker; 
  visualization_msgs::Marker markerRot; 
  KDL::Tree my_tree;
  KDL::Chain chain;
  int jointJacIndex; 
  bool receivedCommandYet;
  int fk_index;
  std::vector<std::string> joint_names;
  KDL::JntArray jointInverseKin;
  int initialized;
};

int main(int argc, char** argv)
{
 
  if (argc != 2) {
    std::cout << "Error, requires joint index argument" << std::endl;
    return EXIT_FAILURE;
  } 
  
  ros::init(argc, argv, "handler");
  ros::NodeHandle node;
  std::string robot_desc_string;

  if (!node.getParam("robot_kin_description", robot_desc_string)) {
    ROS_ERROR("Failed to get robot kin description");
  }

  KDL::Tree my_tree;
  KDL::Chain chain;
  if(!kdl_parser::treeFromString(robot_desc_string, my_tree)){
    ROS_ERROR("Failed to contruct kdl tree");
    return false; 
  }
  std::string end_tracker_link;

  if (!node.getParam("/3dof/endlink_tracker",  end_tracker_link)) {
    ROS_ERROR("No /3dof/endlink_tracker loaded in rosparam");
  }

  bool exit_value = my_tree.getChain("world", end_tracker_link, chain);

  ROS_INFO(" exit value= %d", exit_value);

  Arm sp(atoi(argv[1]), my_tree, chain);
  ros::spin();

  return 0; 
}
