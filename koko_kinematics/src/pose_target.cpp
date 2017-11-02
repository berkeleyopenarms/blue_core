#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
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
KDL::Tree my_tree;
KDL::Chain chain;


std::string visualizer = "simple_6dof_MOVE_ROTATE_3D";
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    receivedVisualTarget = false;
    pub = n_.advertise<std_msgs::Float64MultiArray>("/DOF/simple_controller/command", 1000);
    subJoint = n_.subscribe("/DOF/joint_states", 1000, &SubscribeAndPublish::jointCallback, this);
    subVisual = n_.subscribe("/basic_controls/feedback", 1000, &SubscribeAndPublish::visualCallback, this);
    subController = n_.subscribe("/right_controller_pose", 1000, &SubscribeAndPublish::controllerPoseCallback, this);
    subCommand = n_.subscribe("/command_label", 1000, &SubscribeAndPublish::commandCallback, this);
    arrowPub = n_.advertise<visualization_msgs::Marker>("arrow_marker", 0);

    if (!n_.getParam("/DOF/joint_names", joint_names)) {
      ROS_ERROR("No joint_names given (namespace: %s)", n_.getNamespace().c_str());
    }

    command_label = 0;
	  marker.header.frame_id = "base_link";
	  marker.header.stamp = ros::Time();
	  marker.ns = "my_namespace";
	  marker.id = 0;
	  marker.type = visualization_msgs::Marker::ARROW;
	  marker.action = visualization_msgs::Marker::ADD;
	  marker.scale.x = 0.01;
	  marker.scale.y = 0.03;
	  marker.scale.z = 0;
	  marker.color.a = 1.0; // Don't forget to set the alpha!
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


    markerRot.header.frame_id = "base_link";
  	markerRot.header.stamp = ros::Time();
  	markerRot.ns = "my_namespace";
  	markerRot.id = 1;
  	markerRot.type = visualization_msgs::Marker::ARROW;
  	markerRot.action = visualization_msgs::Marker::ADD;
  	markerRot.scale.x = 0.01;
  	markerRot.scale.y = 0.03;
  	markerRot.scale.z = 0;
  	markerRot.color.a = 1.0; // Don't forget to set the alpha!
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


  }
  
  void jointCallback(const sensor_msgs::JointState msg)
  {

    unsigned int nj = my_tree.getNrOfJoints();
    KDL::JntArray jointPositions = KDL::JntArray(nj);


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
  

    //ROS_INFO("desired commandpos with joint count %d:  %f, %f, %f", nj, commandPose.position.x, commandPose.position.y, commandPose.position.z);



    KDL::Frame cartpos;
    int status = fksolver1.JntToCart(jointPositions, cartpos);
    ROS_INFO("status  %d", status);

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

    for (int j = 0; j < 40; j++)
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
      deltaX(0, 0) = pose_difference(0, 0);
      deltaX(1, 0) = pose_difference(1, 0);
      deltaX(2, 0) = pose_difference(2, 0);
      deltaX(3, 0) = 0;
      deltaX(4, 0) = 0;
      deltaX(5, 0) = 0;
      //deltaX(3, 0) = rotation_difference_vec(0);
      //deltaX(4, 0) = rotation_difference_vec(1);
      //deltaX(5, 0) = rotation_difference_vec(2);


      Eigen::MatrixXd deltaJoint = jacPos.transpose() * deltaX;

      
      for (int k = 0; k < nj; k++) {
        jointInverseKin(k) = deltaJoint(k,0) + jointInverseKin(k);
      }
      // ROS_INFO("joint inverse kin: %f, %f, %f, %f", jointInverseKin(0), jointInverseKin(1), jointInverseKin(2), jointInverseKin(3));
      // ROS_INFO("deltaJoint: %f, %f, %f, %f", deltaJoint(0), deltaJoint(1, 0), deltaJoint(2, 0), deltaJoint(3, 0));

    }

    std_msgs::Float64MultiArray commandMsg;
    if (receivedVisualTarget && command_label == 25){
      for (int i = 0; i < nj; i++) {
          commandMsg.data.push_back(jointInverseKin(i));
          //ROS_INFO("published command %d, %f", i, commandMsg.data[i]);
        }
      pub.publish(commandMsg);
    }
    else {
      for(int i = 0; i < nj; i++) {
        commandMsg.data.push_back(jointPositions(i));
        //ROS_INFO("publish command pre %d, %f", i, commandMsg.data[i]);
      }
      pub.publish(commandMsg);

      //ROS_INFO("have not received visual target yet, publishing current joint position as target. command_label=%d receivedVisualTarget=%d", command_label, (int) receivedVisualTarget);
    }
  }


  void visualCallback(const visualization_msgs::InteractiveMarkerFeedback msg) 
  {
    if (!receivedVisualTarget){
      receivedVisualTarget = true;
    }
    if (strcmp(msg.marker_name.c_str(), visualizer.c_str()) == 0) {
      commandPose = msg.pose;
    }
  }

  void controllerPoseCallback(const geometry_msgs::PoseStamped msg) 
  {
    if (!receivedVisualTarget){
      receivedVisualTarget = true;
    }
    
    commandPose = msg.pose;
    
  }

  void commandCallback(const std_msgs::Int32 msg) 
  {
     command_label = msg.data;
    
  }
 

private:
  ros::NodeHandle n_;
  ros::Publisher pub;
  ros::Publisher arrowPub;
  ros::Subscriber subJoint;
  ros::Subscriber subVisual; 
  ros::Subscriber subController;
  ros::Subscriber subCommand;
  geometry_msgs::Pose commandPose;
  visualization_msgs::Marker marker; 
  visualization_msgs::Marker markerRot; 
  std::vector<std::string> joint_names;
  bool receivedVisualTarget;
  int command_label;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "inverse_kin_target");
  ros::NodeHandle node;
  std::string robot_desc_string;

  node.getParam("robot_description", robot_desc_string);

  if(!kdl_parser::treeFromString(robot_desc_string, my_tree)){
    ROS_ERROR("Failed to contruct kdl tree");
    return false;
  }
  std::string end_tracker_link;

  if (!node.getParam("/DOF/endlink_tracker",  end_tracker_link)) {
    ROS_ERROR("No /DOF/endlink_tracker loaded in rosparam");
    return false;
  }

  bool exit_value = my_tree.getChain("base_link", end_tracker_link, chain);
  SubscribeAndPublish sp;

  ros::spin();

  return 0;
}
