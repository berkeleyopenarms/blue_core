#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <std_msgs/Float64.h>
#include <string>
#include <vector>
#include <kdl/frames.hpp>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
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
#include <tf2/LinearMath/Transform.h>
#include <koko_hardware_drivers/MotorState.h>

KDL::Tree my_tree;
KDL::Chain chain;
std::vector<KDL::Chain> chains;
unsigned int nj;
std::vector<double> hardstop_start_angles;
std::vector<std::string> joint_names;
KDL::JntArray jointCur;
ros::Publisher pub;



int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_state_tracker");
  ros::NodeHandle node;
  std::string robot_desc_string;
  node.getParam("robot_description", robot_desc_string);

  if(!kdl_parser::treeFromString(robot_desc_string, my_tree)){
    ROS_ERROR("Failed to contruct kdl tree");
    return false;
  }

  std::string ee_tracker;
  if (!node.getParam("/koko_hardware/ee_tracker",  ee_tracker)) {
    ROS_ERROR("No /koko_hardware/koko_hardware loaded in rosparam");
  }

  bool exit_value = my_tree.getChain("base_tracker_link", ee_tracker, chain);
  ros::Duration(2.0).sleep();

  // publisher and subscriber setup
  pub = node.advertise<sensor_msgs::JointState>("/joint_state_tracker", 1000);

  if (!node.getParam("koko_hardware/joint_names", joint_names)) {
    ROS_ERROR("No joint_names given (namespace: %s)", node.getNamespace().c_str());
  }

  // getting hardstop angles
  if (!node.getParam("koko_hardware/hardstop_start_angles", hardstop_start_angles)) {
    ROS_ERROR("No hardstop_start_angles given (namespace: %s)", node.getNamespace().c_str());
  }
  nj = hardstop_start_angles.size();
  jointCur = KDL::JntArray(nj);
  for (int i = 0; i < nj; i++) {
    jointCur(i) = hardstop_start_angles[i];
  }

  // publish and increment and save
  int seq = 0;
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.seq = seq;
  joint_state_msg.header.frame_id = "base_link";
  joint_state_msg.header.stamp = ros::Time::now();

  for (int i = 0; i < joint_names.size(); i++) {
    joint_state_msg.name.push_back(joint_names[i]);
    joint_state_msg.position.push_back(jointCur(i));
    joint_state_msg.velocity.push_back(0.0);
    joint_state_msg.effort.push_back(0.0);
  }

  ros::Rate  loop_rate(500);

  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
    pub.publish(joint_state_msg);
  }

  ros::spin();

  return 0;
}

