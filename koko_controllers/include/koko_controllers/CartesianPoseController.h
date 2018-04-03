#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/jntarray.hpp>
#include <kdl/segment.hpp>
#include <vector>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>


namespace koko_controllers{



class CartesianPoseController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{



public:
  bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
  void update(const ros::Time& time, const ros::Duration& period);
  void setCommand(const std_msgs::Float64MultiArrayConstPtr& pos_commands);
  double computeCommand(double error, ros::Duration dt, int index);
  void starting(const ros::Time& time);
  void visualCallback(const visualization_msgs::InteractiveMarkerFeedback msg);
  void controllerPoseCallback(const geometry_msgs::PoseStamped msg);
  void commandCallback(const std_msgs::Int32 msg);
  void publishCommandMsg(KDL::Vector desired_position, KDL::Rotation desired_rotation);
  void publishDeltaMsg(KDL::Twist twist_error);
  geometry_msgs::Pose enforceJointLimits(geometry_msgs::Pose commandPose);

private:
  struct JointPD
  {
    std::string joint_name;
    hardware_interface::JointHandle joint;
    double d_gain;
    double max_torque;
    double min_torque;
    double max_angle;
    double min_angle;
  };
  std::vector<std::string> joint_names;
  std::vector<JointPD> joint_vector;
  std::vector<int> paired_constraints;
  ros::Subscriber sub_joint;
  KDL::Chain chain;


  std::string visualizer;
  geometry_msgs::Pose commandPose;
  ros::Subscriber subController;
  ros::Subscriber subVisual;
  ros::Subscriber subCommand;
  ros::Subscriber sub_grav;
  std::vector<double> p_gains;
  std::vector<double> d_gains;
  std::vector<double> p_error_last;
  std::vector<double> d_error;
  std::vector<std::vector<double> > err_dot_histories;

  bool posture_control;
  std::vector<double> posture_target;
  double posture_gain;

  double z_offset_controller;
  int filter_length;
  ros::Publisher commandPub;
  ros::Publisher deltaPub;
  ros::Publisher inverseDynamicsPub;
  std::string target_mode;

};
}

