#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/jntarray.hpp>
#include <kdl/segment.hpp>
#include <vector>


namespace koko_controllers {



class InverseDynamicsController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{



public:
  bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
  void update(const ros::Time& time, const ros::Duration& period);
  void setCommand(const std_msgs::Float64MultiArrayConstPtr& pos_commands);
  double computeCommand(double error, ros::Duration dt, int index, double vel);
  void starting(const ros::Time& time);
  void pCallback(const std_msgs::Float64MultiArrayConstPtr& p_terms);
  void dCallback(const std_msgs::Float64MultiArrayConstPtr& d_terms);
  ~InverseDynamicsController();


private:
  struct JointPD
  {
    std::string joint_name;
    hardware_interface::JointHandle joint;
    double p_gain;
    double d_gain;
    double d_error;
    double p_error_last;
    double max_torque;
    double min_torque;
    double max_angle;
    double min_angle;
    double cmd;
  };
  KDL::Vector gravity;
  std::vector<std::string> joint_names;
  std::vector<JointPD*> joint_vector;
  std::vector<int> paired_constraints;
  ros::Subscriber sub_command;
  ros::Subscriber sub_p;
  ros::Subscriber sub_d;
  KDL::Chain chain;
  double error_filter_size;
  ros::Publisher commandPub;
  ros::Publisher deltaPub;


};
}

