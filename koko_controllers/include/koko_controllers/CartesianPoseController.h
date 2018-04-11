#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_toolbox/pid.h>
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
  double computeCommand(double error, double error_dot, const ros::Duration& dt, int index);
  void starting(const ros::Time& time);

private:
  void controllerPoseCallback(const geometry_msgs::PoseStamped msg);
  void commandCallback(const std_msgs::Int32 msg);
  void publishCommandMsg(KDL::Vector desired_position, KDL::Rotation desired_rotation);
  void publishDeltaMsg(KDL::Twist twist_error);
  geometry_msgs::Pose enforceJointLimits(geometry_msgs::Pose command_pose_);

  struct JointSettings
  {
    std::string joint_name;
    hardware_interface::JointHandle joint;
    double d_gain;
    double max_torque;
    double min_torque;
    double max_angle;
    double min_angle;
    double pos_mult;
    double rot_mult;
  };

  std::vector<std::string> joint_names_;
  std::vector<JointSettings> joint_vector_;
  std::vector<int> paired_constraints_;
  ros::Subscriber sub_joint;
  KDL::Chain kdl_chain_;

  geometry_msgs::Pose command_pose_;
  ros::Subscriber sub_command_;

  std::vector<control_toolbox::Pid> pid_controllers_;

  std::vector<std::vector<double> > d_error_history_;

  bool posture_control_;
  std::vector<double> posture_target_;
  double posture_gain_;

  int filter_length_;
  ros::Publisher pub_command_;
  ros::Publisher pub_delta_;
  std::string target_mode;

};
}
