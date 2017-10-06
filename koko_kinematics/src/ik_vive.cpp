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
KDL::Tree my_tree;
KDL::Chain chain;


//std::string visualizer = "simple_6dof_MOVE_ROTATE_3D"` ;
class SubscribeAndPublish
{
public:

  geometry_msgs::PoseStamped base_tracker_pose;
  geometry_msgs::PoseStamped lift_tracker_pose;

  KDL::JntArray jointCur;
  KDL::JntArray jointOut;
  KDL::Jacobian jacobian;
  KDL::ChainFkSolverPos_recursive fksolver;
  KDL::ChainJntToJacSolver jacSolver;
  int seq;
  unsigned int nj;
  double error_threshold;
  bool base_tracker_read;
  bool lift_tracker_read;
  std::vector<double> calibration_initial_angles;


  int num_iterations;

  SubscribeAndPublish(KDL::Chain chain): fksolver(chain), jacSolver(chain)
  {
    seq = 0;
    pub = n_.advertise<sensor_msgs::JointState>("/joint_state_tracker", 1000);
    pubDeltaPos = n_.advertise<visualization_msgs::Marker>("/deltaPos", 1000);
    pubDeltaRot = n_.advertise<visualization_msgs::Marker>("/deltaRot", 1000);
    base_tracker_read = false;
    lift_tracker_read = false;
    subBaseTracker = n_.subscribe("/base_tracker_pose", 2, &SubscribeAndPublish::baseTrackerCallback, this);
    subLiftTracker = n_.subscribe("/lift_tracker_pose", 2, &SubscribeAndPublish::liftTrackerCallback, this);
    
    ros::NodeHandle node;

    if (!node.getParam("DOF/joint_names", joint_names)) {
      ROS_ERROR("No joint_names given (namespace: %s)", node.getNamespace().c_str());
    }
    
    if (!node.getParam("DOF/calibration_initial_angles", calibration_initial_angles)) {
      ROS_ERROR("No calibration_initial_angles given (namespace: %s)", node.getNamespace().c_str());      
    }


    nj = chain.getNrOfJoints();
    jointCur = KDL::JntArray(nj);
    for (int i = 0; i < calibration_initial_angles.size(); i++) {
      jointCur(i) = calibration_initial_angles[i];
    }


    jointOut = KDL::JntArray(nj);
    jacobian = KDL::Jacobian(nj);
    num_iterations = 10000;
    counter = 0; 
    error_threshold = 0.01;

  }


  
  
  void baseTrackerCallback(const geometry_msgs::PoseStamped msg)
  {
    if (!base_tracker_read)
    {
      base_tracker_read = true;
    }
    base_tracker_pose = msg;
  }

  void liftTrackerCallback(const geometry_msgs::PoseStamped msg)
  {
    if (!lift_tracker_read)
    {
      lift_tracker_read = true;
    }
    lift_tracker_pose = msg;
    update();
  }

  void update()
  {

    if (!(base_tracker_read && lift_tracker_read)){
      ROS_INFO("not received both base and lift trackers");
      return;
    }
    //ROS_INFO("HAVE received both base and lift trackers, kin 6");

    // grab local copies
    geometry_msgs::PoseStamped base_tracker_pose_local = base_tracker_pose;
    geometry_msgs::PoseStamped lift_tracker_pose_local = lift_tracker_pose;

    // desired ee_pose

    Eigen::Matrix<double,3, Eigen::Dynamic> ee_pos_desired(3,1);
 
    ee_pos_desired(0, 0) = lift_tracker_pose_local.pose.position.x;
    ee_pos_desired(1, 0) = lift_tracker_pose_local.pose.position.y;
    ee_pos_desired(2, 0) = lift_tracker_pose_local.pose.position.z;
    //ee_pose_desired(3, 0) = lift_tracker_pose_local.pose.orientation.x;
    //ee_pose_desired(4, 0) = lift_tracker_pose_local.pose.orientation.y;
    //ee_pose_desired(5, 0) = lift_tracker_pose_local.pose.orientation.z;
    //ee_pose_desired(6, 0) = lift_tracker_pose_local.pose.orientation.w;
    KDL::Rotation desired_rotation = KDL::Rotation::Quaternion(lift_tracker_pose_local.pose.orientation.x,
                                                               lift_tracker_pose_local.pose.orientation.y,
                                                               lift_tracker_pose_local.pose.orientation.z,
                                                               lift_tracker_pose_local.pose.orientation.w
                                                               );

    //ROS_INFO("ee_pos_desired: %f %f %f", ee_pos_desired(0), ee_pos_desired(1), ee_pos_desired(2));


    // do inverse kinematics
    for(int iteration = 0; iteration < num_iterations; iteration++)
    {
      jacSolver.JntToJac(jointCur, jacobian, -1);
      Eigen::Matrix<double,6,Eigen::Dynamic> jacMat(6,nj);
   
      
      for (unsigned int joint = 0; joint < nj; joint++) {
        for (unsigned int index = 0; index < 6; index ++) {
          jacMat(index,joint) = jacobian(index,joint);
        }
      } 
      
      // forward kinematics on current joint
      KDL::Frame cartpos;
      int status = fksolver.JntToCart(jointCur, cartpos);


      Eigen::Matrix<double,3, Eigen::Dynamic> ee_pos_cur(3,1);

      for (int i = 0; i < 3; i ++) {
        ee_pos_cur(i, 0) = cartpos.p.data[i];
      }



      KDL::Rotation rotation_difference = desired_rotation * cartpos.M.Inverse();

      KDL::Vector rotation_difference_vec = rotation_difference.GetRot();


      Eigen::Matrix<double, 3, Eigen::Dynamic> pos_difference = ee_pos_desired - ee_pos_cur;
     
      Eigen::Matrix<double, 6, Eigen::Dynamic> deltaX(6,1);
      //deltaX(0, 0) = 0;
      //deltaX(1, 0) = 0;
      //deltaX(2, 0) = 0;
      deltaX(0, 0) = pos_difference(0, 0);
      deltaX(1, 0) = pos_difference(1, 0);
      deltaX(2, 0) = pos_difference(2, 0);
      
      // deltaX(3, 0) = 0;
      // deltaX(4, 0) = 0;
      // deltaX(5, 0) = 0;
      
      deltaX(3, 0) = rotation_difference_vec(0);
      deltaX(4, 0) = rotation_difference_vec(1);
      deltaX(5, 0) = rotation_difference_vec(2);
      
      //ROS_ERROR("iteration %d, pos_difference: (%f, %f, %f, %f, %f, %f)", counter, pos_difference(0, 0), pos_difference(1, 0), pos_difference(2, 0), rotation_difference_vec(0),rotation_difference_vec(2),rotation_difference_vec(2) );
      counter++;
      // change for 6 DOF version

    


      Eigen::MatrixXd deltaJoint = jacMat.transpose() * deltaX;
      
      float alpha = 0.1;
      for(int j = 0; j < nj; j++)
      {
        jointCur(j) = jointCur(j) + alpha * deltaJoint(j, 0);
       // ROS_ERROR("deltaJoint: %f", deltaJoint(j,0));
      } 

      int below_threshold = 1; 
      for (int i = 0; i < 6; i++) {
        if (std::fabs(deltaX(i, 0)) > error_threshold) {
          below_threshold = 0;
        }
      } 
      if (below_threshold == 1) {
        //ROS_ERROR("below threshold break");
      }
    }


    //ROS_INFO("jointCur: %f", jointCur(0));



    // publish and increment and save
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

    pub.publish(joint_state_msg);
    //jointCur = jointOut;
    seq++; 
  }
  

  static void toEulerianAngle(double x, double y, double z, double w, double& roll, double& pitch, double& yaw)
  {
    double ysqr = y * y;

    // roll (x-axis rotation)
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + ysqr);
    roll = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    double t2 = +2.0 * (w * y - z * x);
    t2 = ((t2 > 1.0) ? 1.0 : t2);
    t2 = ((t2 < -1.0) ? -1.0 : t2);
    pitch = std::asin(t2);

    // yaw (z-axis rotation)
    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (ysqr + z * z);  
    yaw = std::atan2(t3, t4);
  }




private:
  ros::NodeHandle n_;
  ros::Publisher pub;
  ros::Publisher pubDeltaPos;
  ros::Publisher pubDeltaRot;
  ros::Subscriber subBaseTracker;
  ros::Subscriber subLiftTracker;
  std::vector<std::string> joint_names;
  int counter;
};


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

  std::string end_tracker_link;

  if (!node.getParam("/DOF/endlink_tracker",  end_tracker_link)) {
    ROS_ERROR("No /DOF/endlink_tracker loaded in rosparam");
  }

  bool exit_value = my_tree.getChain("base_tracker_link", end_tracker_link, chain);
  SubscribeAndPublish sp(chain); 


/*
  ros::Rate  loop_rate(90);
  while(ros::ok){
    
    ros::spinOnce();
    loop_rate.sleep();
  }
*/
  ros::spin();


  return 0; 
}
