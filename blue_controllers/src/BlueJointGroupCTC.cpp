#include <blue_controllers/BlueJointGroupCTC.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>


namespace blue_controllers
{

  BlueJointGroupCTC::BlueJointGroupCTC() {}
  BlueJointGroupCTC::~BlueJointGroupCTC() {sub_command_.shutdown();}

  bool BlueJointGroupCTC::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  {
    // List of controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    n_joints_ = joint_names_.size();
    ROS_INFO("num joints n_joints %d", n_joints_);

    if(n_joints_ == 0){
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }


    std::string robot_desc_string;
    if (!n.getParam("/robot_description", robot_desc_string)) {
      ROS_ERROR("No /robot_description given node namespace %s", n.getNamespace().c_str());
    }

    KDL::Tree my_tree;
    if(!kdl_parser::treeFromString(robot_desc_string, my_tree)){
      ROS_ERROR("Failed to contruct kdl tree");
      return false;
    }
    std::string k_endlink;
    if (!n.searchParam("blue_hardware/endlink", k_endlink)) {
      ROS_ERROR("No endlink given node namespace %s", n.getNamespace().c_str());
    }
    std::string endlink;
    if (!n.getParam(k_endlink, endlink)) {
      ROS_ERROR("No endlink given node namespace %s", n.getNamespace().c_str());
    }
    std::string k_baselink;
    if (!n.searchParam("blue_hardware/baselink", k_baselink)) {
      ROS_ERROR("No baselink given node namespace %s", n.getNamespace().c_str());
    }
    std::string baselink;
    if (!n.getParam(k_baselink, baselink)) {
      ROS_ERROR("No baselink given node namespace %s", n.getNamespace().c_str());
    }

    if (!n.getParam("ctc_weight", ctc_weight)) {
      ROS_ERROR("No ctc_weight given node namespace %s", n.getNamespace().c_str());
    }

    KDL::Chain dummyChain;
    if (!my_tree.getChain(baselink, endlink, dummyChain)) {
      ROS_ERROR("Failed to construct dummy kdl chain");
      return false;
    }

    // Get URDF
    urdf::Model urdf;
    if (!urdf.initParam("robot_description"))
    {
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }

    int ns = dummyChain.getNrOfSegments();
    for(int i = 0; i < ns; i++){
      KDL::Segment seg = dummyChain.segments[i];
      if (seg.getJoint().getType() != 8)
      {
        chain.addSegment(seg);
      }
    }

    // ROS_ERROR_STREAM("1 Starting Controller");
    pid_controllers_.resize(n_joints_);
    torques = KDL::JntArray(n_joints_);

    // ROS_ERROR_STREAM("2 Starting Controller");
    for(unsigned int i=0; i<n_joints_; i++)
    {
      ROS_INFO("Starting Controller %d", i);
      try
      {
        joints_.push_back(hw->getHandle(joint_names_[i]));
        ROS_INFO("Joint Name %s", joint_names_[i].c_str());
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }

      urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
      if (!joint_urdf)
      {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
        return false;
      }
      joint_urdfs_.push_back(joint_urdf);

      // Load PID Controller using gains set on parameter server
      if (!pid_controllers_[i].init(ros::NodeHandle(n, joint_names_[i] + "/pid")))
      {
        ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
        return false;
      }
    }

    first_update_ = true;

    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &BlueJointGroupCTC::commandCB, this);
    ROS_INFO("Starting Controller");
    return true;
  }

  void BlueJointGroupCTC::update(const ros::Time& time, const ros::Duration& period)
  {
    std::vector<double> commands;

    if (first_update_)
    {
      for (unsigned int i=0; i<n_joints_; i++)
      {
        commands.push_back(joints_[i].getPosition());
      }
      commands_buffer_.writeFromNonRT(commands);

      first_update_ = false;
    }
    else
    {
      commands = *commands_buffer_.readFromRT();
    }

    // Check if the change between this position and previous position is greater than a threshhold.
    //antiJump(commands);

    std::vector<double> pid_terms(n_joints_);
    for(unsigned int i=0; i<n_joints_; i++)
    {
        double command_position = commands[i];

        double error; //, vel_error;
        double commanded_effort;

        double current_position = joints_[i].getPosition();

        // Make sure joint is within limits if applicable
        enforceJointLimits(command_position, i);

        // Compute position error
        if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
        {
         angles::shortest_angular_distance_with_limits(
            current_position,
            command_position,
            joint_urdfs_[i]->limits->lower,
            joint_urdfs_[i]->limits->upper,
            error);
        }
        else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
        {
          error = angles::shortest_angular_distance(current_position, command_position);
        }
        else //prismatic
        {
          error = command_position - current_position;
        }

        // Set the PID error and compute the PID command with nonuniform
        // time step size.
        commanded_effort = pid_controllers_[i].computeCommand(error, -joints_[i].getVelocity(), period);

        pid_terms[i] = commanded_effort;
    }
    KDL::JntArray jointPositions(n_joints_);
    KDL::JntArray jointVelocities(n_joints_);
    KDL::JntArray jointAccelerations(n_joints_);
    KDL::Wrenches f_ext;
    for (int i = 0; i < n_joints_; i++) {
        jointPositions(i) = joints_[i].getPosition();
        jointVelocities(i) = joints_[i].getVelocity();
        jointAccelerations(i) = pid_terms[i];
        f_ext.push_back(KDL::Wrench());
    }

    KDL::Vector gravity(0, 0, 0);
    KDL::ChainIdSolver_RNE chainIdSolver(chain, gravity);
    int statusID = chainIdSolver.CartToJnt(jointPositions, jointVelocities, jointAccelerations, f_ext, torques);

    for (int i = 0; i < n_joints_; i++) {
        joints_[i].setCommand(ctc_weight * torques(i) + (1.0 - ctc_weight) * pid_terms[i]);
    }
  }

  void BlueJointGroupCTC::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
  {
    if(msg->data.size()!=n_joints_)
    {
      ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
      return;
    }
    commands_buffer_.writeFromNonRT(msg->data);
  }

  void BlueJointGroupCTC::enforceJointLimits(double &command, unsigned int index)
  {
    // Check that this joint has applicable limits
    if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
    {
      if( command > joint_urdfs_[index]->limits->upper ) // above upper limnit
      {
        command = joint_urdfs_[index]->limits->upper;
      }
      else if( command < joint_urdfs_[index]->limits->lower ) // below lower limit
      {
        command = joint_urdfs_[index]->limits->lower;
      }
    }
  }
} // namespace

PLUGINLIB_EXPORT_CLASS( blue_controllers::BlueJointGroupCTC, controller_interface::ControllerBase)
