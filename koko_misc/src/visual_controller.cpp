#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <math.h>

using namespace visualization_msgs;


// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
ros::Publisher pose_pub;
interactive_markers::MenuHandler menu_handler;
// %EndTag(vars)%


// %Tag(Box)%
Marker makeBox(InteractiveMarker &msg)
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl(InteractiveMarker &msg)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(processFeedback)%
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::ostringstream s;
  geometry_msgs::PoseStamped p;
  switch (feedback->event_type)
  {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM(s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec");

      p.header = feedback->header;
      p.pose = feedback->pose;
      pose_pub.publish(p);
      break;

    default:
      break;
  }

  server->applyChanges();
}
// %EndTag(processFeedback)%

////////////////////////////////////////////////////////////////////////////////////

// %Tag(6DOF)%
void make6DofMarker(bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = .15;

  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";
  int_marker.pose.orientation.w = 0.0;
  int_marker.pose.orientation.x = 1.0;
  int_marker.pose.orientation.y = 1.0;
  int_marker.pose.orientation.z = 0.0;

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if (fixed)
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D)         mode_text = "MOVE_3D";
      if(interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D)       mode_text = "ROTATE_3D";
      if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D)  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    menu_handler.apply(*server, int_marker.name);
}
// %EndTag(6DOF)%

void frameCallback(const ros::TimerEvent&) {}

// %Tag(main)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_controller");
  ros::NodeHandle n;
  ros::Timer frame = n.createTimer(ros::Duration(0.05), frameCallback);

  server.reset(new interactive_markers::InteractiveMarkerServer("cartesian_pose_teleop","",false));

  ros::Duration(0.1).sleep();

  menu_handler.insert("First Entry", &processFeedback);
  menu_handler.insert("Second Entry", &processFeedback);
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert("Submenu");
  menu_handler.insert(sub_menu_handle, "First Entry", &processFeedback);
  menu_handler.insert(sub_menu_handle, "Second Entry", &processFeedback);

  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/koko_controllers/cartesian_pose_controller/target_pose", 1);

  tf::TransformListener listener(n);
  ros::Duration(5).sleep();
  // TODO: parameterize, add support for two arms
  while(!listener.canTransform("world", "wrist_roll_link", ros::Time(0))) {
    ROS_INFO("Waiting for world->wrist_roll_link transform...");
    ros::Duration(1).sleep();
  }
  tf::StampedTransform end_effector_position;
  listener.lookupTransform("world", "wrist_roll_link", ros::Time(0), end_effector_position);

  make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, end_effector_position.getOrigin(), true);

  server->applyChanges();

  ros::spin();

  server.reset();
}
