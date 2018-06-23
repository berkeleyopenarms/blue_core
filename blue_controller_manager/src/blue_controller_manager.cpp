#include <controller_manager/controller_manager.h>
#include <ros/console.h>

#include "blue_controller_manager/blue_hardware_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "blue_controller_manager");

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
  if( ros::console::set_logger_level("ros.controller_manager.pluginlib.ClassLoader", ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
  if( ros::console::set_logger_level("ros.controller_manager", ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle nh("");
  BlueHW robot(nh);
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate loop_rate(1000);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  int count = 0;
  while (ros::ok())
  {
    robot.read();

    ros::Time current_time = ros::Time::now();
    cm.update(current_time, current_time - prev_time);
    prev_time = current_time;

    robot.write();

    ros::Time temp_time = ros::Time::now();
    robot.updateComms();
    if ((count++%100) == 0) {
      ROS_ERROR("Communications took %f seconds", (ros::Time::now() - temp_time).toSec());
    }

    //loop_rate.sleep();
  }
}
