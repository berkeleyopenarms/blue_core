#include <controller_manager/controller_manager.h>
#include <vector>
#include <ros/console.h>
#include <pthread.h>
#include "koko_control/koko_hardware_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "koko_controller_manager");

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
  KokoHW robot(nh);
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate loop_rate(200);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  while (ros::ok())
  {
    robot.read();

    ros::Time current_time = ros::Time::now();
    cm.update(current_time, current_time - prev_time);
    prev_time = current_time;

    robot.write();

    loop_rate.sleep();
  }
}
