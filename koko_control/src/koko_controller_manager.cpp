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

  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (ros::ok())
  {
    robot.read();
    cm.update(robot.get_time(), robot.get_period());
    robot.write();
    loop_rate.sleep();
  }
}
