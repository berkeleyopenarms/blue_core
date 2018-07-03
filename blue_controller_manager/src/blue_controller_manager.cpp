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
  float frequency = 0;
  while (ros::ok())
  {
    robot.read();

    ros::Time current_time = ros::Time::now();
    cm.update(current_time, current_time - prev_time);
    prev_time = current_time;

    robot.write();

    ros::Time temp_time = ros::Time::now();

    // TODO: Count number of errors by making updateComs return bool for success
    // If above a threshold, print the number out of 100 or whatever that are bad.
    robot.updateComms();


    frequency += (ros::Time::now() - temp_time).toSec();
    if ((count++%100) == 0) {
      ROS_INFO("Communication frequency is %fHz", (1/(frequency/100)));
      frequency = 0;
    }

    loop_rate.sleep();
  }
}
