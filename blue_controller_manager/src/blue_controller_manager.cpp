#include <controller_manager/controller_manager.h>
#include <ros/console.h>
#include <thread>

#include "blue_controller_manager/blue_hardware_interface.h"
/*
void commsThread(BlueHW & robot) {
  ros::Rate comms_rate(1000);
  while (ros::ok())
  {
    robot.updateComms();
    comms_rate.sleep();
  }
}
*/

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
  //controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate loop_rate(1000);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  //std::thread comms(commsThread, robot);

  ros::Time prev_time = ros::Time::now();
  while (ros::ok())
  {
    robot.read();

    ros::Time current_time = ros::Time::now();
    //cm.update(current_time, current_time - prev_time);
    prev_time = current_time;

    robot.write();

    robot.updateComms();

    loop_rate.sleep();
  }

  //comms.join();
}
