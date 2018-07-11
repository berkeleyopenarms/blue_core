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
  
  bool home_mode = false;
  //robot.setControl(false);

  ros::Time prev_time = ros::Time::now();
  int count = 0;
  float frequency = 0;
  int num_errors = 0;
  int num_comms_per_update = 100;
  while (ros::ok())
  {
    if ( !home_mode ) {
      ros::Time temp_time = ros::Time::now();

      robot.read();

      ros::Time current_time = ros::Time::now();
      cm.update(current_time, current_time - prev_time);
      prev_time = current_time;

      robot.write();

      try {
        robot.updateComms();
      } catch (comms_error e) {
        //ROS_ERROR("%s\n", e.what());
        num_errors++;
      }

      // Information for user! 
      frequency += (ros::Time::now() - temp_time).toSec();
      if ((count++%num_comms_per_update) == 0) {
        ROS_INFO("Communication frequency is %fHz. Number of errors in last %d packets: %d", (1/(frequency/num_comms_per_update)), num_comms_per_update, num_errors);
        frequency = 0;
        num_errors = 0;
      }
    }

    loop_rate.sleep();
  }
}
