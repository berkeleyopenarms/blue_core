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

  //robot.setControl(false);

  ros::Time prev_time = ros::Time::now();
  int num_errors = 0;
  long count = 0;

  while (ros::ok())
  {
    ros::Time temp_time = ros::Time::now();

    try {
      robot.read();
      num_errors = 0;
    } catch (comms_error e) {
      ROS_WARN_THROTTLE(1.0, "%s\n", e.what());
      num_errors++;
      if (num_errors == 100) {
        ROS_ERROR("100 consecutive communication errors, exiting...");
        break;
      }
    }

    ros::Time current_time = ros::Time::now();
    cm.update(current_time, current_time - prev_time);
    double frequency = 1.0 / (current_time - prev_time).toSec();
    prev_time = current_time;

    robot.write();

    // Information for user!
    ROS_INFO_THROTTLE(1.0, "Communication frequency is %.2fHz", frequency);
    count++;

    loop_rate.sleep();
  }
}
