#include <controller_manager/controller_manager.h>
#include <ros/console.h>

#include "blue_hardware_interface/blue_hardware_interface.h"

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
  ros::Time last_update = ros::Time::now();
  int consecutive_errors = 0;
  int num_errors = 0;
  long count = 0, last_count = 0;
  double sum_freq = 0.0;

  while (ros::ok())
  {
    ros::Time temp_time = ros::Time::now();

    try {
      robot.read();
      consecutive_errors = 0;
    } catch (blue_hardware_drivers::comms_error e) {
      ROS_WARN("%s\n", e.what());
      consecutive_errors++;
      num_errors++;
      if (consecutive_errors == 100) {
        ROS_ERROR("100 consecutive communication errors, exiting...");
        break;
      }
    }

    ros::Time current_time = ros::Time::now();
    cm.update(current_time, current_time - prev_time);
    double frequency = 1.0 / (current_time - prev_time).toSec();
    sum_freq += frequency;
    prev_time = current_time;

    robot.write();

    // Information for user!
    if ((current_time - last_update).toSec() > 1.0) {
      long iters = count - last_count;
      double error_perc = (double)(num_errors) / (double)(iters) * 100.0;
      double avg_freq = sum_freq / (double)(iters);
      ROS_INFO("Communication frequency is %.2fHz with %.2f%% error rate", avg_freq, error_perc);
      last_update = current_time;
      last_count = count;
      num_errors = 0;
      sum_freq = 0.0;
    }

    count++;

    loop_rate.sleep();
  }
}
