#include <koko_control/koko_hardware_interface.h>
#include <controller_manager/controller_manager.h>
#include <vector>
#include <ros/console.h>
#include <pthread.h>

typedef struct update_struct{
  KokoHW *robot;
  controller_manager::ControllerManager *manager;
} update_struct;

void ExecuteUpdate(update_struct *my_data) {
   (* my_data->robot).read();
   (* my_data->manager).update((* my_data->robot).get_time(), (* my_data->robot).get_period());
   (* my_data->robot).write();
   ros::spinOnce();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "handler");

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
  ros::spinOnce();

  ros::Rate loop_rate(1000);

  update_struct us;
  us.manager = &cm;
  us.robot = &robot;

  ExecuteUpdate(&us);

  cm.loadController("koko_controllers/joint_state_controller");
  ROS_INFO("Loaded Joint State Controller");
  std::vector<std::string> controllers_state_start;
  controllers_state_start.push_back("koko_controllers/joint_state_controller");
  std::vector<std::string> controllers_stop;
  cm.switchController(controllers_state_start, controllers_stop, 1);

  while (robot.getPositionRead() != 1) {
    ROS_INFO_ONCE("Waiting for first joint state message read");
  }
  ROS_INFO("Got position and loading controller");

  cm.loadController("koko_controllers/joint_position_controller");
  ROS_INFO("Loaded Controller");

  std::vector<std::string> controllers_start;
  controllers_start.push_back("koko_controllers/joint_position_controller");

  cm.switchController(controllers_start, controllers_stop, 1);
  //ROS_ERROR("Here2");
  while (ros::ok())
  {
    ExecuteUpdate(&us);
    loop_rate.sleep();
  }

}
