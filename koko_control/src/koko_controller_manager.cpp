#include <koko_control/koko_hardware_interface.h>
#include <controller_manager/controller_manager.h>
#include <vector>
#include <ros/console.h>
#include <pthread.h>

struct thread_struct{
  KokoHW *robot;
  controller_manager::ControllerManager *manager;
  ros::Rate *loop_rate;
};

void *ExecuteUpdate(void *threadarg) {

  struct thread_struct *my_data;
  my_data = (struct thread_struct *) threadarg;

  while (ros::ok())
  {
     (* my_data->robot).read();
     (* my_data->manager).update((* my_data->robot).get_time(), (* my_data->robot).get_period());
     (* my_data->robot).write();
     ros::spinOnce();
     (* my_data->loop_rate).sleep();
  }
  pthread_exit(NULL);
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
  int x = 0;
  ros::NodeHandle nh("");
  KokoHW robot(nh);
  controller_manager::ControllerManager cm(&robot, nh);
  ros::spinOnce();

  ros::Rate loop_rate(1000);
  ros::Rate loop_rate2(500);

  thread_struct ts;
  ts.manager = &cm;
  ts.robot = &robot;
  ts.loop_rate = &loop_rate;
  pthread_t thread;
  int rc = pthread_create(&thread, NULL, ExecuteUpdate, (void *) &ts);

  cm.loadController("koko_controllers/joint_state_controller");
  std::vector<std::string> controllers_state_start;
  controllers_state_start.push_back("koko_controllers/joint_state_controller");
  std::vector<std::string> controllers_stop;
  cm.switchController(controllers_state_start, controllers_stop, 1);

  while (robot.getPositionRead() != 1) {
    ROS_INFO_ONCE("Waiting for first joint state message read");
  }

  cm.loadController("koko_controllers/joint_position_controller");
  // cm.loadController("koko_controllers/cartesian_pose_controller");
  std::vector<std::string> controllers_start;
  controllers_start.push_back("koko_controllers/joint_position_controller");
  //controllers_start.push_back("koko_controllers/cartestian_pose_controller");

  // cm.loadController("koko_controllers/cartesian_pose_controller");
  // std::vector<std::string> controllers_start;
  // controllers_start.push_back("koko_controllers/cartesian_pose_controller");

  cm.switchController(controllers_start, controllers_stop, 1);
  while (ros::ok())
  {
    loop_rate2.sleep();
  }


}
