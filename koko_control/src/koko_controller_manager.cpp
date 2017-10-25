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

  ros::NodeHandle nh("");
  KokoHW robot(nh);
  controller_manager::ControllerManager cm(&robot, nh);
  ros::spinOnce();
  
  
  ros::Rate loop_rate(100);
  ros::Rate loop_rate2(500);
  
  
  thread_struct ts;
  ts.manager = &cm;
  ts.robot = &robot;
  ts.loop_rate = &loop_rate;
  pthread_t thread;
  int rc = pthread_create(&thread, NULL, ExecuteUpdate, (void *) &ts);
  
  cm.loadController("joint_state_controller");
  ROS_INFO("Loaded Joint State Controller");
  ROS_ERROR("AA");
  std::vector<std::string> controllers_state_start;
  controllers_state_start.push_back("joint_state_controller");
  std::vector<std::string> controllers_stop;
  ROS_ERROR("B");
  cm.switchController(controllers_state_start, controllers_stop, 1);
  ROS_ERROR("BB-8");
  
  
  while (robot.getPositionRead() != 1) {
    ROS_INFO("Waiting for first joint state message read");
  }
  ROS_ERROR("C");
  
  cm.loadController("simple_controller");
  ROS_INFO("Loaded Controller");
  ROS_ERROR("D");
  
  std::vector<std::string> controllers_start;
  controllers_start.push_back("simple_controller");
  ROS_ERROR("E");
  
  
  
  cm.switchController(controllers_start, controllers_stop, 1);
  ROS_ERROR("F");
  //ROS_INFO("Here2");
  while (ros::ok())
  {
    //ROS_INFO("running in parallel");
    loop_rate2.sleep();
  }
  

}
