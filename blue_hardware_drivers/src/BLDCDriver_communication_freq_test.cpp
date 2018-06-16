#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "blue_hardware_drivers/BLDCControllerClient.h"
#include <vector>
#include <string>
#include "math.h"
#include <numeric>
#include "time.h"

const unsigned int ENCODER_ANGLE_PERIOD = 1 << 14;
/* const double MAX_CURRENT = 2.8; */
const unsigned int CONTROL_LOOP_FREQ = 5000000;
const unsigned int BAUD_RATE = 1000000;

ros::Time last_time;

ros::Time get_time() {
  return ros::Time::now();
}

float get_period() {
  ros::Time current_time = ros::Time::now();
  ros::Duration period = current_time - last_time;
  last_time = current_time;
  return period.toSec();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "comms", ros::init_options::AnonymousName);

  ros::NodeHandle n;
  std::vector<comm_id_t> board_list;
  std::map<comm_id_t, std::vector<double> > velocity_history_mapping;
  board_list.push_back(45);

  char* port = argv[1];
  BLDCControllerClient device;
  try {
    device.init(port, board_list);
  } catch (std::exception& e) { ROS_ERROR(e.what()); }

  // Kick all boards out of bootloader!
  for (auto id : board_list) {
    device.queueLeaveBootloader(id, 0);
  }
  device.exchange();

  ros::Duration(1).sleep();

  for (auto id : board_list) {
    // Initialize the motor
    device.initMotor(id);
    // Set motor timeout to 1 second
    device.queueSetTimeout(id, 1000);
    ROS_DEBUG("Initialized board: %d", id);
  }

  device.exchange();

  last_time = get_time(); 
  float dt = 0;
  int counter = 0;
  ros::Rate r(CONTROL_LOOP_FREQ);

  while (1) {
    // dt = get_period();
    // ROS_ERROR("slept: %f", dt);
    for (int i = 0; i < 10; i ++) { // "low pass filter"
      for (auto id : board_list) {
        float pos = 0;
        try {
          device.queueSetCommandAndGetRotorPosition(id, 0.3);

          device.exchange();

          device.resultGetRotorPosition(id, &pos);
          // device.getRotorPosition(id);
        } catch(comms_error e) {
          ROS_ERROR(e.what());
        }
        //std::cout << pos << std::endl;
      }
    }
    dt = get_period() / 10.0;
    ROS_ERROR("comm time: dt: %f, freq: %f", dt, 1.0 / dt);
  }
  r.sleep();


  return 0;
}
