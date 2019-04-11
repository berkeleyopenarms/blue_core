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

using namespace blue_hardware_drivers;

const unsigned int ENCODER_ANGLE_PERIOD = 1 << 14;
/* const double MAX_CURRENT = 2.8; */
const unsigned int CONTROL_LOOP_FREQ = 1000;
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
  ROS_ERROR("h");

  ros::NodeHandle n;
  std::vector<comm_id_t> board_list;
  ROS_ERROR("1");
  std::map<comm_id_t, std::vector<double> > velocity_history_mapping;
  ROS_ERROR("2");
  board_list.push_back(40); // BASE
  ROS_ERROR("3");
  board_list.push_back(43);
  ROS_ERROR("4");
  // board_list.push_back(38);
  // board_list.push_back(16);
  // board_list.push_back(33);
  // board_list.push_back(41);
  // board_list.push_back(42);
  // board_list.push_back(52);

  char* port = argv[1];
  BLDCControllerClient device;
  try {
    device.init(port, board_list);
  } catch (std::exception& e) { ROS_ERROR("Please Provide USB\n%s\n", e.what()); }

  // Assign boards IDs!
  bool success;
  for (auto id : board_list) {
    success = false;
    while (!success && ros::ok()) {
      try {
        device.queueEnumerate(id);
        device.exchange();
        success = true;
      } catch (comms_error e) {
        ROS_ERROR("%s\n", e.what());
        ROS_ERROR("Could not assign board id %d, retrying...", id);
        ros::Duration(0.2).sleep();
      }
      ros::Duration(0.2).sleep();
    }
  }

  // Kick all boards out of bootloader!
  for (auto id : board_list) {
    success = false;
    while (!success) {
      try {
        device.queueLeaveBootloader(id, 0);
        device.exchange();
        success = true;
      } catch (comms_error e) {
        ROS_ERROR("%s\n", e.what());
        ROS_ERROR("Could not kick board %d out of bootloader, retrying...", id);
      }
    }
  }

  for (auto id : board_list) {
    success = false; // set to false to initialize boards (doing this because some test boards are not calibrated)
    while (!success) {
      // Initialize the motor
      try { device.initMotor(id); }
      catch (comms_error e) {
        ROS_ERROR("%s\n", e.what());
        ROS_ERROR("Could not initialize motor %d, retrying...", id);
      }
      success = true;
    }
    // Set motor timeout to 1 second
    device.queueSetTimeout(id, 1000);
    device.exchange();
    ROS_DEBUG("Initialized board: %d", id);

  }

  last_time = get_time();
  float dt = 0;
  int counter = 0;
  ros::Rate r(CONTROL_LOOP_FREQ);

  int num_packets_per_log = 100;
  int errors = 0;
  int num_packets = 0;
  while (ros::ok()) {
    for (int i = 0; i < num_packets_per_log; i ++) { // "low pass filter"
      for (auto id : board_list) {
        device.queueSetCommandAndGetRotorPosition(id, 0);
      }
      try { device.exchange(); }
      catch(comms_error e) {
        ROS_ERROR("%s\n", e.what());
        device.clearQueue();
        errors++;
        continue;
      }
      float pos = 0;
      for (auto id : board_list) {
        device.resultGetRotorPosition(id, &pos);
        //std::cout << "Position: " << pos << std::endl;
      }
    }
    num_packets += num_packets_per_log;
    dt = get_period() / (float) num_packets_per_log;
    ROS_INFO("comm time: dt: %f, freq: %f", dt, 1.0 / dt);
  }
  r.sleep();


  return 0;
}

