#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "blue_hardware_drivers/BLDCControllerClient.h"
#include <vector>
#include <string>
#include "math.h"
#include <numeric>
#include "time.h"

const unsigned int ENCODER_ANGLE_PERIOD = 1 << 14;
/* const double MAX_CURRENT = 2.8; */
const unsigned int CONTROL_LOOP_FREQ = 1000;
const unsigned int BAUD_RATE = 1000000;

ros::Time last_time;

float pos = 0.0f;

ros::Time get_time() {
  return ros::Time::now();
}

float get_period() {
  ros::Time current_time = ros::Time::now();
  ros::Duration period = current_time - last_time;
  last_time = current_time;
  return period.toSec();
}

void pos_callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  pos = msg->data[0];
  // msg.data[1];
  ROS_ERROR("got callback");

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "comms", ros::init_options::AnonymousName);

  ros::NodeHandle n;
  std::vector<comm_id_t> board_list;
  std::map<comm_id_t, std::vector<double> > velocity_history_mapping;
  board_list.push_back(22); // BASE
  board_list.push_back(43);


  ros::Subscriber sub = n.subscribe("/blue_controller/joint_position_controller/command", 1000, pos_callback);


  char* port = argv[1];
  BLDCControllerClient device;
  try {
    device.init(port, board_list);
  } catch (std::exception& e) { ROS_ERROR("%s\n", e.what()); }

  // Kick all boards out of bootloader!
  bool success;
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

  // Setting board Ki and Kp values!
  float position_ki = 20,
        position_kp = 20,
        velocity_ki = 5,
        velocity_kp = 2;

  for (auto id : board_list) {
    success = false; // set to false to initialize boards (doing this because some test boards are not calibrated)
    while (!success) {
      try {
        device.queueSetPositionControllerKi(id, position_ki);
        device.exchange();
        device.queueSetPositionControllerKp(id, position_kp);
        device.exchange();
        device.queueSetVelocityControllerKi(id, velocity_ki);
        device.exchange();
        device.queueSetVelocityControllerKp(id, velocity_kp);
        device.exchange();
      }
      catch (comms_error e) {
        ROS_ERROR("%s\n", e.what());
        ROS_ERROR("Could not write kp/ki values %d, retrying...", id);
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
  try {
    for (auto id : board_list) {
      device.queueSetControlMode(id, COMM_CTRL_MODE_POSITION);
      device.exchange();
      // device.queueSetPositionAndGetRotorPosition(id, mult*pos);
      // device.exchange();
    }
  } catch(comms_error e) {
        ROS_ERROR("%s\n", e.what());
        device.clearQueue();
  }
  while (ros::ok()) {
    float posi = 0;
    for (int i = 0; i < num_packets_per_log; i ++) { // "low pass filter"
      double mult;
      try {
        mult = 1.0;
        for (auto id : board_list) {
          // device.queueSetControlMode(id, COMM_CTRL_MODE_POSITION);
          // device.exchange();
          device.queueSetPositionAndGetRotorPosition(id, mult*pos);
          device.exchange();
        }
        mult= mult * -1.0;
      }
      catch(comms_error e) {
        ROS_ERROR("%s\n", e.what());
        device.clearQueue();
        errors++;
        continue;
      }

      for (auto id : board_list) {
        device.resultGetRotorPosition(id, &posi);
        //std::cout << "Position: " << pos << std::endl;
      }
      ros::spinOnce();
    }
    num_packets += num_packets_per_log;
    dt = get_period() / (float) num_packets_per_log;
    ROS_INFO("comm time: dt: %f, freq: %f", dt, 1.0 / dt);
    ROS_INFO("rotor_pos: %f", posi);
  }
  r.sleep();


  return 0;
}
