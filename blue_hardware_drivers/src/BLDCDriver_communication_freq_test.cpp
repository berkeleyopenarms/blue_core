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
const unsigned int CONTROL_LOOP_FREQ = 5000;
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

void initMapsTest(std::map<uint8_t, std::string>& joint_mapping,
    std::map<uint8_t, uint16_t>& angle_mapping,
    std::map<uint8_t, uint8_t>& invert_mapping,
    std::map<uint8_t, uint8_t>& erevs_mapping) {
  joint_mapping[52] = "base_roll_motor";
  angle_mapping[52] = 12164;
  invert_mapping[52] = true;
  erevs_mapping[52] = 14;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "jointInterface", ros::init_options::AnonymousName);

  ros::NodeHandle n;
  std::map<uint8_t, std::string> joint_mapping;
  std::map<uint8_t, uint16_t> angle_mapping;
  std::map<uint8_t, std::vector<double> > velocity_history_mapping;
  std::map<uint8_t, uint8_t> invert_mapping;
  std::map<uint8_t, uint8_t> erevs_mapping;
  std::map<uint8_t, float> angle_zero;
  // initMaps(joint_mapping, angle_mapping, invert_mapping, erevs_mapping);
  initMapsTest(joint_mapping, angle_mapping, invert_mapping, erevs_mapping);

  char* port = argv[1];
  BLDCControllerClient device;
  try {
    device.init(port);
  } catch (std::exception& e) { ROS_ERROR(e.what()); }

  ros::Rate r(CONTROL_LOOP_FREQ);

  bool result;
  std::map<uint8_t, std::string>::iterator it;
  for (it = joint_mapping.begin(); it != joint_mapping.end(); it++) {
    ROS_ERROR("c21");
    device.leaveBootloader(it->first, 0, &result);
  }
  device.exchange();

  ROS_ERROR("c3");
  ros::Duration(0.5).sleep();

  for (it = joint_mapping.begin(); it != joint_mapping.end(); it++) {
    ROS_ERROR("c41, id: %d", it->first);
    device.getRotorPosition(it->first, &(angle_zero[it->first]));
    ROS_ERROR("c42");
  }

  device.exchange();
  ROS_ERROR(std::to_string(angle_zero[52]).c_str());

  for(std::map<uint8_t, uint16_t>::iterator it2 = angle_mapping.begin(); it2 != angle_mapping.end(); it2++) {
    // uint8_t* angle = (uint8_t*) &it2->second;
    device.setZeroAngle(it2->first, it2->second, nullptr);
    device.setERevsPerMRev(it2->first, erevs_mapping[it2->first], nullptr);
    device.setInvertPhases(it2->first, invert_mapping[it2->first], nullptr);
    device.setCurrentControlMode(it2->first, nullptr);
  }
  ROS_ERROR("c6");

  last_time = get_time(); //
  float dt = 0;
  int counter = 0;
  while (ros::ok()) {
    // dt = get_period();
    // ROS_ERROR("slept: %f", dt);
    for (int i = 0; i < 10; i ++) { // "low pass filter"
      for (it = joint_mapping.begin(); it != joint_mapping.end(); it++) {
        uint8_t id = it->first;
        try {
          device.setCommandAndGetRotorPosition(id, 0.2, nullptr);
          // device.getRotorPosition(id);
        } catch(...) {
          ROS_ERROR("reg read failed");
        }
      }
    }
    dt = get_period() / 10.0;
    ROS_ERROR("comm time: dt: %f, freq: %f", dt, 1.0 / dt);
  }
  r.sleep();


  return 0;
}
