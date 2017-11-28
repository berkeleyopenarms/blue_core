#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "koko_hardware_drivers/BLDCControllerClient.h"
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


void initMaps(std::map<uint8_t, std::string>& joint_mapping,
    std::map<uint8_t, uint16_t>& angle_mapping,
    std::map<uint8_t, uint8_t>& invert_mapping,
    std::map<uint8_t, uint8_t>& erevs_mapping) {
    joint_mapping[15] = "base_roll_motor";
    joint_mapping[11] = "right_motor1";
    joint_mapping[12] = "left_motor1";
    joint_mapping[14] = "right_motor2";
    joint_mapping[16] = "left_motor2";
    joint_mapping[21] = "right_motor3";
    joint_mapping[19] = "left_motor3";
    //joint_mapping[17] = "left_motor3";
    //joint_mapping[13] = "base_roll_motor";

    angle_mapping[15] = 13002;
    angle_mapping[11] = 2164;
    angle_mapping[12] = 1200;
    angle_mapping[14] = 4484;
    angle_mapping[16] = 2373;
    angle_mapping[21] = 5899;
    angle_mapping[19] = 2668;
    //angle_mapping[17] = 10720;
    //angle_mapping[13] = 11839;

    invert_mapping[15] = 0;
    invert_mapping[11] = 0;
    invert_mapping[12] = 0;
    invert_mapping[14] = 0;
    invert_mapping[16] = 0;
    invert_mapping[21] = 0;
    invert_mapping[19] = 0;
    //invert_mapping[17] = 1;
    //invert_mapping[13] = 1;

    erevs_mapping[15] = 14;
    erevs_mapping[11] = 14;
    erevs_mapping[12] = 14;
    erevs_mapping[14] = 14;
    erevs_mapping[16] = 14;
    erevs_mapping[21] = 21;
    erevs_mapping[19] = 21;
    //erevs_mapping[17] = 14;
    //erevs_mapping[13] = 14;

  /* joint_mapping[1] = "left_motor"; */
  /* joint_mapping[2] = "right_motor"; */
  /* joint_mapping[3] = "right_motor2"; */
  /* joint_mapping[4] = "left_motor2"; */
//  joint_mapping[10] = "test_motor";
//  angle_mapping[10] = 7568;
  /* std::map<std::string, int> angles; */
  /* ros::param::get("/koko_hardware_drivers/calibrations", angles); */
  /* if (angles.size() < 5) { */
  /*   std::cerr << "did not get correct map, size " << angles.size() << "\n"; */
  /* } */

  /* for(std::map<std::string, int>::iterator it = angles.begin(); it != angles.end(); it++) { */
  /*   uint16_t angle = (uint16_t) it->second; */
  /*   uint8_t id = atoi(it->first.c_str()); */
  /*   if (id == 0) { */
  /*     return; */
  /*   } */
  /*   angle_mapping[id] = angle; */
  /* } */
}

double getEncoderAngleRadians(BLDCControllerClient& device, uint8_t id) {
  double x = ((double) device.getEncoderRadians(id));
  return x;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "jointInterface", ros::init_options::AnonymousName);

  ros::NodeHandle n;
  std::map<uint8_t, std::string> joint_mapping;
  std::map<uint8_t, uint16_t> angle_mapping;
  std::map<uint8_t, std::vector<double> > velocity_history_mapping;
  std::map<uint8_t, uint8_t> invert_mapping;
  std::map<uint8_t, uint8_t> erevs_mapping;
  std::map<uint8_t, double> angle_zero;
  initMaps(joint_mapping, angle_mapping, invert_mapping, erevs_mapping);

  char* port = argv[1];
  ROS_ERROR("c1");
  BLDCControllerClient device(port);
  ROS_ERROR("c2");

  ros::Rate r(CONTROL_LOOP_FREQ);

  std::map<uint8_t, std::string>::iterator it;
  for (it = joint_mapping.begin(); it != joint_mapping.end(); it++) {
    ROS_ERROR("c21");
    device.leaveBootloader(it->first, 0);
  }
  ROS_ERROR("c3");
  n_sleep(500);

  for (it = joint_mapping.begin(); it != joint_mapping.end(); it++) {
    ROS_ERROR("c41, id: %d", it->first);
    angle_zero[it->first] = getEncoderAngleRadians(device, it->first);
    ROS_ERROR("c42");
  }

  for(std::map<uint8_t, uint16_t>::iterator it2 = angle_mapping.begin(); it2 != angle_mapping.end(); it2++) {
    uint8_t* angle = (uint8_t*) &it2->second;
    // ROS_ERROR("c50");
    // device.writeRegisters(it2->first, 0x101, 1, angle, 2);
    // ROS_ERROR("c51");
    //
    // uint8_t r = 0;
    // device.writeRegisters(it2->first, 0x102, 1, &r, 1);
    // ROS_ERROR("c52");
    // device.writeRegisters(it2->first, 0x109, 1, &invert_mapping[it2->first], 1);
    // ROS_ERROR("c53");
    // device.writeRegisters(it2->first, 0x10A, 1, &erevs_mapping[it2->first], 1);
    // ROS_ERROR("c54");
  }
  ROS_ERROR("c6");

  last_time = get_time(); //
  float dt = 0;
  int counter = 0;
  while (ros::ok()) {
    dt = get_period();
    ROS_ERROR("slept: %f", dt);
    for (it = joint_mapping.begin(); it != joint_mapping.end(); it++) {
      uint8_t id = it->first;

      std::string joint_name = it->second;
      double curr_angle = getEncoderAngleRadians(device, id);

    }
    dt = get_period();
    ROS_ERROR("comm time: dt: %f, freq: %f", dt, 1.0 / dt);
    r.sleep();
  }


  return 0;
}
