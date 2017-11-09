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

std::map<uint8_t, float> g_command_queue;
ros::Time last_time;
int filter_length = 10;

ros::Time get_time() {
  return ros::Time::now();
} 

float get_period() {
  ros::Time current_time = ros::Time::now();
  ros::Duration period = current_time - last_time; 
  last_time = current_time;
  return period.toSec(); 
}

class SetCommand {
  private:
    uint8_t id_;
  public:
    void callback(const std_msgs::Float64::ConstPtr& msg);
    SetCommand(uint8_t id);
};

void SetCommand::callback(const std_msgs::Float64::ConstPtr& msg) {
  double effort_raw = msg->data;
  ROS_ERROR("%d", id_);
  ROS_ERROR("%f", effort_raw);
  g_command_queue[id_] = effort_raw;
}

SetCommand::SetCommand(uint8_t id) : id_(id) {}

void initMaps(std::map<uint8_t, std::string>& joint_mapping,
    std::map<uint8_t, uint16_t>& angle_mapping,
    std::map<uint8_t, uint8_t>& invert_mapping,
    std::map<uint8_t, uint8_t>& erevs_mapping) {
    joint_mapping[15] = "base_roll_motor";
    joint_mapping[11] = "right_motor1";
    joint_mapping[12] = "left_motor1";
    joint_mapping[14] = "right_motor2";
    joint_mapping[16] = "left_motor2";

    angle_mapping[15] = 13002;
    angle_mapping[11] = 2164;
    angle_mapping[12] = 1200;
    angle_mapping[14] = 4484;
    angle_mapping[16] = 2373;

    invert_mapping[15] = 0;
    invert_mapping[11] = 0;
    invert_mapping[12] = 0;
    invert_mapping[14] = 0;
    invert_mapping[16] = 0;

    erevs_mapping[15] = 14;
    erevs_mapping[11] = 14;
    erevs_mapping[12] = 14;
    erevs_mapping[14] = 14;
    erevs_mapping[16] = 14;

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
  initMaps(joint_mapping, angle_mapping, invert_mapping, erevs_mapping);

  char* port = argv[1];
  BLDCControllerClient device(port, BAUD_RATE, serial::Timeout::simpleTimeout(10));

  ros::Rate r(CONTROL_LOOP_FREQ);

  std::map<uint8_t, std::string>::iterator it;
  for (it = joint_mapping.begin(); it != joint_mapping.end(); it++) {
    device.leaveBootloader(it->first, 0);
    device.flushSerial();
  }
  n_sleep(500);
  std::map<uint8_t, ros::Publisher> publishers;
  std::map<uint8_t, ros::Publisher> publishers_curr;
  std::map<uint8_t, ros::Subscriber> subscribers;
  for (it = joint_mapping.begin(); it != joint_mapping.end(); it++) {
    publishers[it->first] = n.advertise<sensor_msgs::JointState>("/DOF/" + it->second + "_State", 10);
    publishers_curr[it->first] = n.advertise<std_msgs::Float32>("/DOF/" + it->second + "_Current", 10);
    SetCommand *cmd = new SetCommand(it->first);
    subscribers[it->first] = n.subscribe("/DOF/" + it->second + "_Cmd", 1, &SetCommand::callback, cmd);
  }
  std::map<uint8_t, double> angle_zero;
  std::map<uint8_t, double> past_angle;
  n_sleep(200);

  for (it = joint_mapping.begin(); it != joint_mapping.end(); it++) {
    angle_zero[it->first] = getEncoderAngleRadians(device, it->first);
    velocity_history_mapping[it->first].resize(filter_length);
    past_angle[it->first] = angle_zero[it->first];
  }

  for(std::map<uint8_t, uint16_t>::iterator it2 = angle_mapping.begin(); it2 != angle_mapping.end(); it2++) {
    uint8_t* angle = (uint8_t*) &it2->second;
    device.writeRegisters(it2->first, 0x101, 1, angle, 2);

    uint8_t r = 0;
    device.writeRegisters(it2->first, 0x102, 1, &r, 1);
    device.writeRegisters(it2->first, 0x109, 1, &invert_mapping[it2->first], 1);
    device.writeRegisters(it2->first, 0x10A, 1, &erevs_mapping[it2->first], 1);
  }

  last_time = get_time(); //
  float dt = 0;
  int counter = 0;
  while (ros::ok()) {
    ros::spinOnce();
    dt = get_period();
    for (it = joint_mapping.begin(); it != joint_mapping.end(); it++) {
      uint8_t id = it->first;
      std::vector<double> v = velocity_history_mapping[id];

      std::string joint_name = it->second;
      double curr_angle = getEncoderAngleRadians(device, id) - angle_zero[id];

      v[counter] = (curr_angle - past_angle[id])/dt;
      past_angle[id] = curr_angle;

      sensor_msgs::JointState joint_msg;
      joint_msg.name = std::vector<std::string>(1, joint_name);
      joint_msg.position = std::vector<double>(1, curr_angle);

      double sum = std::accumulate(v.begin(), v.end(), 0.0);
      double avg_vel = sum / v.size();
      joint_msg.velocity = std::vector<double>(1, avg_vel);

      joint_msg.effort = std::vector<double>(1, 0.0);
      publishers[id].publish(joint_msg);

      std_msgs::Float32 curr_msg;
      curr_msg.data = (float) 0.0;
      publishers_curr[id].publish(curr_msg);
      // ROS_ERROR("TRy to set duty");
      if (g_command_queue.find(id) != g_command_queue.end()) {
        // std::cerr << "setting duty to " << g_command_queue[id];
        device.setDuty(id, g_command_queue[id]);
        // ROS_ERROR("Set Duty");
      }
    }
    counter ++;
    counter = counter % filter_length;
    g_command_queue.clear();
    r.sleep();
  }


  return 0;
}
