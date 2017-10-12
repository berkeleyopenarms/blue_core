#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "koko_hardware_drivers/BLDCControllerClient.h"
#include <vector>
#include <string>
#include <sstream>
#include "math.h"
#include "time.h"

const unsigned int ENCODER_ANGLE_PERIOD = 1 << 14;
const double MAX_CURRENT = 2.8;
const unsigned int CONTROL_LOOP_FREQ = 1000;
const unsigned int BAUD_RATE = 1000000;

std::map<uint8_t, float> g_command_queue;

class SetCommand {
  private:
    uint8_t id_;
  public:
    void callback(const std_msgs::Float64::ConstPtr& msg) {
      double effort_raw = msg->data;
      double effort = effort_raw;
      if (effort > MAX_CURRENT) {
        effort = MAX_CURRENT;
      } else if (effort < -MAX_CURRENT) {
        effort = -MAX_CURRENT;
      }
      g_command_queue[id_] = effort * 10;
      // print "I heard " + effort
    }
    SetCommand(uint8_t id) {
      id_ = id;
    }
};

void initMaps(std::map<uint8_t, std::string>& joint_mapping,
    std::map<uint8_t, uint16_t>& angle_mapping) {
  joint_mapping[1] = "left_motor";
  joint_mapping[2] = "right_motor";
  joint_mapping[3] = "right_motor2";
  joint_mapping[4] = "left_motor2";
  angle_mapping[1] = 10356;
  angle_mapping[2] = 13430;
  angle_mapping[3] = 12164;
  angle_mapping[4] = 8132;
}

double getEncoderAngleRadians(BLDCControllerClient& device, uint8_t id) {
  return ((double) device.getEncoder(id)) / ENCODER_ANGLE_PERIOD * 2 * M_PI;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "jointInterface", ros::init_options::AnonymousName);

  ros::NodeHandle n;
  std::map<uint8_t, std::string> joint_mapping;
  std::map<uint8_t, uint16_t> angle_mapping;
  initMaps(joint_mapping, angle_mapping);

  //TODO: make publisher and subscribers
  /* ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000); */
  /* ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback); */
  char* port = argv[1];
  BLDCControllerClient device(port, BAUD_RATE, serial::Timeout::simpleTimeout(1));

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
    SetCommand cmd(it->first);
    subscribers[it->first] = n.subscribe("/DOF/" + it->second + "_Cmd", 1, &SetCommand::callback, &cmd);
  }
  std::map<uint8_t, double> angle_previous_mod;
  std::map<uint8_t, double> angle_accumulated;
  n_sleep(200);

  for (it = joint_mapping.begin(); it != joint_mapping.end(); it++) {
    angle_previous_mod[it->first] = getEncoderAngleRadians(device, it->first);
    angle_accumulated[it->first] = 0.0;
  }

  for(std::map<uint8_t, uint16_t>::iterator it2 = angle_mapping.begin(); it2 != angle_mapping.end(); it2++) {
    uint8_t* angle = (uint8_t*) &it2->second;
    device.writeRegisters(it2->first, 0x101, 1, angle, 2);
    uint8_t r1 = 1;
    device.writeRegisters(it2->first, 0x109, 1, &r1, 1);
  }

  while (ros::ok()) {
    for (it = joint_mapping.begin(); it != joint_mapping.end(); it++) {
      uint8_t id = it->first;
      std::string joint_name = it->second;
      double mod_angle = getEncoderAngleRadians(device, id);
      double delta_angle = fmod(mod_angle - angle_previous_mod[id] + M_PI, 2 * M_PI) - M_PI;
      angle_previous_mod[id] = mod_angle;
      angle_accumulated[id] += delta_angle;

      sensor_msgs::JointState joint_msg;
      joint_msg.name = std::vector<std::string>(1, joint_name);
      joint_msg.position = std::vector<double>(1, mod_angle);
      joint_msg.velocity = std::vector<double>(1, 0.0);
      joint_msg.effort = std::vector<double>(1, 0.0);
      publishers[id].publish(joint_msg);
      //print("name: " + joint_name + " position: " + str(joint_msg.position)
      
      std_msgs::Float32 curr_msg;
      curr_msg.data = (float) 0.0;
      publishers_curr[id].publish(curr_msg);
      if (g_command_queue.find(id) != g_command_queue.end()) {
        device.setDuty(id, g_command_queue[id]);
      }
    }

    //ros::spinOnce();

    g_command_queue.clear();
    r.sleep();
  }


  return 0;
}

