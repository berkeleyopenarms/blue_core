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
#include "time.h"

class BLDCDriver {
  private:
    serial::Serial ser;

    std::vector<uint8_t> angle_id_mapping;
    std::map<uint8_t, std::string> motor_mapping;
    std::map<uint8_t, uint16_t> angle_mapping;
    std::map<uint8_t, uint8_t> invert_mapping;
    std::map<uint8_t, uint8_t> erevs_mapping;

    std::map<uint8_t, double> angle_zero;
    std::map<uint8_t, double> prev_angle;
    BLDCControllerClient device;

    std::vector<double>* pos;
    std::vector<double>* vel;
    std::vector<double>* eff;
    const std::vector<double>* cmd;
    ros::Time get_time();
    ros::Duration get_period();
    ros::Time last_time;

  public:
    void init(std::vector<double>* in_pos, std::vector<double>* in_vel, std::vector<double>* in_eff, const std::vector<double>* in_cmd);
    void read();
    void write();
    BLDCDriver();

};

