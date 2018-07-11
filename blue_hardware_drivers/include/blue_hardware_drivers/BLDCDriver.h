#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <string>
#include "math.h"
#include "time.h"

#include "blue_hardware_drivers/BLDCControllerClient.h"

constexpr int MAX_TEMP_WARNING = 60;
constexpr int MAX_TEMP_SHUTOFF = 70;

struct MotorState {
  float position;
  float velocity;
  float di;
  float qi;
  float temp;
  float voltage;
  int32_t acc_x, acc_y, acc_z;
};

class BLDCDriver {
  public:
    void init(const std::vector<comm_id_t> &boards, std::map<comm_id_t, MotorState>* states, std::string port);
    void update(std::map<comm_id_t, float>& commands);
    void engageControl();
    void disengageControl();
    BLDCDriver();
  
  private:
    std::map<comm_id_t, MotorState>* states_;
    std::map<comm_id_t, float> zero_angles_;
    std::vector<comm_id_t> boards_;

    serial::Serial ser_;
    BLDCControllerClient device_;

    unsigned int loop_count_;
    bool stop_motors_;
    bool engaged_;
};

