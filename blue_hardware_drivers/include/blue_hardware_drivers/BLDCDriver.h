#ifndef BLDC_DRIVER_H
#define BLDC_DRIVER_H

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

#include "blue_msgs/MotorState.h"
#include "blue_hardware_drivers/BLDCControllerClient.h"

namespace blue_hardware_drivers {

constexpr int MAX_TEMP_WARNING = 60;
constexpr int MAX_TEMP_SHUTOFF = 70;

class BLDCDriver {
  public:
    void init(std::string port, std::vector<uint8_t> board_ids);
    void update(std::map<uint8_t, float>& commands, blue_msgs::MotorState& motor_states);
    void engageControl();
    void disengageControl();
    BLDCDriver();

  private:
    std::vector<comm_id_t> board_ids_;

    serial::Serial ser_;
    BLDCControllerClient device_;

    unsigned int loop_count_;
    bool stop_motors_;
    bool engaged_;
};

} // namespace blue_hardware_drivers

#endif // BLDC_DRIVER
