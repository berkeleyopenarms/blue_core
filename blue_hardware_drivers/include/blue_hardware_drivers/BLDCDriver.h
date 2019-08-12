#ifndef BLDC_DRIVER_H
#define BLDC_DRIVER_H

#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <unordered_map>
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
    void init(std::string port, std::vector<comm_id_t> board_ids);
    void update(
        std::unordered_map<comm_id_t, float>& pos_commands,
        std::unordered_map<comm_id_t, float>& feed_forward_commands,
        blue_msgs::MotorState& motor_states);
    void engageControl();
    void disengageControl();
    bool setControlMode(int id, comm_ctrl_mode_t control_mode);

    BLDCDriver();

  private:
    std::vector<comm_id_t> board_ids_;
    std::unordered_map<comm_id_t, comm_ctrl_mode_t> board_control_modes;
    std::unordered_map<comm_id_t, signed int> revolutions_;
    std::unordered_map<comm_id_t, signed int> angle_;

    serial::Serial ser_;
    BLDCControllerClient device_;

    unsigned int loop_count_;
    bool stop_motors_;
    bool engaged_;
    bool first_read_;
    void _update_state(int motor_count, blue_msgs::MotorState& motor_states);
};

} // namespace blue_hardware_drivers

#endif // BLDC_DRIVER
