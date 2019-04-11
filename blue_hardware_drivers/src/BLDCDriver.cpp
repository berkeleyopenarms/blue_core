#include "blue_hardware_drivers/BLDCDriver.h"

namespace blue_hardware_drivers {

const unsigned int ENCODER_ANGLE_PERIOD = 1 << 14;
/* const double MAX_CURRENT = 2.8; */
const unsigned int CONTROL_LOOP_FREQ = 1000;
const unsigned int BAUD_RATE = 1000000;

void BLDCDriver::init(std::string port, std::vector<uint8_t> board_ids)
{
  board_ids_ = board_ids;

  device_.init(port, board_ids);

  // Assign boards IDs!
  bool success;
  for (auto id : board_ids_) {
    success = false;
    while (!success && ros::ok()) {
      try {
        device_.queueEnumerate(id);
        device_.exchange();
        comm_id_t response_id = 0;
        device_.getEnumerateResponse(id, &response_id);
        if (response_id == id)
          success = true;
      } catch (comms_error e) {
        ROS_ERROR("%s\n", e.what());
        ROS_ERROR("Could not assign board id %d, retrying...", id);
        ros::Duration(0.2).sleep();
        continue;
      }
      ros::Duration(1).sleep();
    }
  }


  // Kick all board_ids_ out of bootloader!
  for (auto id : board_ids_) {
    success = false;
    while (!success && ros::ok()) {
      try {
        // 0 jumps to default firmware address
        device_.queueLeaveBootloader(id, 0);
        device_.exchange();
        success = true;
      } catch (comms_error e) {
        ROS_ERROR("%s\n", e.what());
        ROS_ERROR("Could not kick board %d out of bootloader, retrying...", id);
        ros::Duration(0.2).sleep();
      }
    }
  }

  for (auto id : board_ids_) {
    success = false; // set to false to initialize board_ids_ (doing this because some test board_ids_ are not calibrated)
    while (!success && ros::ok()) {
      // Initialize the motor
      try {
        device_.initMotor(id);
        success = true;
      }
      catch (comms_error e) {
        ROS_ERROR("%s\n", e.what());
        ROS_ERROR("Could not initialize motor %d, retrying...", id);
        ros::Duration(0.2).sleep();
      }
    }
    // Set motor timeout to 1 second
    success = false;
    while (!success && ros::ok()) {
      // Initialize the motor
      try {
        device_.queueSetTimeout(id, 1000);
        device_.exchange();
        success = true;
      }
      catch (comms_error e) {
        ROS_ERROR("%s\n", e.what());
        ROS_ERROR("Could not set timeout on motor %d, retrying...", id);
        ros::Duration(0.2).sleep();
      }
    }
    ROS_DEBUG("Initialized board: %d", id);
  }

  for (auto id : board_ids_) {
    success = false;
    while (!success && ros::ok()) {
      try {
        device_.queueGetState(id);
        device_.exchange();
        success = true;
      } catch (comms_error e) {
        ROS_ERROR("%s\n", e.what());
        ROS_ERROR("Could not get initial state of board %d, retrying...", id);
        ros::Duration(0.2).sleep();
      }
    }
  }

  engaged_ = true;
}

BLDCDriver::BLDCDriver(){
  stop_motors_ = false;
  loop_count_ = 0;
  engaged_ = false;
}

void BLDCDriver::update(std::unordered_map<uint8_t, float>& commands, blue_msgs::MotorState& motor_states) {

  // Resize MotorState message to fit our read data
  int motor_count = commands.size();
  motor_states.command.resize(motor_count);
  motor_states.position.resize(motor_count);
  motor_states.velocity.resize(motor_count);
  motor_states.direct_current.resize(motor_count);
  motor_states.quadrature_current.resize(motor_count);
  motor_states.temperature.resize(motor_count);
  motor_states.supply_voltage.resize(motor_count);
  motor_states.accel_x.resize(motor_count);
  motor_states.accel_y.resize(motor_count);
  motor_states.accel_z.resize(motor_count);

  if (engaged_) {
    if (!stop_motors_) {
      // Send next motor current command
      for (int i = 0; i < board_ids_.size(); i++) {
        comm_id_t id = board_ids_[i];
        device_.queueSetCommandAndGetState(id, commands[id]);
        motor_states.command[i] = commands[id];
      }
    } else {
      // If one of the motors is too hot, we still want to grab the state and set effort to 0
      for (int i = 0; i < board_ids_.size(); i++) {
        comm_id_t id = board_ids_[i];
        device_.queueSetCommandAndGetState(id, 0.0);
        motor_states.command[i] = 0.0;
      }
    }

    // Run the communication with each board
    device_.exchange();
  }

  // Get the state of the each board
  for (int i = 0; i < board_ids_.size(); i++) {
    comm_id_t id = board_ids_[i];
    device_.resultGetState(id
        , &motor_states.position[i]
        , &motor_states.velocity[i]
        , &motor_states.direct_current[i]
        , &motor_states.quadrature_current[i]
        , &motor_states.supply_voltage[i]
        , &motor_states.temperature[i]
        , &motor_states.accel_x[i]
        , &motor_states.accel_y[i]
        , &motor_states.accel_z[i]
        );

    if (motor_states.temperature[i] > MAX_TEMP_SHUTOFF) {
      stop_motors_ = true;
      ROS_ERROR_THROTTLE(1, "Motor %d is too hot! Shutting off system.", id);
    } else if (motor_states.temperature[i] > MAX_TEMP_WARNING) {
      ROS_WARN_THROTTLE(1, "Motor %d is warm, currently at %fC", id, motor_states.temperature[i]);
    }
  }

  loop_count_++;
}

void BLDCDriver::disengageControl() {
  engaged_ = false;
  for (auto id : board_ids_) {
    bool success = false;
    while (!success && ros::ok()) {
      try {
        device_.queueSetControlMode(id, COMM_CTRL_MODE_RAW_PWM);
        device_.exchange();
        success = true;
      } catch (comms_error e) {
        ROS_ERROR("%s\n", e.what());
        ROS_ERROR("Could not disengage board %d, retrying...", id);
        ros::Duration(0.2).sleep();
      }
    }
    success = false;
    while (!success && ros::ok()) {
      // Initialize the motor
      try {
        device_.queueSetTimeout(id, 0);
        device_.exchange();
        success = true;
      }
      catch (comms_error e) {
        ROS_ERROR("%s\n", e.what());
        ROS_ERROR("Could not set timeout on motor %d, retrying...", id);
        ros::Duration(0.2).sleep();
      }
    }
  }
}

void BLDCDriver::engageControl() {
  for (auto id : board_ids_) {
    bool success = false;
    while (!success && ros::ok()) {
      try {
        device_.queueSetControlMode(id, COMM_CTRL_MODE_CURRENT);
        device_.exchange();
        success = true;
      } catch (comms_error e) {
        ROS_ERROR("%s\n", e.what());
        ROS_ERROR("Could not engage board %d, retrying...", id);
        ros::Duration(0.2).sleep();
      }
    }
    success = false;
    while (!success && ros::ok()) {
      // Initialize the motor
      try {
        device_.queueSetTimeout(id, 1000);
        device_.exchange();
        success = true;
      }
      catch (comms_error e) {
        ROS_ERROR("%s\n", e.what());
        ROS_ERROR("Could not set timeout on motor %d, retrying...", id);
        ros::Duration(0.2).sleep();
      }
    }
  }

  engaged_ = true;
}

} // namespace blue_hardware_drivers
