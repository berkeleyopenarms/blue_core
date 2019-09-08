#include "blue_hardware_drivers/BLDCDriver.h"

namespace blue_hardware_drivers {

const unsigned int ENCODER_ANGLE_PERIOD = 1 << 14;
/* const double MAX_CURRENT = 2.8; */
const unsigned int CONTROL_LOOP_FREQ = 1000;
const unsigned int BAUD_RATE = 1000000;

void BLDCDriver::init(std::string port, std::vector<comm_id_t> board_ids)
{
  board_ids_ = board_ids;

  for (auto id : board_ids_) {
    // Boards should start in current control mode by default
    board_control_modes_[id] = COMM_CTRL_MODE_CURRENT;
    revolutions_[id] = 0;
  }

  device_.init(port, board_ids);

  device_.resetBuffer();

  // Put all boards into bootloader!
  int count = 0;
  while (count < 3) {
    try {
      device_.resetBoards();
    } catch (comms_error e) {
      ROS_ERROR("%s\n", e.what());
      device_.resetBuffer();
    }
    count++;
    ros::Duration(0.01).sleep();
  }

  device_.resetBuffer();

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
        if (response_id == id) {
          ros::Duration(0.01).sleep();
          device_.queueConfirmID(id);
          device_.exchange();
          success = true;
        }
      } catch (comms_error e) {
        ROS_ERROR("%s\n", e.what());
        ros::Duration(0.01).sleep();
        device_.resetBuffer();
      }

      if (success) {
        ROS_INFO("Enumerated Board %d\n", id);
      } else {
        ROS_ERROR("Could not assign board id %d, retrying...", id);
      }

    }
  }

  device_.resetBuffer();

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
        //ROS_ERROR("%s\n", e.what());
        //ROS_ERROR("Could not kick board %d out of bootloader, retrying...", id);
        ros::Duration(0.01).sleep();
        device_.resetBuffer();
      }

      if (success) {
        ROS_INFO("Board %d left bootloader.\n", id);
      } 
    }
    ros::Duration(0.01).sleep();
  }

  device_.resetBuffer();

  // Start the Motors
  engageControl();
}

BLDCDriver::BLDCDriver(){
  stop_motors_ = false;
  loop_count_ = 0;
  engaged_ = false;
  first_read_ = true;
}

void BLDCDriver::update(
    std::unordered_map<comm_id_t, float>& pos_commands,
    std::unordered_map<comm_id_t, float>& current_commands,
    blue_msgs::MotorState& motor_states) {
  // Resize MotorState message to fit our read data
  const size_t motor_count = current_commands.size();
  ROS_ASSERT(current_commands.size() == motor_count);
  ROS_ASSERT(pos_commands.size() == motor_count);
  // TODO add extra motor command to the motor state information
  motor_states.command.resize(motor_count);

  if (engaged_) {
    if (!stop_motors_) {
      // Send next motor current command
      for (int i = 0; i < board_ids_.size(); i++) {
        comm_id_t id = board_ids_[i];
        switch(board_control_modes_[id]){
          case COMM_CTRL_MODE_CURRENT:
            device_.queueSetCommandAndGetState(id, current_commands[id]);
            break;
          case COMM_CTRL_MODE_POS_FF:
            device_.queueSetPosCommandAndGetState(id, pos_commands[id], current_commands[id]);
            break;
          default:
            ROS_FATAL("Control mode not supported");
            ROS_BREAK();
        }

        // TODO add extra motor command to the motor state information
        motor_states.command[i] = current_commands[id];
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

  // First check if any motors have had a watchdog reset (should happen very infrequently)
  // TODO: This will not work if a motor is on the border between a full revolution when 
  //       a reset occurs and the motor rolls back. The time window for a reset/reload 
  //       should be around 15ms so not very likely but still possible.
  bool reset = false;
  for (auto id : board_ids_) {
    if (device_.checkWDGRST(id)) {
      ROS_WARN("Watchdog reset detected on board %d, reloading state.", id);
      reloadMotor(id);
      reset = true;
    }
  }

  // Only use the last readings for an update if none of the watchdog timers were tripped.
  if (not reset) {
    updateState(motor_states);
  }
}

void BLDCDriver::updateState(blue_msgs::MotorState& motor_states) {
  // Should only be called after the new board states are updated

  // Resize MotorState message to fit our read data
  size_t motor_count = board_ids_.size();
  motor_states.position.resize(motor_count);
  motor_states.velocity.resize(motor_count);
  motor_states.direct_current.resize(motor_count);
  motor_states.quadrature_current.resize(motor_count);
  motor_states.supply_voltage.resize(motor_count);
  motor_states.temperature.resize(motor_count);
  motor_states.accel_x.resize(motor_count);
  motor_states.accel_y.resize(motor_count);
  motor_states.accel_z.resize(motor_count);

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

    revolutions_[id] = std::round(motor_states.position[i] / (2 * M_PI));

    if (motor_states.temperature[i] > MAX_TEMP_SHUTOFF) {
      ROS_ERROR("Motor %d is too hot! Shutting off system: %fC", id, motor_states.temperature[i]);
      stop_motors_ = true;
    } else if (motor_states.temperature[i] > MAX_TEMP_WARNING) {
      ROS_WARN_THROTTLE(1, "Motor %d is warm, currently at %fC", id, motor_states.temperature[i]);
    }
  }

  first_read_ = false;
  loop_count_++;
}

void BLDCDriver::reloadMotor(comm_id_t id) {
  setControlMode(id, board_control_modes_[id]);

  bool success = false;
  // Update the number of revs!
  while (!success && ros::ok()) {
    try {
      device_.queueSetRevolutions(id, revolutions_[id]);
      device_.exchange();
      success = true;
    }
    catch (comms_error e) {
      ROS_ERROR("%s\n", e.what());
      ROS_ERROR("Could not set rotations on board %d, retrying...", id);
      ros::Duration(0.01).sleep();
      device_.resetBuffer();
    }
  }

  success = false;
  // Clear the watchdog flag on the board
  while (device_.checkWDGRST(id) && ros::ok()) {
    try {
      device_.queueClearWDGRST(id);
      device_.exchange();
    }
    catch (comms_error e) {
      ROS_ERROR("%s\n", e.what());
      ROS_ERROR("Could not clear WDGRST flag on board %d, retrying...", id);
      ros::Duration(0.01).sleep();
      device_.resetBuffer();
    }
  }
}

void BLDCDriver::disengageControl() {
  engaged_ = false;
  for (auto id : board_ids_) {
    setControlMode(id, COMM_CTRL_MODE_RAW_PWM);

    bool success = false;
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
        ros::Duration(0.01).sleep();
        device_.resetBuffer();
      }
    }
  }
}

void BLDCDriver::engageControl() {
  for (auto id : board_ids_) {
    setControlMode(id, board_control_modes_[id]);

    bool success = false;
    while (!success && ros::ok()) {
      // Initialize the motor
      try {
        device_.queueSetTimeout(id, 300);
        device_.exchange();
        success = true;
      }
      catch (comms_error e) {
        ROS_ERROR("%s\n", e.what());
        ROS_ERROR("Could not set timeout on motor %d, retrying...", id);
        ros::Duration(0.01).sleep();
        device_.resetBuffer();
      }
    }
  }

  engaged_ = true;
}

void BLDCDriver::setControlMode(comm_id_t id, comm_ctrl_mode_t control_mode){
  device_.clearQueue();
  bool success = false;
  while (!success && ros::ok()) {
    try {
      device_.queueSetControlMode(id, control_mode);
      device_.exchange();
      board_control_modes_[id] = control_mode;
      success = true;
    } catch (comms_error e) {
      ROS_ERROR("%s\n", e.what());
      ROS_ERROR("Could not engage board %d, retrying...", id);
      ros::Duration(0.01).sleep();
      device_.resetBuffer();
    }
  }
}

} // namespace blue_hardware_drivers
