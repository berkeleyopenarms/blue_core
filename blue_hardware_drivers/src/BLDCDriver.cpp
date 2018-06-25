#include "blue_hardware_drivers/BLDCDriver.h"

const unsigned int ENCODER_ANGLE_PERIOD = 1 << 14;
/* const double MAX_CURRENT = 2.8; */
const unsigned int CONTROL_LOOP_FREQ = 1000;
const unsigned int BAUD_RATE = 1000000;

void BLDCDriver::init(const std::vector<comm_id_t> &boards, std::map<comm_id_t, MotorState>* states) {

  std::string port = "/dev/ttyUSB0"; // TODO: read from launch/config

  states_ = states;
  boards_ = boards;

  device_.init(port, boards);

  // Kick all boards_ out of bootloader!
  bool success;
  for (auto id : boards_) {
    success = false;
    while (!success) {
      try {
        device_.queueLeaveBootloader(id, 0);
        device_.exchange();
        success = true;
      } catch (comms_error e) {
        ROS_ERROR(e.what());
        ROS_ERROR("Could not kick board %d out of bootloader, retrying...", id);
      }
    }
  }

  for (auto id : boards_) {
    success = false; // set to false to initialize boards_ (doing this because some test boards_ are not calibrated)
    while (!success) {
      // Initialize the motor
      try { 
        device_.initMotor(id); 
        success = true;
      }
      catch (comms_error e) {
        ROS_ERROR(e.what());
        ROS_ERROR("Could not initialize motor %d, retrying...", id);
      }
    }
    // Set motor timeout to 1 second
    success = false;
    while (!success) {
      // Initialize the motor
      try { 
        device_.queueSetTimeout(id, 1000);
        device_.exchange();
        success = true;
      }
      catch (comms_error e) {
        ROS_ERROR(e.what());
        ROS_ERROR("Could not initialize motor %d, retrying...", id);
      }
    }
    ROS_DEBUG("Initialized board: %d", id);
  }

  for (auto id : boards_) {
    success = false;
    while (!success) {
      try {
        device_.queueGetState(id);
        device_.exchange();
        success = true;
      } catch (comms_error e) {
        ROS_ERROR(e.what());
        ROS_ERROR("Could not get initial state of board %d, retrying...", id);
      }
    }
  }
  // Init angle
  for (auto id : boards_) {
    device_.resultGetState(id
        , &(*states_)[id].position
        , &(*states_)[id].velocity
        , &(*states_)[id].di
        , &(*states_)[id].qi
        , &(*states_)[id].voltage
        , &(*states_)[id].temp
        , &(*states_)[id].acc_x
        , &(*states_)[id].acc_y
        , &(*states_)[id].acc_z
        );
    zero_angles_[id] = (*states)[id].position;
  }
}

BLDCDriver::BLDCDriver(){
  states_ = nullptr;
  stop_motors_ = false;
  loop_count_ = 0;
}

void BLDCDriver::update(std::map<comm_id_t, float>& commands){
 
  if (!stop_motors_) {
    // Send next motor current command
    for (auto id : boards_) {
        device_.queueSetCommandAndGetState(id, commands[id]);
    }
  } else {
    // If one of the motors is too hot, we still want to grab the state and set effort to 0
    for (auto id : boards_) {
        device_.queueSetCommandAndGetState(id, 0.0);
    }
  }

  // Run the communication with each board
  device_.exchange();
    
  // Get the state of the each board
  for (auto id : boards_) {
    device_.resultGetState(id
        , &(*states_)[id].position
        , &(*states_)[id].velocity
        , &(*states_)[id].di
        , &(*states_)[id].qi
        , &(*states_)[id].voltage
        , &(*states_)[id].temp
        , &(*states_)[id].acc_x
        , &(*states_)[id].acc_y
        , &(*states_)[id].acc_z
        );
    if ((*states_)[id].temp > MAX_TEMP_SHUTOFF) {
      stop_motors_ = true;
      ROS_ERROR("Motor %d is too hot! Shutting off system.", id);
    }
    else if ((*states_)[id].temp > MAX_TEMP_WARNING) {
      ROS_ERROR("Motor %d is warm, currently at %fC", id, (*states_)[id].temp);
    }
  }

  loop_count_++;
}

