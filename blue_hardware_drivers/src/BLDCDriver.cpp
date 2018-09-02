#include "blue_hardware_drivers/BLDCDriver.h"

const unsigned int ENCODER_ANGLE_PERIOD = 1 << 14;
/* const double MAX_CURRENT = 2.8; */
const unsigned int CONTROL_LOOP_FREQ = 1000;
const unsigned int BAUD_RATE = 1000000;

void BLDCDriver::init(const std::vector<comm_id_t> &boards, blue_msgs::MotorState* states, std::string port)
  {
  boards_ = boards;
  states_ = states;

  device_.init(port, boards);

  // Kick all boards_ out of bootloader!
  bool success;
  for (auto id : boards_) {
    success = false;
    while (!success && ros::ok()) {
      try {
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

  for (auto id : boards_) {
    success = false; // set to false to initialize boards_ (doing this because some test boards_ are not calibrated)
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

  for (auto id : boards_) {
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
  states_ = nullptr;
  stop_motors_ = false;
  loop_count_ = 0;
  engaged_ = false;
}

void BLDCDriver::update(std::map<comm_id_t, float>& commands){

  if (engaged_) {
    if (!stop_motors_) {
      // Send next motor current command
      for (int i = 0; i < boards_.size(); i++) {
        comm_id_t id = boards_[i];
        device_.queueSetCommandAndGetState(id, commands[id]);
        states_->command[i] = commands[id];
      }
    } else {
      // If one of the motors is too hot, we still want to grab the state and set effort to 0
      for (int i = 0; i < boards_.size(); i++) {
        comm_id_t id = boards_[i];
        device_.queueSetCommandAndGetState(id, 0.0);
        states_->command[i] = 0.0;
      }
    }

    // Run the communication with each board
    device_.exchange();
  }

  // Get the state of the each board
  for (int i = 0; i < boards_.size(); i++) {
    comm_id_t id = boards_[i];
    device_.resultGetState(id
        , &states_->position[i]
        , &states_->velocity[i]
        , &states_->direct_current[i]
        , &states_->quadrature_current[i]
        , &states_->supply_voltage[i]
        , &states_->temperature[i]
        , &states_->accel_x[i]
        , &states_->accel_y[i]
        , &states_->accel_z[i]
        );

    if (states_->temperature[i] > MAX_TEMP_SHUTOFF) {
      stop_motors_ = true;
      ROS_ERROR_THROTTLE(1, "Motor %d is too hot! Shutting off system.", id);
    } else if (states_->temperature[i] > MAX_TEMP_WARNING) {
      ROS_WARN_THROTTLE(1, "Motor %d is warm, currently at %fC", id, states_->temperature[i]);
    }
  }

  loop_count_++;
}

void BLDCDriver::disengageControl() {
  engaged_ = false;
  for (auto id : boards_) {
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
  for (auto id : boards_) {
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
