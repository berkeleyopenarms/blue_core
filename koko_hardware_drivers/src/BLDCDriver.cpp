#include <koko_hardware_drivers/BLDCDriver.h>

const unsigned int ENCODER_ANGLE_PERIOD = 1 << 14;
/* const double MAX_CURRENT = 2.8; */
const unsigned int CONTROL_LOOP_FREQ = 1000;
const unsigned int BAUD_RATE = 1000000;

void initMaps(std::vector<uint8_t>& angle_id_mapping, 
    std::map<uint8_t, std::string>& motor_mapping,
    std::map<uint8_t, uint16_t>& angle_mapping,
    std::map<uint8_t, uint8_t>& invert_mapping,
    std::map<uint8_t, uint8_t>& erevs_mapping) { // TODO: make this read from config

    angle_id_mapping.push_back(15);
    angle_id_mapping.push_back(11);
    angle_id_mapping.push_back(12);
    angle_id_mapping.push_back(14);
    angle_id_mapping.push_back(16);

    motor_mapping[15] = "base_roll_motor";
    motor_mapping[11] = "right_motor1";
    motor_mapping[12] = "left_motor1";
    motor_mapping[14] = "right_motor2";
    motor_mapping[16] = "left_motor2";

    angle_mapping[15] = 13002;
    angle_mapping[11] = 2164;
    angle_mapping[12] = 1200;
    angle_mapping[14] = 4484;
    angle_mapping[16] = 2373;

    invert_mapping[15] = 0;
    invert_mapping[11] = 0;
    invert_mapping[12] = 0;
    invert_mapping[12] = 0;
    invert_mapping[12] = 0;

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

void BLDCDriver::init(std::vector<double>* in_pos, std::vector<double>* in_vel, std::vector<double>* in_eff, const std::vector<double>* in_cmd) {
  // Init comms/data structures
  pos = in_pos;
  vel = in_vel;
  eff = in_eff;
  cmd = in_cmd;
  
  initMaps(angle_id_mapping, motor_mapping, angle_mapping, invert_mapping, erevs_mapping);

  std::string port = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A506NO9F-if00-port0"; // TODO: read from launch/config
  device.init(port, BAUD_RATE, serial::Timeout::simpleTimeout(10)); // TODO: likely candidate for the frequency issues

  std::map<uint8_t, std::string>::iterator it;
  for (it = motor_mapping.begin(); it != motor_mapping.end(); it++) {
    device.leaveBootloader(it->first, 0);
    device.flushSerial();
  }
  n_sleep(500);


  // Init angle
  for (it = motor_mapping.begin(); it != motor_mapping.end(); it++) {
    angle_zero[it->first] = getEncoderAngleRadians(device, it->first);
  }

  
  // Init driver firmware data structures
  for(std::map<uint8_t, uint16_t>::iterator it2 = angle_mapping.begin(); it2 != angle_mapping.end(); it2++) {
    uint8_t* angle = (uint8_t*) &it2->second;
    device.writeRegisters(it2->first, 0x101, 1, angle, 2);
    uint8_t r = 0;
    device.writeRegisters(it2->first, 0x102, 1, &r, 1);
    device.writeRegisters(it2->first, 0x109, 1, &invert_mapping[it2->first], 1);
    device.writeRegisters(it2->first, 0x10A, 1, &erevs_mapping[it2->first], 1);
  }
}

BLDCDriver::BLDCDriver(){
  pos = NULL;
  vel = NULL;
  eff = NULL;
  cmd = NULL;
}


void BLDCDriver::read(){
  for (int i = 0; i < angle_id_mapping.size(); i++) {
      uint8_t id = angle_id_mapping[i];

      double curr_angle = getEncoderAngleRadians(device, id) - angle_zero[id];

      (*pos)[i] = curr_angle;
      (*vel)[i] = 0.0; // TODO: onboard velocity estimate (need kalman?)
      (*eff)[i] = 0.0; // TODO: current estimate retrieval
    }

}

void BLDCDriver::write(){
  for (int i = 0; i < angle_id_mapping.size(); i++) {
      uint8_t id = angle_id_mapping[i];

      const double cmd_i = (*cmd)[i];
      
      device.setDuty(id, (float)cmd_i);
  }
}

