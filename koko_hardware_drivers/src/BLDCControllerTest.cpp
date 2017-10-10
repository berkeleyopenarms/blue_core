#include <serial/serial.h>
#include <iostream>
#include <vector>
/* #include <comms.h> */
/* #include <BLDCControllerClient.cpp> */
#include <BLDCControllerClient.h>
//#include <unistd.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

const unsigned int ENCODER_ANGLE_PERIOD = 1 << 14;
int main(int argc, char** argv) {
  if (argc < 2) {
    return 0;
  }
  BLDCControllerClient client(argv[1], 1000000, serial::Timeout::simpleTimeout(10));
  /*float angle = (float) client.getEncoder(3) / ENCODER_ANGLE_PERIOD * 2 * M_PI;
  std::cout << angle << "\n";
  sleep(100);*/
  client.setDuty(3, 6.0);
  uint8_t r1 = 0;
  uint8_t r2 = 1;
  client.writeRegisters(3, 0x102, 1, &r1, 1);
  client.writeRegisters(3, 0x109, 1, &r2, 1);
  return 0;
}
