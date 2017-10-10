#include <serial/serial.h>
#include <iostream>
#include <vector>
#include <comms.h>
#include <BLDCControllerClient.cpp>
#include <time.h>
#include <string>

int main(int argc, char** argv) {

  BLDCControllerClient::BLDCControllerClient client(argv[1], 1000000, //timeout object);
}
