/* Controller for Communication Protocol 
 * Protocol Version: 3
 */

#ifndef BLDCCONTROLLERCLIENT_H
#define BLDCCONTROLLERCLIENT_H

#include <serial/serial.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <exception>

#include "blue_hardware_drivers/comms_defs.h"
#include "blue_hardware_drivers/Packets.h"

class BLDCControllerClient {
  public:
    static constexpr uint16_t crc_16_ibm = 0x8005;
    
    BLDCControllerClient(); //requires init afterwards
    BLDCControllerClient(std::string port);
    void init(std::string port);

    void exchange();

    void queuePacket(comm_id_t server_id, Packet* packet);

    void leaveBootloader(comm_id_t server_id, uint32_t jump_addr);

    void setCurrentControlMode(comm_id_t server_id, bool* result);
    void setZeroAngle(comm_id_t server_id, uint16_t value, bool* result);
    void setERevsPerMRev(comm_id_t server_id, uint8_t value, bool* result);
    void setInvertPhases(comm_id_t server_id, uint8_t value, bool* result);
    void setCommand(comm_id_t server_id, float value, bool* result);
    void setCommandAndGetRotorPosition(comm_id_t server_id, float value, float* result);
    void getRotorPosition(comm_id_t server_id, float* result);


  private:
    serial::Serial ser_;
    std::map<uint8_t, Packet*> packet_queue_;

    void transmit();
    void receive();

    uint16_t computeCRC(std::string message);
};

class comms_error : public std::exception {
  public:
    comms_error(std::string msg) : error(msg){}
    
    virtual const char* what() const throw() {
      return error.c_str();
    } 

  private:
    std::string error;
};

#endif /* BLDCCONTROLLERCLIENT_H */
