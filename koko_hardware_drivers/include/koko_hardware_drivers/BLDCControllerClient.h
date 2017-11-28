#ifndef BLDCCONTROLLERCLIENT_H
#define BLDCCONTROLLERCLIENT_H

#include <serial/serial.h>
#include <iostream>
#include <vector>
#include <string>

#include "koko_hardware_drivers/comms.h"

typedef std::vector<uint8_t> bytebuf_t;

class BLDCControllerClient {
  private:
    serial::Serial ser_;
    void writeRequest_(uint8_t server_id, uint8_t func_code, bytebuf_t& data);
    bool writeRegisters_(uint8_t server_id, uint32_t start_addr, uint8_t count, uint8_t* data, size_t size);
    bytebuf_t readResponse_(uint8_t server_id, uint8_t func_code);
    bytebuf_t readRegisters_(uint8_t server_id, uint32_t start_addr, uint8_t count);
    bytebuf_t doTransaction_(uint8_t server_id, uint8_t func_code, bytebuf_t& data);

  public:
    BLDCControllerClient(); //requires init afterwards
    BLDCControllerClient(std::string port);
    void init(std::string port);

    float getRotorPosition(uint8_t server_id);
    bool setCurrentControlMode(uint8_t server_id);
    bool setZeroAngle(uint8_t server_id, uint16_t value);
    bool setERevsPerMRev(uint8_t server_id, uint8_t value);
    bool setInvertPhases(uint8_t server_id, uint8_t value);
    bool setCommand(uint8_t server_id, float value);
    bool leaveBootloader(uint8_t server_id, uint32_t jump_addr);
};

#endif /* BLDCCONTROLLERCLIENT_H */
