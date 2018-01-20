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
    bytebuf_t readResponse_(uint8_t server_id, uint8_t func_code);
    bytebuf_t doTransaction_(uint8_t server_id, uint8_t func_code, bytebuf_t& data);

    bytebuf_t readRegisters_(uint8_t server_id, uint32_t start_addr, uint8_t count);
    bool writeRegisters_(
      uint8_t server_id,
      uint32_t start_addr,
      uint8_t count,
      uint8_t* data,
      size_t size
    );
    bytebuf_t readWriteRegisters_(
      uint8_t server_id,
      uint32_t read_start_addr,
      uint8_t read_count,
      uint32_t write_start_addr,
      uint8_t write_count,
      uint8_t* write_data,
      size_t write_size
    );

  public:
    BLDCControllerClient(); //requires init afterwards
    BLDCControllerClient(std::string port);
    void init(std::string port);

    bool leaveBootloader(uint8_t server_id, uint32_t jump_addr);

    bool setCurrentControlMode(uint8_t server_id);
    bool setZeroAngle(uint8_t server_id, uint16_t value);
    bool setERevsPerMRev(uint8_t server_id, uint8_t value);
    bool setInvertPhases(uint8_t server_id, uint8_t value);
    bool setCommand(uint8_t server_id, float value);
    float setCommandAndGetRotorPosition(uint8_t server_id, float value);
    float getRotorPosition(uint8_t server_id);
};

#endif /* BLDCCONTROLLERCLIENT_H */
