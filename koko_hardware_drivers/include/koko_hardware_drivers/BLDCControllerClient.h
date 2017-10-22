#ifndef BLDCCONTROLLERCLIENT_H
#define BLDCCONTROLLERCLIENT_H

#include <serial/serial.h>
#include <iostream>
#include <vector>
#include <string>

typedef std::vector<uint8_t> bytebuf_t;

class BLDCControllerClient {
  private:
    serial::Serial ser;
  public:
    void writeRequest(uint8_t server_id, uint8_t func_code, bytebuf_t& data);
    bool writeRegisters(uint8_t server_id, uint16_t start_addr, uint8_t count, uint8_t* data, size_t size);
    bytebuf_t readResponse(uint8_t server_id, uint8_t func_code, unsigned int num_tries, unsigned int try_interval);
    bytebuf_t readRegisters(uint8_t server_id, uint16_t start_addr, uint8_t count);
    bytebuf_t doTransaction(uint8_t server_id, uint8_t func_code, bytebuf_t& data);
    float getEncoderRadians(uint8_t id);
    uint16_t getEncoder(uint8_t id);
    bool leaveBootloader(uint8_t server_id, unsigned int jump_addr);
    bool setDuty(uint8_t id, float value);
    void flushSerial();
    BLDCControllerClient(std::string port, unsigned int baud, serial::Timeout to);
};

void n_sleep(unsigned int milliseconds);
#endif /* BLDCCONTROLLERCLIENT_H */
