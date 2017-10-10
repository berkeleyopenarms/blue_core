#include <serial/serial.h>
#include <iostream>
#include <vector>
#include <comms.h>
#include <BLDCControllerClient.h>
#include <time.h>
#include <string>

void sleep(unsigned int mseconds) {
  clock_t goal = mseconds + clock();
  while (goal > clock());
}
BLDCControllerClient::BLDCControllerClient(std::string port, unsigned int baud, serial::Timeout to) {
  ser.setPort(port);
  ser.setBaudrate(baud);
  ser.setTimeout(to);
  ser.open();
}

void BLDCControllerClient::writeRequest(uint8_t server_id, uint8_t func_code, bytebuf_t& data) {
  data.at(0) = data.size() - 1;
  data.at(1) = server_id;
  data.at(2) = func_code;
  //TODO: Make this return CRC
  data.push_back(0);
  data.push_back(0);
  ser.write(data);
}

bytebuf_t BLDCControllerClient::readResponse(uint8_t server_id, uint8_t func_code, unsigned int num_tries, unsigned int try_interval) {
  uint8_t lb = 0;
  for (int i = 0; i < num_tries && lb == 0; i++) {
    ser.read(&lb, 1);
    sleep(try_interval);
  }
  bytebuf_t empty(0);
  if (lb == 0) {
    ser.flushInput();
    return empty;
  }

  //TODO: confirm endianness
  uint8_t message_server_id = 0;
  uint8_t message_func_code = 0;
  uint16_t errors = 0;
  ser.read(&message_server_id, 1);
  ser.read(&message_func_code, 1);
  ser.read(reinterpret_cast<uint8_t*>(&errors), 2);
  if (message_server_id != server_id || message_func_code != func_code) {
    throw "Received unexpected server ID or function code";
  }

  bytebuf_t message(lb - 4);
  if (ser.read(message, lb - 4) < lb - 4) {
    ser.flushInput();
    return empty;
  }

  uint16_t crc_bytes = 0;
  if (ser.read(reinterpret_cast<uint8_t*>(&crc_bytes), 2) < 2) {
    ser.flushInput();
    return empty;
  }

  // TODO: CRC check

  if ((errors & COMM_ERRORS_OP_FAILED) != 0) {
    return empty;
  }

  return message;
}

bytebuf_t BLDCControllerClient::doTransaction(uint8_t server_id, uint8_t func_code, bytebuf_t& data) {
  BLDCControllerClient::writeRequest(server_id, func_code, data);
  ser.flushInput();
  return BLDCControllerClient::readResponse(server_id, func_code, 10, 10);
}

bool BLDCControllerClient::writeRegisters(uint8_t server_id, uint16_t start_addr, uint8_t count, uint8_t* data, size_t size) {
  bytebuf_t buffer(size + 8);
  buffer.push_back(0); buffer.push_back(0); buffer.push_back(0);
  //TODO: confirm endianness
  buffer.push_back(start_addr >> 8);
  buffer.push_back(start_addr & 0xFF);
  buffer.push_back(count);
  for (int i = 0; i < size; i++) {
    buffer.push_back(data[i]);
  }
  bytebuf_t message = BLDCControllerClient::doTransaction(server_id, COMM_FC_READ_REGS, buffer);
  if (message.size() == 0) {
    return false;
  }
  return true;
}

bytebuf_t BLDCControllerClient::readRegisters(uint8_t server_id, uint16_t start_addr, uint8_t count) {
  bytebuf_t buffer(8);
  buffer.push_back(0); buffer.push_back(0); buffer.push_back(0);
  //TODO: confirm endianness
  buffer.push_back(start_addr >> 8);
  buffer.push_back(start_addr & 0xFF);
  buffer.push_back(count);

  bytebuf_t data = BLDCControllerClient::doTransaction(server_id, COMM_FC_READ_REGS, buffer);
  if (data.size() == 0) {
    throw "Register read failed";
  }
  return data;
}
