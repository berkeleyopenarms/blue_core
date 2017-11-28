#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <stdio.h>

#include "koko_hardware_drivers/comms.h"
#include "koko_hardware_drivers/BLDCControllerClient.h"


BLDCControllerClient::BLDCControllerClient() {
}

BLDCControllerClient::BLDCControllerClient(std::string port) {
  init(port);
}

void BLDCControllerClient::init(std::string port) {
  ser_.setPort(port);
  ser_.setBaudrate(1000000);
  ser_.setTimeout(serial::Timeout::max(), 10, 0, 10, 0);
  ser_.open();
}

float BLDCControllerClient::getRotorPosition(uint8_t server_id) {
  bytebuf_t message = BLDCControllerClient::readRegisters_(server_id, 0x3000, 1);

  uint8_t temp[4];
  temp[0] = message.at(0);
  temp[1] = message.at(1);
  temp[2] = message.at(2);
  temp[3] = message.at(3);

  return *reinterpret_cast<float*>(temp);
}

bool BLDCControllerClient::setCurrentControlMode(uint8_t server_id) {
  uint8_t zero = 0; // this feels wrong
  return BLDCControllerClient::writeRegisters_(server_id, 0x2000, 1, &zero, 1);
}

bool BLDCControllerClient::setZeroAngle(uint8_t server_id, uint16_t value) {
  return BLDCControllerClient::writeRegisters_(server_id, 0x1000, 1, (uint8_t*) &value, 2);
}

bool BLDCControllerClient::setERevsPerMRev(uint8_t server_id, uint8_t value) {
  return BLDCControllerClient::writeRegisters_(server_id, 0x1001, 1, (uint8_t*) &value, 1);
}

bool BLDCControllerClient::setInvertPhases(uint8_t server_id, uint8_t value) {
  return BLDCControllerClient::writeRegisters_(server_id, 0x1002, 1, (uint8_t*) &value, 1);
}

bool BLDCControllerClient::leaveBootloader(uint8_t server_id, uint32_t jump_addr) {
  if (jump_addr == 0) {
    jump_addr = COMM_FIRMWARE_OFFSET;
  }

  uint8_t* src = (uint8_t*) &jump_addr;
  bytebuf_t buffer;
  buffer.insert(buffer.end(), 5, 0);
  buffer.push_back(src[0]);
  buffer.push_back(src[1]);
  buffer.push_back(src[2]);
  buffer.push_back(src[3]);
  BLDCControllerClient::writeRequest_(server_id, COMM_FC_JUMP_TO_ADDR, buffer);

  uint8_t dumping = 0;
  while (ser_.read(&dumping, 1)){};

  ser_.flush();

  return true;
}

bool BLDCControllerClient::setCommand(uint8_t server_id, float value) {
  return BLDCControllerClient::writeRegisters_(server_id, 0x0106, 1, reinterpret_cast<uint8_t*>(&value), 4);
}

float BLDCControllerClient::setCommandAndGetRotorPosition(uint8_t server_id, float value) {

  bytebuf_t message = BLDCControllerClient::readWriteRegisters_(
    server_id,
    0x3000,
    1,
    0x0106,
    1,
    reinterpret_cast<uint8_t*>(&value),
    4
  );

  uint8_t temp[4];
  temp[0] = message.at(0);
  temp[1] = message.at(1);
  temp[2] = message.at(2);
  temp[3] = message.at(3);

  return *reinterpret_cast<float*>(temp);

}

/* Private members */

void BLDCControllerClient::writeRequest_(uint8_t server_id, uint8_t func_code, bytebuf_t& data) {
  data.at(0) = 0xFF; // Sync flag
  data.at(1) = 0xFF; // Protocol version
  data.at(2) = data.size() - 4; // Add length field, which excludes the sync, version, & length bytes
  data.at(3) = 0;
  data.at(4) = server_id;
  data.at(5) = func_code;

  //TODO: Make this return CRC
  data.push_back(0);
  data.push_back(0);
  ser_.write(data);
}

bytebuf_t BLDCControllerClient::readResponse_(uint8_t server_id, uint8_t func_code) {
  bytebuf_t empty(0);
  uint8_t sync = 1;
  uint8_t test = 0xFF;
  ser_.read(&sync, 1);
  if (sync != 0xFF) {
    ser_.flushInput();
    return empty;
  }

  uint8_t protocol_version = 0;
  ser_.read(&protocol_version, 1);
  if (protocol_version != 0xFF) {
    ser_.flushInput();
    return empty;
  }

  uint16_t length = 0;
  ser_.read(reinterpret_cast<uint8_t*>(&length), 2);

  if (length == 0) {
    std::cout << "length was zero\n";
    ser_.flushInput();
    return empty;
  }

  comm_id_t message_server_id = 0;
  comm_fc_t message_func_code = 0;
  uint8_t errorsh = 0;
  uint8_t errorsl = 0;

  bool complete_read = true;
  complete_read &= ser_.read(&message_server_id, 1);
  complete_read &= ser_.read(&message_func_code, 1);
  complete_read &= ser_.read(&errorsl, 1);
  complete_read &= ser_.read(&errorsh, 1);

  comm_errors_t errors = errorsl + (errorsh << 8);
  if (!complete_read || message_server_id != server_id || message_func_code != func_code) {
    std::cerr << "Incomplete serial read or received unexpected server ID or function code\n";
    throw "error";
  }
  if (errors) {
    if (errors & COMM_ERRORS_OP_FAILED) {
      std::cout << "operation failed\n";
    }
    if (errors & COMM_ERRORS_MALFORMED) {
      std::cout << "malformed request\n";
    }
    if (errors & COMM_ERRORS_INVALID_FC) {
      std::cout << "invalid function code\n";
    }
    if (errors & COMM_ERRORS_INVALID_ARGS) {
      std::cout << "invalid arguments\n";
    }
    if (errors & COMM_ERRORS_BUF_LEN_MISMATCH) {
      std::cout << "buffer length mismatch\n";
    }
    return empty;
  }

  bytebuf_t message;

  if (length - 4 == 0 || ser_.read(message, length - 4) < (size_t) (length - 4)) {
    std::cout << "message length was not long enough\n";
    ser_.flushInput();
    return empty;
  }

  /* uint8_t crc_bytesh = 0; */
  /* uint8_t crc_bytesl = 0; */
  uint16_t crc_bytes = 0;
  if (ser_.read(reinterpret_cast<uint8_t*>(&crc_bytes), 2) < 2) {
    std::cout << "crc bytes not read properly\n";
    ser_.flushInput();
    return empty;
  }

  // TODO: CRC check

  return message;
}

bytebuf_t BLDCControllerClient::doTransaction_(uint8_t server_id, uint8_t func_code, bytebuf_t& data) {
  writeRequest_(server_id, func_code, data);
  ser_.flush();
  return readResponse_(server_id, func_code);
}

bool BLDCControllerClient::writeRegisters_(uint8_t server_id, uint32_t start_addr, uint8_t count, uint8_t* data, size_t size) {
  bytebuf_t buffer;
  buffer.insert(buffer.end(), 6, 0);
  buffer.push_back(start_addr & 0xFF);
  buffer.push_back(start_addr >> 8);
  buffer.push_back(count);
  for (size_t i = 0; i < size; i++) {
    buffer.push_back(data[i]);
  }
  bytebuf_t message = BLDCControllerClient::doTransaction_(server_id, COMM_FC_REG_WRITE, buffer);
  if (message.size() == 0) {
    return false;
  }
  return true;
}

bytebuf_t BLDCControllerClient::readRegisters_(uint8_t server_id, uint32_t start_addr, uint8_t count) {
  bytebuf_t buffer;
  buffer.insert(buffer.end(), 6, 0);
  buffer.push_back(start_addr & 0xFF);
  buffer.push_back(start_addr >> 8);
  buffer.push_back(count);

  bytebuf_t data = BLDCControllerClient::doTransaction_(server_id, COMM_FC_REG_READ, buffer);
  if (data.size() == 0) {
    std::cerr << "Register read failed\n";
    throw "error";
  }
  return data;
}

bytebuf_t BLDCControllerClient::readWriteRegisters_(
  uint8_t server_id,
  uint32_t read_start_addr,
  uint8_t read_count,
  uint32_t write_start_addr,
  uint8_t write_count,
  uint8_t* write_data,
  size_t write_size
) {
  bytebuf_t buffer;
  buffer.insert(buffer.end(), 6, 0);

  buffer.push_back(read_start_addr & 0xFF);
  buffer.push_back(read_start_addr >> 8);
  buffer.push_back(read_count);

  buffer.push_back(write_start_addr & 0xFF);
  buffer.push_back(write_start_addr >> 8);
  buffer.push_back(write_count);
  for (size_t i = 0; i < write_size; i++) {
    buffer.push_back(write_data[i]);
  }

  bytebuf_t data = BLDCControllerClient::doTransaction_(server_id, COMM_FC_REG_READ_WRITE, buffer);
  if (data.size() == 0) {
    std::cerr << "Register read failed\n";
    throw "error";
  }
  return data;
}

