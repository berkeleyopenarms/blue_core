#include "blue_hardware_drivers/BLDCControllerClient.h"

#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <stdio.h>

#include "blue_hardware_drivers/comms_defs.h"
#include "blue_hardware_drivers/Packets.h"

BLDCControllerClient::BLDCControllerClient() {
}

BLDCControllerClient::BLDCControllerClient(std::string port) {
  init(port);
}

void BLDCControllerClient::init(std::string port) {
  ser_.setPort(port);
  ser_.setBaudrate(COMM_DEFAULT_BAUD_RATE);
  ser_.setTimeout(serial::Timeout::max(), 20, 0, 20, 0);
  ser_.open();
}

void BLDCControllerClient::leaveBootloader(comm_id_t server_id, uint32_t jump_addr) {
  if (jump_addr == 0) {
    jump_addr = COMM_FIRMWARE_OFFSET;
  }

  Packet* packet = new JumpToAddrPacket(server_id, jump_addr);
  queuePacket(server_id, packet);

  // TODO: Generate Receive Packet
}

void BLDCControllerClient::getRotorPosition(comm_id_t server_id, float* result) {
  // Generate Transmit Packet
  Packet* packet = new ReadRegPacket(server_id, COMM_REG_RO_ROTOR_P, sizeof(*result));
  queuePacket(server_id, packet);

  // TODO: Generate Receive Packet
}

void BLDCControllerClient::setCurrentControlMode(comm_id_t server_id, bool* result) {
  // Create new packet
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_VOL_CTRL_MODE, sizeof(COMM_CTRL_MODE), reinterpret_cast<char*> (new uint8_t(COMM_CTRL_MODE)));
  queuePacket(server_id, packet);

  // TODO: Generate Receive Packet
}


void BLDCControllerClient::setZeroAngle(comm_id_t server_id, uint16_t value, bool* result) {
  // Create new packet
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_REV_START, sizeof(value), reinterpret_cast<char*> (&value));
  queuePacket(server_id, packet);

  // TODO: Generate Receive Packet
}

void BLDCControllerClient::setERevsPerMRev(comm_id_t server_id, uint8_t value, bool* result) {
  // Create new packet
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_EREVS_PER_MREV, sizeof(value), reinterpret_cast<char*> (&value));
  queuePacket(server_id, packet);

  // TODO: Generate Receive Packet
}

void BLDCControllerClient::setInvertPhases(comm_id_t server_id, uint8_t value, bool* result) {
  // Create new packet
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_INV_PHASES, sizeof(value), reinterpret_cast<char*> (&value));
  queuePacket(server_id, packet);

  // TODO: Generate Receive Packet
}

void BLDCControllerClient::setCommand(comm_id_t server_id, float value, bool* result) {
  // Create new packet
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_VOL_DI_COMM, sizeof(value), reinterpret_cast<char*> (&value));
  queuePacket(server_id, packet);

  // TODO: Generate Receive Packet
}

void BLDCControllerClient::setCommandAndGetRotorPosition(comm_id_t server_id, float value, float* result) {
  // Generate Transmit Packet
  Packet* packet = new ReadWriteRegPacket (server_id, 
    COMM_REG_RO_ROTOR_P, sizeof(*result),                                   // Read
    COMM_REG_VOL_DI_COMM, sizeof(value), reinterpret_cast<char*>(&value));  // Write
  queuePacket(server_id, packet);
}

// Sends packets to boards and collects data
void BLDCControllerClient::exchange() {
  transmit();
  receive();
}

/* Private members */

void BLDCControllerClient::queuePacket(comm_id_t server_id, Packet* packet) {
  if (packet_queue_[server_id] != nullptr) {
    std::string error = "A packet is already queued for motor " + std::to_string((int) server_id);
    throw comms_error(error);
  }
  packet_queue_[server_id] = packet;
}

void BLDCControllerClient::transmit() {
  // Transmission buffer
  std::stringstream tx_buf;
  // Add Header
  tx_buf.write(reinterpret_cast<char*> (new uint8_t(COMM_SYNC_FLAG)), sizeof(COMM_SYNC_FLAG));
  tx_buf.write(reinterpret_cast<char*> (new uint8_t(COMM_VERSION)), sizeof(COMM_VERSION));
  tx_buf.write(reinterpret_cast<char*> (new comm_fg_t(COMM_FG_COMP)), sizeof(COMM_FG_COMP));

  // Generate Payload
  std::stringstream payload;
  for (auto it = packet_queue_.begin(); it != packet_queue_.end(); it++) {
    // Acquire packet byte format
    auto packet = it->second; // Get packet pointer (value in key pair)
    std::string msg = packet->dump();

    for (unsigned char c : msg)
      printf("%02x:", c);
    std::cout << std::endl;

    comm_msg_len_t sub_msg_len = msg.size(); 
    payload.write(reinterpret_cast<char*> (&sub_msg_len), sizeof(sub_msg_len));
    payload << msg;
    delete packet;
  }
  // Add total packet length
  std::string payload_str = payload.str();
  comm_msg_len_t payload_len = payload_str.size();
  tx_buf.write(reinterpret_cast<char*> (&payload_len), sizeof(payload_len));
  tx_buf << payload_str;

  // Adding CRC
  uint16_t crc = computeCRC(payload_str);
  tx_buf.write(reinterpret_cast<char*> (&crc), sizeof(crc));

  std::string tx_str = tx_buf.str();

  for (unsigned char c : tx_str)
    printf("%02x:", c);
  std::cout << std::endl;
  
  // Send Packet!
  ser_.write(tx_str);
  // Empty the queue
  packet_queue_.clear();
}

void BLDCControllerClient::receive() {
  // TODO: Write board receive code!
}

uint16_t BLDCControllerClient::computeCRC( std::string message ) {
  const char* buf = message.c_str();
  size_t len = message.size();
  uint16_t out = 0;
  uint16_t bits_read = 0, bit_flag;

  /* Sanity check */
  if (buf == nullptr)
    return 0;

  while (len > 0) {
    bit_flag = out >> 15;

    /* Get next bit: */
    out <<= 1;
    out |= (*buf >> bits_read) & 1; // item a) work from the least significant bits
    
    /* Increment bit counter: */
    bits_read++;
    if(bits_read > 7) {
      bits_read = 0;
      buf++;
      len--;
    }
    
    /* Cycle check: */
    if(bit_flag) out ^= crc_16_ibm;
  }

  // item b) "push out" the last 16 bits
  int i;
  for (i = 0; i < 16; ++i) {
    bit_flag = out >> 15;
    out <<= 1;
    if(bit_flag) out ^= crc_16_ibm;
  }

  // item c) reverse the bits
  uint16_t crc = 0;
  i = 0x8000;
  int j = 0x0001;
  for (; i != 0; i >>=1, j <<= 1) {
    if (i & out) crc |= j;
  }

  return crc;
}

/*
ByteBuf BLDCControllerClient::readResponse_(comm_id_t server_id, uint8_t func_code) {
  ByteBuf empty(0);
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
  ser_.read(reinterpret_cast<char*>(&length), 2);

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
    throw;
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

  ByteBuf message;

  if (length - 4 == 0 || ser_.read(message, length - 4) < (size_t) (length - 4)) {
    std::cerr << "message length was not long enough\n";
    ser_.flushInput();
    return empty;
  }

  // uint8_t crc_bytesh = 0; 
  // uint8_t crc_bytesl = 0; 
  uint16_t crc_bytes = 0;
  if (ser_.read(reinterpret_cast<char*>(&crc_bytes), 2) < 2) {
    std::cout << "crc bytes not read properly\n";
    ser_.flushInput();
    return empty;
  }

  // TODO: CRC check

  return message;
}
*/



