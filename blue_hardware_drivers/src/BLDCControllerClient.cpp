#include "blue_hardware_drivers/BLDCControllerClient.h"

#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <stdio.h>

#include "blue_hardware_drivers/comms_defs.h"
#include "blue_hardware_drivers/Packets.h"
#include "blue_hardware_drivers/crc16.h"
#include "blue_hardware_drivers/Buffer.h"
#include "json/json.h"

BLDCControllerClient::BLDCControllerClient() {
}

BLDCControllerClient::BLDCControllerClient(std::string port, const std::vector<comm_id_t>& boards) {
  init(port, boards);
}

void BLDCControllerClient::init(std::string port, const std::vector<comm_id_t>& boards) {
  ser_.setPort(port);
  ser_.setBaudrate(COMM_DEFAULT_BAUD_RATE);
  ser_.setTimeout(serial::Timeout::max(), 3, 1, 3, 1);
  ser_.open();

  tx_buf_.init(COMM_MAX_BUF);
  sub_packet_buf_.init(COMM_MAX_BUF);
  payload_buf_.init(COMM_MAX_BUF);

  for (auto id : boards) {
    rx_bufs_[id].init(COMM_MAX_BUF);
  }
}

void BLDCControllerClient::queueLeaveBootloader(comm_id_t server_id, uint32_t jump_addr) {
  if (jump_addr == 0) {
    jump_addr = COMM_FIRMWARE_OFFSET;
  }

  Packet* packet = new JumpToAddrPacket(server_id, jump_addr);
  queuePacket(server_id, packet);
}

void BLDCControllerClient::queueSetCurrentControlMode(comm_id_t server_id) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_VOL_CTRL_MODE, sizeof(COMM_CTRL_MODE), reinterpret_cast<uint8_t*> (new uint8_t(COMM_CTRL_MODE)));
  queuePacket(server_id, packet);
}

void BLDCControllerClient::queueSetTimeout(comm_id_t server_id, uint16_t value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_WATCHDOG, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
}

void BLDCControllerClient::queueSetZeroAngle(comm_id_t server_id, uint16_t value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_REV_START, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
}

void BLDCControllerClient::queueSetERevsPerMRev(comm_id_t server_id, uint8_t value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_EREVS_PER_MREV, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
}

void BLDCControllerClient::queueSetInvertPhases(comm_id_t server_id, uint8_t value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_INV_PHASES, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
}

void BLDCControllerClient::queueSetTorqueConstant(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_MOTOR_T, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
}

void BLDCControllerClient::queueSetPositionOffset(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_POS_OFFSET, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
}

void BLDCControllerClient::queueSetCommand(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_VOL_QI_COMM, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
}

void BLDCControllerClient::queueGetRotorPosition(comm_id_t server_id) {
  // Generate Transmit Packet
  Packet* packet = new ReadRegPacket(server_id, COMM_REG_RO_ROTOR_P, 1);
  queuePacket(server_id, packet);
}

void BLDCControllerClient::resultGetRotorPosition(comm_id_t server_id, float* position) {
  rx_bufs_[server_id].read(reinterpret_cast<uint8_t*>(position), sizeof(*position));
}

void BLDCControllerClient::queueSetCommandAndGetRotorPosition(comm_id_t server_id, float value) {
  // Generate Transmit Packet
  Packet* packet = new ReadWriteRegPacket (server_id, 
    COMM_REG_RO_ROTOR_P, 1,                                          // Read
    COMM_REG_VOL_QI_COMM, sizeof(value), reinterpret_cast<uint8_t*>(&value));  // Write
  queuePacket(server_id, packet);
}

void BLDCControllerClient::queueGetState(comm_id_t server_id) {
  // Generate Transmit Packet
  Packet* packet = new ReadRegPacket (server_id, COMM_REG_RO_ROTOR_P, 9);  
  queuePacket(server_id, packet);
}

void BLDCControllerClient::resultGetState(comm_id_t server_id, float* position, float* velocity, float* di, float* qi, float* voltage, float* temp, uint32_t* acc_x, uint32_t* acc_y, uint32_t* acc_z) {
  rx_bufs_[server_id].read(reinterpret_cast<uint8_t*>(position), sizeof(*position));
  rx_bufs_[server_id].read(reinterpret_cast<uint8_t*>(velocity), sizeof(*velocity));
  rx_bufs_[server_id].read(reinterpret_cast<uint8_t*>(di), sizeof(*di));
  rx_bufs_[server_id].read(reinterpret_cast<uint8_t*>(qi), sizeof(*qi));
  rx_bufs_[server_id].read(reinterpret_cast<uint8_t*>(voltage), sizeof(*voltage));
  rx_bufs_[server_id].read(reinterpret_cast<uint8_t*>(temp), sizeof(*temp));
  rx_bufs_[server_id].read(reinterpret_cast<uint8_t*>(acc_x), sizeof(*acc_x));
  rx_bufs_[server_id].read(reinterpret_cast<uint8_t*>(acc_y), sizeof(*acc_y));
  rx_bufs_[server_id].read(reinterpret_cast<uint8_t*>(acc_z), sizeof(*acc_z));
}

void BLDCControllerClient::queueSetCommandAndGetState(comm_id_t server_id, float value) {
  // Generate Transmit Packet
  Packet* packet = new ReadWriteRegPacket (server_id, 
    COMM_REG_RO_ROTOR_P, 9,                                          // Read
    COMM_REG_VOL_QI_COMM, sizeof(value), reinterpret_cast<uint8_t*>(&value));  // Write
  queuePacket(server_id, packet);
}

void BLDCControllerClient::initMotor(comm_id_t server_id){
  uint32_t len = 0;
  std::string data;

  readFlash(server_id, COMM_NVPARAMS_OFFSET+1, 2, data);
#ifdef DEBUG_CALIBRATION_DATA
  for (unsigned char c : data)
    printf("%02x:", c);
  std::cout << std::endl;
#endif

  std::stringstream buf;
  buf << data;
  buf.read(reinterpret_cast<char*>(&len), sizeof(len));

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << len << std::endl;
#endif
  data = "";
  readFlash(server_id, COMM_NVPARAMS_OFFSET+3, len, data); 

  Json::Reader reader;
  Json::Value calibrations;
  reader.parse(data, calibrations);

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Zero Angle: " << calibrations["angle"].asUInt() << std::endl
            << "Invert Phases: " << calibrations["inv"].asUInt() << std::endl
            << "Encoder Revs Per Magnetic Revolution: " << calibrations["epm"].asUInt() << std::endl
            << "Torque Constant: " << calibrations["torque"].asFloat() << std::endl
            << "Position Offset: " << calibrations["zero"].asFloat() << std::endl;
#endif

  queueSetZeroAngle(server_id, (uint16_t) calibrations["angle"].asUInt());
  exchange(); 
  queueSetInvertPhases(server_id, (uint8_t) calibrations["inv"].asUInt());
  exchange(); 
  queueSetERevsPerMRev(server_id, (uint8_t) calibrations["epm"].asUInt());
  exchange(); 
  queueSetTorqueConstant(server_id, calibrations["torque"].asFloat());
  exchange(); 
  queueSetPositionOffset(server_id, calibrations["zero"].asFloat());
  exchange(); 
  queueSetCurrentControlMode(server_id);
  exchange(); 
}

// Sends packets to boards and collects data
void BLDCControllerClient::exchange() {
  transmit();
  // Receive data from each board in order
  for (auto it = packet_queue_.begin(); it != packet_queue_.end(); it++) {
#ifdef DEBUG_RECEIVE
    std::cout << "Expected packet from board " << (int) it->first << std::endl;
#endif
    while (!receive(it->first)); // Get server id (key in [key, value] pair)
  }
  // Empty the queue
  packet_queue_.clear();
}

void BLDCControllerClient::clearQueue() {
  for (auto it = packet_queue_.begin(); it != packet_queue_.end(); it++) {
    auto packet = it->second; // Get packet pointer (value in [key, value] pair)
    if (packet != nullptr) {
      delete packet;
      it->second = nullptr;
    }
  }
  //ser_.flush();
  packet_queue_.clear();
}

/* Private Flash Accessor/Mutator Members */
void BLDCControllerClient::readFlash(comm_id_t server_id, comm_full_addr_t addr, uint32_t count, std::string& buffer){
  std::vector<comm_id_t> board;
  board.push_back(server_id);

  for (size_t i = 0; i < count; i += std::min(count - i, COMM_SINGLE_READ_LENGTH)) {
    queuePacket(server_id, new ReadFlashPacket(server_id, addr, count)); 
    exchange(); // This can error which means when running flash commands make sure to try/catch comm_error!   
    buffer.append(rx_bufs_[server_id].remain_str()); 
  }
}

/* Private Transmission Members */
void BLDCControllerClient::queuePacket(comm_id_t server_id, Packet* packet) {
  if (packet_queue_[server_id] != nullptr) {
    std::string error = "A packet is already queued for motor " + std::to_string((int) server_id);
    throw comms_error(error);
  }
  packet_queue_[server_id] = packet;
}

void BLDCControllerClient::transmit() {
  tx_buf_.clear();
  // Add Header
  tx_buf_.write(reinterpret_cast<uint8_t*> (new uint8_t(COMM_SYNC_FLAG)), sizeof(COMM_SYNC_FLAG));
  tx_buf_.write(reinterpret_cast<uint8_t*> (new uint8_t(COMM_VERSION)), sizeof(COMM_VERSION));
  tx_buf_.write(reinterpret_cast<uint8_t*> (new comm_fg_t(COMM_FG_COMP)), sizeof(COMM_FG_COMP));

  // Generate Payload
  payload_buf_.clear();
  for (auto it = packet_queue_.begin(); it != packet_queue_.end(); it++) {
    sub_packet_buf_.clear();
    // Acquire packet byte format
    auto packet = it->second; // Get packet pointer (value in [key, value] pair)
    packet->dump(sub_packet_buf_);

#ifdef DEBUG_TRANSMIT
    // Print Sub-message packet
    std::string msg = sub_packet_buf_.str();
    std::cout << "Sub-packet for " << (int) id << ": ";
    for (unsigned char c : msg)
      printf("%02x:", c);
    std::cout << std::endl;
#endif

    comm_msg_len_t sub_msg_len = sub_packet_buf_.size(); 
    payload_buf_.write(reinterpret_cast<uint8_t*> (&sub_msg_len), sizeof(sub_msg_len));
    payload_buf_.addBuf(sub_packet_buf_);

    delete packet;
    it->second = nullptr;
  }
  // Add total packet length
  comm_msg_len_t payload_len = payload_buf_.size();
  tx_buf_.write(reinterpret_cast<uint8_t*> (&payload_len), sizeof(payload_len));
  tx_buf_.addBuf(payload_buf_);

  // Adding CRC
  uint16_t crc = computeCRC(payload_buf_.ptr(), payload_buf_.size());
  tx_buf_.write(reinterpret_cast<uint8_t*> (&crc), sizeof(crc));

#ifdef DEBUG_TRANSMIT
  // print total packet
  std::string msg = tx_buf_.str();
  std::cout << "Transmitting: ";
  for (unsigned char c : msg)
    printf("%02x:", c);
  std::cout << std::endl;
#endif
  
  // Send Packet!
  size_t write_len = ser_.write(tx_buf_.ptr(), tx_buf_.size());
  ser_.flushOutput();
  if (write_len != tx_buf_.size()) {
    throw comms_error("Failed to transmit full packet");
  }
}

void BLDCControllerClient::ser_read_check(uint8_t * data, size_t len) {
  int read_len = 0;  
  size_t tries = 0;
  do { 
    read_len = ser_.read(data, len);
  } while (read_len == 0 && read_len != -1 && tries++ < COMM_MAX_RETRIES);

  if (read_len == -1) {
    throw comms_error("Serial Port Closed");
  }
  if (read_len != len) {
    std::string msg = "Not enough data received. Expected " + std::to_string(len) + " got " + std::to_string(read_len);
    throw comms_error(msg);
  }
}

bool BLDCControllerClient::receive( comm_id_t server_id ) {
  uint8_t sync = 0;
  ser_read_check(&sync, sizeof(sync));
  if (sync != COMM_SYNC_FLAG) {
    return false;
  }

  comm_protocol_t protocol_version = 0;
  ser_read_check(reinterpret_cast<uint8_t*>(&protocol_version), sizeof(protocol_version)); 
  if (protocol_version != COMM_VERSION) {
    return false;
  }

  comm_fg_t flags = 0;
  ser_read_check(reinterpret_cast<uint8_t*>(&flags), sizeof(flags));
  if ((flags & COMM_FG_BOARD) != COMM_FG_BOARD) {
    return false;
  }

  comm_msg_len_t packet_len = 0;
  ser_read_check(reinterpret_cast<uint8_t*>(&packet_len), sizeof(packet_len));

#ifdef DEBUG_RECEIVE
  std::cout << "Received packet length: " << (int) packet_len << std::endl;
#endif
  
  rx_bufs_[server_id].clear();
  
  if (!rx_bufs_[server_id].addHead(packet_len))
    throw comms_error("rx buffer too small for message"); 
  ser_read_check(rx_bufs_[server_id].ptr(), packet_len);

#ifdef DEBUG_RECEIVE
  // print message 
  std::string message = rx_bufs_[server_id].str();
  std::cout << "Receiving: ";
  for (unsigned char c : message)
    printf("%02x:", c);
  std::cout << std::endl;
#endif
  
  crc16_t crc = 0;
  ser_read_check(reinterpret_cast<uint8_t*>(&crc), 2);
  
  crc16_t computed_crc = computeCRC(rx_bufs_[server_id].ptr(), rx_bufs_[server_id].size());

#ifdef DEBUG_RECEIVE_CRC
  // Print the two CRCs
  std::cout << "Received CRC: " << std::hex << crc << std::endl;

  std::cout << "Calculated CRC: " << std::hex << computed_crc << std::endl;
#endif

  if (computed_crc != crc) {
    throw comms_error("Incorrect crc!");
  }

  // Parse message
  comm_msg_len_t sub_msg_len = 0;
  rx_bufs_[server_id].read(reinterpret_cast<uint8_t*>(&sub_msg_len), sizeof(sub_msg_len));
  
  comm_id_t id = 0;
  rx_bufs_[server_id].read(reinterpret_cast<uint8_t*>(&id), sizeof(id));
  if (id != server_id) {
    throw comms_error("Incorrect server id, expected: " + std::to_string((int)server_id) + ", got: " + std::to_string((int)id));
  }

  comm_fc_t func_code = 0;
  rx_bufs_[server_id].read(reinterpret_cast<uint8_t*>(&func_code), sizeof(func_code));

  comm_errors_t errors = 0;
  rx_bufs_[server_id].read(reinterpret_cast<uint8_t*>(&errors), sizeof(errors));
 
  /* 
  if (errors) {
    if (errors & COMM_ERRORS_OP_FAILED) {
      ROS_ERROR("operation failed\n");
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
  }
  */

  return true;
}

crc16_t BLDCControllerClient::computeCRC( const uint8_t* buf, size_t len ) {
  crc16_t crc = crc16_init();
  crc = crc16_update(crc, buf, len);
  return crc16_finalize(crc);
}

