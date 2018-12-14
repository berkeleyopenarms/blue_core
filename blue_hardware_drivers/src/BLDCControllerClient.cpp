#include "blue_hardware_drivers/BLDCControllerClient.h"
#include <serial/serial.h>
#include <iostream>
#include <string>

// Needed for setting low latency flag
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <stdio.h>

#include "blue_hardware_drivers/comms_defs.h"
#include "blue_hardware_drivers/Packets.h"
#include "blue_hardware_drivers/crc16.h"
#include "blue_hardware_drivers/Buffer.h"
#include "json/json.h"

namespace blue_hardware_drivers {

BLDCControllerClient::BLDCControllerClient() {
}

BLDCControllerClient::BLDCControllerClient(std::string port, const std::vector<comm_id_t>& boards) {
  init(port, boards);
  allocs_ = 0;
}

void BLDCControllerClient::init(std::string port, const std::vector<comm_id_t>& boards) {
  // First, manualy set the port to low latency mode
  int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  struct serial_struct ser;
  ioctl(fd, TIOCGSERIAL, &ser);
  ser.flags |= ASYNC_LOW_LATENCY;
  ioctl(fd, TIOCSSERIAL, &ser);
  close(fd);

  // Then set up serial library
  ser_.setPort(port);
  ser_.setBaudrate(COMM_DEFAULT_BAUD_RATE);
  ser_.setTimeout(serial::Timeout::max(), 4, 1, 4, 1);
  ser_.open();

  tx_buf_.init(COMM_MAX_BUF);
  sub_packet_buf_.init(COMM_MAX_BUF);
  payload_buf_.init(COMM_MAX_BUF);

  for (auto id : boards) {
    rx_bufs_[id].init(COMM_MAX_BUF);
  }
}

void BLDCControllerClient::initMotor(comm_id_t server_id){
  uint32_t len = 0;
  std::string data;

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Calibrating board: " << (int) server_id << std::endl;
#endif

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
  std::cout << "Calibration length: " << len << std::endl;
#endif
  data = "";
  readFlash(server_id, COMM_NVPARAMS_OFFSET+3, len, data);

  Json::Reader reader;
  Json::Value calibrations;
  reader.parse(data, calibrations);

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Zero Angle: " << calibrations["angle"].asUInt() << std::endl;
#endif
  queueSetZeroAngle(server_id, (uint16_t) calibrations["angle"].asUInt());
  exchange();

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Invert Phases: " << calibrations["inv"].asUInt() << std::endl;
#endif
  queueSetInvertPhases(server_id, (uint8_t) calibrations["inv"].asUInt());
  exchange();

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Encoder Revs Per Magnetic Revolution: " << calibrations["epm"].asUInt() << std::endl;
#endif
  queueSetERevsPerMRev(server_id, (uint8_t) calibrations["epm"].asUInt());
  exchange();

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Torque Constant: " << calibrations["torque"].asFloat() << std::endl;
#endif
  queueSetTorqueConstant(server_id, calibrations["torque"].asFloat());
  exchange();

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Position Offset: " << calibrations["zero"].asFloat() << std::endl;
#endif
  queueSetPositionOffset(server_id, calibrations["zero"].asFloat());
  exchange();

  if (calibrations.isMember("eac_type")) {
    std::string eac_type = calibrations["eac_type"].asString();

    if (eac_type.compare("int8") == 0) {
#ifdef DEBUG_CALIBRATION_DATA
      std::cout << "EAC calibration available" << std::endl;
#endif

      Json::Value eac_table = calibrations["eac_table"];

#ifdef DEBUG_CALIBRATION_DATA
      std::cout << eac_table << std::endl;
#endif

      if (eac_table.isArray()) {
        size_t eac_table_len = eac_table.size();

        // Copy the table values into a contiguous block of memory
        std::vector<uint8_t> eac_table_values(eac_table_len);
        for (size_t i = 0; i < eac_table_len; i++) {
          eac_table_values[i] = eac_table[(unsigned int) i].asInt();
        }

        size_t slice_len = COMM_SINGLE_SET_EAC_TABLE_LENGTH;
        for (size_t i = 0; i < eac_table_len; i += slice_len) {
          queueSetEACTable(server_id, i, &eac_table_values[i], std::min(slice_len, eac_table_len - i));
          exchange();
        }
      }

      queueSetEACOffset(server_id, calibrations["eac_offset"].asFloat());
      exchange();

      queueSetEACScale(server_id, calibrations["eac_scale"].asFloat());
      exchange();
    } else {

#ifdef DEBUG_CALIBRATION_DATA
      std::cout << "Unsupported EAC type \"" << eac_type << "\", ignoring" << std::endl;
#endif
    }
  }

  queueSetDirectCurrentControllerKp(server_id, 0.5f);
  exchange();
  queueSetDirectCurrentControllerKi(server_id, 0.001f);
  exchange();
  queueSetQuadratureCurrentControllerKp(server_id, 0.5f);
  exchange();
  queueSetQuadratureCurrentControllerKi(server_id, 0.001f);
  exchange();

#ifdef DEBUG_CALIBRATION_DATA
  std::cout << "Setting control mode" << std::endl;
#endif

  queueSetControlMode(server_id, COMM_CTRL_MODE_CURRENT);
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
  clearQueue();
#ifdef DEBUG_ALLOCS
  // This should ALWAYS be 0
  std::cout << "Number of unfreed allocations: " << allocs_ << std::endl;
#endif
}

void BLDCControllerClient::clearQueue() {
  for (auto it = packet_queue_.begin(); it != packet_queue_.end(); it++) {
    auto packet = it->second; // Get packet pointer (value in [key, value] pair)
    if (packet != nullptr) {
      delete packet;
      allocs_--;
      it->second = nullptr;
    }
  }
  //ser_.flush();
  packet_queue_.clear();
}

/* Private Flash Accessor/Mutator Members */
void BLDCControllerClient::readFlash(comm_id_t server_id, comm_full_addr_t addr, uint32_t count, std::string& buffer){
  for (size_t i = 0; i < count; i += COMM_SINGLE_READ_LENGTH) {
	size_t num_bytes = std::min(count - i, COMM_SINGLE_READ_LENGTH);
    queuePacket(server_id, new ReadFlashPacket(server_id, addr + i, num_bytes));
    allocs_++;
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
  uint8_t sync_flag = COMM_SYNC_FLAG;
  comm_protocol_t version = COMM_VERSION;
  comm_fg_t computer_flag = COMM_FG_COMP;
  tx_buf_.writeVar(COMM_SYNC_FLAG);
  tx_buf_.writeVar(COMM_VERSION);
  tx_buf_.writeVar(COMM_FG_COMP);

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
    std::cout << "Sub-packet for " << (int) it->first << ": ";
    for (unsigned char c : msg)
      printf("%02x:", c);
    std::cout << std::endl;
#endif

    comm_msg_len_t sub_msg_len = sub_packet_buf_.size();
    payload_buf_.write(reinterpret_cast<uint8_t*> (&sub_msg_len), sizeof(sub_msg_len));
    payload_buf_.addBuf(sub_packet_buf_);

    delete packet;
    allocs_--;
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

  if (errors) {
    if (errors & COMM_ERRORS_OP_FAILED) {
      throw comms_error("operation failed");
    }
    if (errors & COMM_ERRORS_MALFORMED) {
      throw comms_error("malformed request");
    }
    if (errors & COMM_ERRORS_INVALID_FC) {
      throw comms_error("invalid function code");
    }
    if (errors & COMM_ERRORS_INVALID_ARGS) {
      throw comms_error("invalid arguments");
    }
    if (errors & COMM_ERRORS_BUF_LEN_MISMATCH) {
      throw comms_error("buffer length mismatch");
    }
  }

  return true;
}

crc16_t BLDCControllerClient::computeCRC( const uint8_t* buf, size_t len ) {
  crc16_t crc = crc16_init();
  crc = crc16_update(crc, buf, len);
  return crc16_finalize(crc);
}

/*              Start of Transmission Packet Abstract Functions                 */

void BLDCControllerClient::queueLeaveBootloader(comm_id_t server_id, uint32_t jump_addr) {
  if (jump_addr == 0) {
    jump_addr = COMM_FIRMWARE_OFFSET;
  }

  Packet* packet = new JumpToAddrPacket(server_id, jump_addr);
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetControlMode(comm_id_t server_id, comm_ctrl_mode_t control_mode) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_VOL_CTRL_MODE, sizeof(control_mode), reinterpret_cast<uint8_t*> (&control_mode));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetTimeout(comm_id_t server_id, uint16_t value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_WATCHDOG, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetZeroAngle(comm_id_t server_id, uint16_t value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_REV_START, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetERevsPerMRev(comm_id_t server_id, uint8_t value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_EREVS_PER_MREV, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetInvertPhases(comm_id_t server_id, uint8_t value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_INV_PHASES, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetTorqueConstant(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_MOTOR_T, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetPositionOffset(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_POS_OFFSET, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetEACScale(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_EAC_SCALE, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetEACOffset(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_EAC_OFFSET, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetEACTable(comm_id_t server_id, size_t start_index, uint8_t *values, size_t count) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_EAC_TABLE + start_index, sizeof(*values) * count, values, count);
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetDirectCurrentControllerKp(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_DI_KP, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetDirectCurrentControllerKi(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_DI_KI, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetQuadratureCurrentControllerKp(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_QI_KP, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetQuadratureCurrentControllerKi(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_QI_KI, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetVelocityControllerKp(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_V_KP, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetVelocityControllerKi(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_V_KI, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}
void BLDCControllerClient::queueSetPositionControllerKp(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_P_KP, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetPositionControllerKi(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_CAL_P_KI, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueSetCommand(comm_id_t server_id, float value) {
  Packet* packet = new WriteRegPacket(server_id, COMM_REG_VOL_QI_COMM, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueGetRotorPosition(comm_id_t server_id) {
  // Generate Transmit Packet
  Packet* packet = new ReadRegPacket(server_id, COMM_REG_RO_ROTOR_P, 1);
  queuePacket(server_id, packet);
  allocs_++;
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
  allocs_++;
}

void BLDCControllerClient::queueSetPositionAndGetRotorPosition(comm_id_t server_id, float value) {
  // Generate Transmit Packet
  Packet* packet = new ReadWriteRegPacket (server_id,
    COMM_REG_RO_ROTOR_P, 1,                                          // Read
    COMM_REG_VOL_SETPOINT_P, sizeof(value), reinterpret_cast<uint8_t*>(&value));  // Write
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::queueGetState(comm_id_t server_id) {
  // Generate Transmit Packet
  Packet* packet = new ReadRegPacket (server_id, COMM_REG_RO_ROTOR_P, 9);
  queuePacket(server_id, packet);
  allocs_++;
}

void BLDCControllerClient::resultGetState(comm_id_t server_id, float* position, float* velocity, float* di, float* qi, float* voltage, float* temp, int32_t* acc_x, int32_t* acc_y, int32_t* acc_z) {
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
  allocs_++;
}

/*              End of Transmission Packet Abstract Functions                   */


} // namespace blue_hardware_drivers
