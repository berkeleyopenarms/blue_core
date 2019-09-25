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
#include <chrono>

#include "blue_hardware_drivers/comms_defs.h"
#include "blue_hardware_drivers/Packets.h"
#include "blue_hardware_drivers/crc16.h"
#include "blue_hardware_drivers/Buffer.h"

namespace blue_hardware_drivers {

BLDCControllerClient::BLDCControllerClient() {
}

BLDCControllerClient::BLDCControllerClient(std::string port, const std::vector<comm_id_t>& boards) {
  init(port, boards);
  allocs_ = 0;
}

void BLDCControllerClient::init(std::string port, const std::vector<comm_id_t>& boards) {
  // First, manually set the port to low latency mode
  int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  struct serial_struct ser;
  ioctl(fd, TIOCGSERIAL, &ser);
  ser.flags |= ASYNC_LOW_LATENCY;
  ioctl(fd, TIOCSSERIAL, &ser);
  close(fd);

  // Then set up serial library
  ser_.setPort(port);
  ser_.setBaudrate(COMM_DEFAULT_BAUD_RATE);
  ser_.setTimeout(serial::Timeout::max(), 1, 1, 1, 1);
  ser_.open();

  tx_buf_.init(COMM_MAX_BUF);
  sub_packet_buf_.init(COMM_MAX_BUF);
  payload_buf_.init(COMM_MAX_BUF);

  for (auto id : boards) {
    rx_bufs_[id].init(COMM_MAX_BUF);
    board_flags_[id];
  }
}

// Sends packets to boards and collects data
void BLDCControllerClient::exchange() {

  typedef std::chrono::high_resolution_clock Time;
  using fsec = std::chrono::duration<double>;
  static auto last_successful_transmission = Time::now();

  std::string err = "";

  transmit();
  // Receive data from each board in order
  for (auto it = packet_queue_.begin(); it != packet_queue_.end(); it++) {
    comm_id_t board_id = it->first; // Get server id (key in [key, value] pair)
    if (board_id != 0) {
      bool success = false;
      while (!success)
      {
        success = receive(board_id, err);
        if (err != "")
        {
          std::string elapsed = std::to_string(fsec(Time::now() - last_successful_transmission).count());
          throw comms_error("Board " + std::to_string((int)board_id) + ": " + err + 
                            "\n\t\t\t\t\tTime since last successful comms: " + elapsed);
        }
      }
    }
  }

  last_successful_transmission = Time::now();
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
  ser_.flush();
  packet_queue_.clear();
}

/* Private Flash Accessor/Mutator Members */
void BLDCControllerClient::readFlash(comm_id_t board_id, comm_full_addr_t addr, uint32_t count, std::string& buffer){
  for (size_t i = 0; i < count; i += COMM_SINGLE_READ_LENGTH) {
	size_t num_bytes = std::min(count - i, COMM_SINGLE_READ_LENGTH);
    queuePacket(board_id, new ReadFlashPacket(board_id, addr + i, num_bytes));
    exchange(); // This can error which means when running flash commands make sure to try/catch comm_error!
    buffer.append(rx_bufs_[board_id].remain_str());
  }
}

/* Private Transmission Members */
void BLDCControllerClient::queuePacket(comm_id_t board_id, Packet* packet) {
  if (packet_queue_[board_id] != nullptr) {
    std::string error = "A packet is already queued for motor " + std::to_string((int) board_id);
    throw comms_error(error);
  }
  packet_queue_[board_id] = packet;
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

#ifdef DEBUG_TRANSMIT
  std::cout << std::endl << "TRANSMITTING" << std::endl;
#endif

  // Generate Payload
  payload_buf_.clear();
  for (auto it = packet_queue_.begin(); it != packet_queue_.end(); it++) {
    sub_packet_buf_.clear();
    // Acquire packet byte format
    auto packet = it->second; // Get packet pointer (value in [key, value] pair)
#ifdef DEBUG_TRANSMIT
    if (packet == NULL) {
      std::cerr << "Generating packet for board: " << (int) it->first << " is NULL" << std::endl;
      assert(0);
    }
#endif

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

void BLDCControllerClient::ser_read_check(uint8_t * data, size_t len, std::string & err) {
  int read_len = 0;
  size_t tries = 0;

  typedef std::chrono::high_resolution_clock Time;
  using fsec = std::chrono::duration<double>;

  auto start = Time::now();
  do {
    read_len = ser_.read(data, len);
  } while (read_len == 0 && read_len != -1 && tries++ < COMM_MAX_RETRIES);

  if (read_len == -1) {
    err = "Serial Port Closed";
  } 
  else if (read_len != len) {
    auto elapsed = Time::now() - start;
    err = "Expected " + std::to_string(len) + " Byte(s) got " + std::to_string(read_len) + ". Waited for: " + std::to_string(fsec(elapsed).count()) + "s.";
  }
}

bool BLDCControllerClient::receive( comm_id_t board_id, std::string & err ) {

  uint8_t sync = 0;
  ser_read_check(&sync, sizeof(sync), err);
  if (sync != COMM_SYNC_FLAG) {
    return false;
  }

#ifdef DEBUG_RECEIVE
  std::cout << std::endl << "RECEIVING for BID: " << (int) board_id << std::endl;
#endif

  comm_protocol_t protocol_version = 0;
  ser_read_check(reinterpret_cast<uint8_t*>(&protocol_version), sizeof(protocol_version), err);
  if (protocol_version != COMM_VERSION) {
    return false;
  }

  comm_fg_t flags = 0xFF;
  ser_read_check(reinterpret_cast<uint8_t*>(&flags), sizeof(flags), err);
  if ((flags & COMM_FG_BOARD) != COMM_FG_BOARD) {
    return false;
  }

  // Check if there was a watchdog reset
  if (flags & COMM_FG_RESET) {
    board_flags_[board_id].reset = true;
  } else {
    board_flags_[board_id].reset = false;
  }
  if (flags & COMM_FG_TIMEOUT) {
    board_flags_[board_id].timeout = true;
  } else {
    board_flags_[board_id].timeout = false;
  }



  comm_msg_len_t packet_len = 0;
  ser_read_check(reinterpret_cast<uint8_t*>(&packet_len), sizeof(packet_len), err);

#ifdef DEBUG_RECEIVE
  std::cout << "Packet length: " << (int) packet_len << std::endl;
#endif

  rx_bufs_[board_id].clear();

  if (!rx_bufs_[board_id].addHead(packet_len)) {
    err = "rx buffer too small for message";
    return false;
  }
  ser_read_check(rx_bufs_[board_id].ptr(), packet_len, err);

#ifdef DEBUG_RECEIVE
  // print message
  std::string message = rx_bufs_[board_id].str();
  std::cout << "Receiving: ";
  for (unsigned char c : message)
    printf("%02x:", c);
  std::cout << std::endl;
#endif

  crc16_t crc = 0;
  ser_read_check(reinterpret_cast<uint8_t*>(&crc), 2, err);

  crc16_t computed_crc = computeCRC(rx_bufs_[board_id].ptr(), rx_bufs_[board_id].size());

#ifdef DEBUG_RECEIVE_CRC
  // Print the two CRCs
  std::cout << "Received CRC: " << std::hex << crc << std::endl;

  std::cout << "Calculated CRC: " << std::hex << computed_crc << std::endl;
#endif

  if (computed_crc != crc) {
    err = "Incorrect crc!";
    return false;
  }

  // Parse message
  comm_msg_len_t sub_msg_len = 0;
  rx_bufs_[board_id].read(reinterpret_cast<uint8_t*>(&sub_msg_len), sizeof(sub_msg_len));

  comm_id_t id = 0;
  rx_bufs_[board_id].read(reinterpret_cast<uint8_t*>(&id), sizeof(id));
  if (!(id == board_id || id == 0)) {
    err = "Incorrect server id, got: " + std::to_string((int)id);
    return false;
  }

#ifdef DEBUG_RECEIVE
  std::cout << "Board ID: " << (int) id << std::endl;
#endif

  comm_fc_t func_code = 0;
  rx_bufs_[board_id].read(reinterpret_cast<uint8_t*>(&func_code), sizeof(func_code));

  comm_errors_t errors = 0;
  rx_bufs_[board_id].read(reinterpret_cast<uint8_t*>(&errors), sizeof(errors));

  if (errors) {
    if (errors & COMM_ERRORS_OP_FAILED) {
      err = "operation failed";
      return false;
    }
    if (errors & COMM_ERRORS_MALFORMED) {
      err = "malformed request";
      return false;
    }
    if (errors & COMM_ERRORS_INVALID_FC) {
      err = "invalid function code";
      return false;
    }
    if (errors & COMM_ERRORS_INVALID_ARGS) {
      err = "invalid arguments";
      return false;
    }
    if (errors & COMM_ERRORS_BUF_LEN_MISMATCH) {
      err = "buffer length mismatch";
      return false;
    }
  }

  return true;
}

crc16_t BLDCControllerClient::computeCRC( const uint8_t* buf, size_t len ) {
  crc16_t crc = crc16_init();
  crc = crc16_update(crc, buf, len);
  return crc16_finalize(crc);
}

bool BLDCControllerClient::checkWDGRST(comm_id_t board_id) {
  return board_flags_[board_id].reset;
}

bool BLDCControllerClient::checkTimeout(comm_id_t board_id) {
  return board_flags_[board_id].timeout;
}

void BLDCControllerClient::resetBoards() {
  Packet* packet = new ResetPacket(0);
  queuePacket(0, packet);
  exchange();
}

void BLDCControllerClient::resetBuffer() {
  ser_.flush();
}

/*              Start of Transmission Packet Abstract Functions                 */

void BLDCControllerClient::queueConfirmID(comm_id_t board_id) {
  Packet* packet = new ConfirmIDPacket(board_id);
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueEnumerate(comm_id_t board_id) {
  Packet* packet = new EnumeratePacket(board_id);
  queuePacket(board_id, packet);
}

void BLDCControllerClient::getEnumerateResponse(comm_id_t board_id, comm_id_t* response_id) {
  rx_bufs_[board_id].read(reinterpret_cast<uint8_t*>(response_id), sizeof(*response_id));
}

void BLDCControllerClient::queueLeaveBootloader(comm_id_t board_id, uint32_t jump_addr) {
  if (jump_addr == 0) {
    jump_addr = COMM_FIRMWARE_OFFSET;
  }

  Packet* packet = new JumpToAddrPacket(board_id, jump_addr);
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueClearWDGRST(comm_id_t board_id) {
  Packet* packet = new ClearWDGRSTPacket(board_id);
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetControlMode(comm_id_t board_id, comm_ctrl_mode_t control_mode) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_VOL_CTRL_MODE, sizeof(control_mode), reinterpret_cast<uint8_t*> (&control_mode));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetTimeout(comm_id_t board_id, uint16_t value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_WATCHDOG, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetZeroAngle(comm_id_t board_id, uint16_t value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_REV_START, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetERevsPerMRev(comm_id_t board_id, uint8_t value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_EREVS_PER_MREV, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetInvertPhases(comm_id_t board_id, uint8_t value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_INV_PHASES, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetTorqueConstant(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_MOTOR_T, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetPositionOffset(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_POS_OFFSET, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetEACScale(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_EAC_SCALE, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetEACOffset(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_EAC_OFFSET, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetEACTable(comm_id_t board_id, size_t start_index, uint8_t *values, size_t count) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_EAC_TABLE + start_index, sizeof(*values) * count, values, count);
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetDirectCurrentControllerKp(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_DI_KP, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetDirectCurrentControllerKi(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_DI_KI, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetQuadratureCurrentControllerKp(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_QI_KP, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetQuadratureCurrentControllerKi(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_QI_KI, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetVelocityControllerKp(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_V_KP, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetVelocityControllerKi(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_V_KI, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}
void BLDCControllerClient::queueSetPositionControllerKp(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_P_KP, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetPositionControllerKi(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_P_KI, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetIAOffset(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_IA_OFF, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}
void BLDCControllerClient::queueSetIBOffset(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_IA_OFF, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}
void BLDCControllerClient::queueSetICOffset(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_CAL_IA_OFF, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetCommand(comm_id_t board_id, float value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_VOL_QI_COMM, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetPosCommand(comm_id_t board_id, float position, float feedforward) {
  const static size_t num_registers = 2; // Number of registers in this write 
  float values[num_registers] = {position, feedforward};
  Packet* packet = new WriteRegPacket(
                        board_id, 
                        COMM_REG_VOL_SETPOINT_P, 
                        sizeof(position) + sizeof(feedforward), 
                        reinterpret_cast<uint8_t*> (&values), 
                        num_registers);
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueGetRotorPosition(comm_id_t board_id) {
  // Generate Transmit Packet
  Packet* packet = new ReadRegPacket(board_id, COMM_REG_RO_ROTOR_P, 1);
  queuePacket(board_id, packet);
}

void BLDCControllerClient::resultGetRotorPosition(comm_id_t board_id, float* position) {
  rx_bufs_[board_id].read(reinterpret_cast<uint8_t*>(position), sizeof(*position));
}

void BLDCControllerClient::queueSetCommandAndGetRotorPosition(comm_id_t board_id, float value) {
  // Generate Transmit Packet
  Packet* packet = new ReadWriteRegPacket (board_id,
    COMM_REG_RO_ROTOR_P, 1,                                                    // Read
    COMM_REG_VOL_QI_COMM, sizeof(value), reinterpret_cast<uint8_t*>(&value));  // Write
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetPositionAndGetRotorPosition(comm_id_t board_id, float value) {
  // Generate Transmit Packet
  Packet* packet = new ReadWriteRegPacket (board_id,
    COMM_REG_RO_ROTOR_P, 1,                                                       // Read
    COMM_REG_VOL_SETPOINT_P, sizeof(value), reinterpret_cast<uint8_t*>(&value));  // Write
  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueGetState(comm_id_t board_id) {
  // Generate Transmit Packet
  Packet* packet = new ReadRegPacket (board_id, COMM_REG_RO_ROTOR_P, 9);
  queuePacket(board_id, packet);
}

void BLDCControllerClient::resultGetState(comm_id_t board_id, float* position, float* velocity, float* di, float* qi, float* voltage, float* temp, int32_t* acc_x, int32_t* acc_y, int32_t* acc_z) {
  rx_bufs_[board_id].read(reinterpret_cast<uint8_t*>(position), sizeof(*position));
  rx_bufs_[board_id].read(reinterpret_cast<uint8_t*>(velocity), sizeof(*velocity));
  rx_bufs_[board_id].read(reinterpret_cast<uint8_t*>(di), sizeof(*di));
  rx_bufs_[board_id].read(reinterpret_cast<uint8_t*>(qi), sizeof(*qi));
  rx_bufs_[board_id].read(reinterpret_cast<uint8_t*>(voltage), sizeof(*voltage));
  rx_bufs_[board_id].read(reinterpret_cast<uint8_t*>(temp), sizeof(*temp));
  rx_bufs_[board_id].read(reinterpret_cast<uint8_t*>(acc_x), sizeof(*acc_x));
  rx_bufs_[board_id].read(reinterpret_cast<uint8_t*>(acc_y), sizeof(*acc_y));
  rx_bufs_[board_id].read(reinterpret_cast<uint8_t*>(acc_z), sizeof(*acc_z));
}

void BLDCControllerClient::queueSetCommandAndGetState(comm_id_t board_id, float value) {
  // Generate Transmit Packet
  Packet* packet = new ReadWriteRegPacket (board_id,
    COMM_REG_RO_ROTOR_P, 9,                                                  // Read
    COMM_REG_VOL_QI_COMM, sizeof(value), reinterpret_cast<uint8_t*>(&value)  // Write
  );

  queuePacket(board_id, packet);
}

void BLDCControllerClient::queueSetPosCommandAndGetState(comm_id_t board_id, float position, float feedforward) {
  const static size_t num_registers = 2; // Number of registers in this write
  float values[num_registers] = {position, feedforward};

  Packet* packet = new ReadWriteRegPacket(board_id,
    COMM_REG_RO_ROTOR_P, 9,                                          // Read
    COMM_REG_VOL_SETPOINT_P, sizeof(position) + sizeof(feedforward), // Write
              reinterpret_cast<uint8_t*> (&values), num_registers
  );

  queuePacket(board_id, packet);
}


void BLDCControllerClient::queueSetRevolutions(comm_id_t board_id, int16_t value) {
  Packet* packet = new WriteRegPacket(board_id, COMM_REG_RO_ROTOR_REVS, sizeof(value), reinterpret_cast<uint8_t*> (&value));
  queuePacket(board_id, packet);
}

/*              End of Transmission Packet Abstract Functions                   */


} // namespace blue_hardware_drivers
