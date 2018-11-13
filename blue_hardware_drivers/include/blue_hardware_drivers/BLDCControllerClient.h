/* Controller for Communication Protocol
 * Protocol Version: 3
 */

#ifndef BLDCCONTROLLERCLIENT_H
#define BLDCCONTROLLERCLIENT_H

#include <serial/serial.h>
#include <iostream>
#include <string>
#include <sstream>
#include <unordered_map>
#include <exception>

#include "blue_hardware_drivers/comms_defs.h"
#include "blue_hardware_drivers/Packets.h"
#include "blue_hardware_drivers/crc16.h"
#include "blue_hardware_drivers/Buffer.h"
#include "json/json.h"

namespace blue_hardware_drivers {

class BLDCControllerClient {
  public:
    BLDCControllerClient(); //requires init afterwards
    BLDCControllerClient(std::string port, const std::vector<comm_id_t>& boards);

    void init(std::string port, const std::vector<comm_id_t>& boards);

    void queuePacket(comm_id_t server_id, Packet* packet);

    // Program Counter Adjustments
    void queueLeaveBootloader(comm_id_t server_id, uint32_t jump_addr);

    // Calibration Setup
    void queueSetTimeout(comm_id_t server_id, uint16_t value);
    void queueSetControlMode(comm_id_t server_id, comm_ctrl_mode_t control_mode);
    void queueSetZeroAngle(comm_id_t server_id, uint16_t value);
    void queueSetERevsPerMRev(comm_id_t server_id, uint8_t value);
    void queueSetInvertPhases(comm_id_t server_id, uint8_t value);
    void queueSetTorqueConstant(comm_id_t server_id, float value);
    void queueSetPositionOffset(comm_id_t server_id, float value);
    void queueSetEACScale(comm_id_t server_id, float value);
    void queueSetEACOffset(comm_id_t server_id, float value);
    void queueSetEACTable(comm_id_t server_id, size_t start_index, uint8_t *values, size_t count);
    void queueSetDirectCurrentControllerKp(comm_id_t server_id, float value);
    void queueSetDirectCurrentControllerKi(comm_id_t server_id, float value);
    void queueSetQuadratureCurrentControllerKp(comm_id_t server_id, float value);
    void queueSetQuadratureCurrentControllerKi(comm_id_t server_id, float value);
    void queueSetVelocityControllerKp(comm_id_t server_id, float value);
    void queueSetVelocityControllerKi(comm_id_t server_id, float value);
    void queueSetPositionControllerKp(comm_id_t server_id, float value);
    void queueSetPositionControllerKi(comm_id_t server_id, float value);
    // Drive Commands
    void queueSetCommand(comm_id_t server_id, float value);
    void queueGetRotorPosition(comm_id_t server_id);
    void queueSetCommandAndGetRotorPosition(comm_id_t server_id, float value);
    void queueSetPositionAndGetRotorPosition(comm_id_t server_id, float value);
    void queueGetState(comm_id_t server_id);
    void queueSetCommandAndGetState(comm_id_t server_id, float value);

    // Send queued packets and receive from boards
    void exchange();

    // Remove all items currently in the queue
    void clearQueue();

    // Result Commands
    void resultGetRotorPosition(comm_id_t server_id, float* result);
    void resultGetState(comm_id_t server_id, float* position, float* velocity, float* di, float* qi, float* voltage, float* temp, int32_t* acc_x, int32_t* acc_y, int32_t* acc_z);

    // Setup/Programming Commands
    void initMotor(comm_id_t server_id);

  private:
    serial::Serial ser_;
    std::unordered_map<comm_id_t, Packet*> packet_queue_;

    Buffer sub_packet_buf_;
    Buffer payload_buf_;
    Buffer tx_buf_;
    std::unordered_map<comm_id_t, Buffer> rx_bufs_;

    int allocs_ = 0;

    // Checks to make sure serial read succeeded
    void ser_read_check(uint8_t * data, size_t len);

    void transmit();
    bool receive( comm_id_t server_id );

    // Flash Commands (Only affects one board at a time)
    void readFlash( comm_id_t server_id, comm_full_addr_t addr, uint32_t count, std::string& buffer );

    crc16_t computeCRC( const uint8_t* buf, size_t len );
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

} // namespace blue_hardware_drivers

#endif /* BLDCCONTROLLERCLIENT_H */
