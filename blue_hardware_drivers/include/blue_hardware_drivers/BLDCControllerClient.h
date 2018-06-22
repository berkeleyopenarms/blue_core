/* Controller for Communication Protocol 
 * Protocol Version: 3
 */

#ifndef BLDCCONTROLLERCLIENT_H
#define BLDCCONTROLLERCLIENT_H

#include <serial/serial.h>
#include <iostream>
#include <string>
#include <sstream>
#include <map>
#include <exception>

#include "blue_hardware_drivers/comms_defs.h"
#include "blue_hardware_drivers/Packets.h"
#include "blue_hardware_drivers/crc16.h"
#include "blue_hardware_drivers/Buffer.h"
#include "json/json.h"

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
    void queueSetCurrentControlMode(comm_id_t server_id);
    void queueSetZeroAngle(comm_id_t server_id, uint16_t value);
    void queueSetERevsPerMRev(comm_id_t server_id, uint8_t value);
    void queueSetInvertPhases(comm_id_t server_id, uint8_t value);
    void queueSetTorqueConstant(comm_id_t server_id, float value);
    void queueSetPositionOffset(comm_id_t server_id, float value);

    // Drive Commands
    void queueSetCommand(comm_id_t server_id, float value);
    void queueGetRotorPosition(comm_id_t server_id);
    void queueSetCommandAndGetRotorPosition(comm_id_t server_id, float value);
    void queueGetState(comm_id_t server_id);
    void queueSetCommandAndGetState(comm_id_t server_id, float value);
    
    // Send queued packets and receive from boards
    void exchange();
    
    // Remove all items currently in the queue
    void clearQueue(); 

    // Result Commands
    void resultGetRotorPosition(comm_id_t server_id, float* result);
    void resultGetState(comm_id_t server_id, float* position, float* velocity, float* di, float* qi, float* voltage, float* temp, uint32_t* acc_x, uint32_t* acc_y, uint32_t* acc_z); 
  
    // Setup/Programming Commands
    void initMotor(comm_id_t server_id);

  private:
    serial::Serial ser_;
    std::map<comm_id_t, Packet*> packet_queue_;

    Buffer sub_packet_buf_;
    Buffer payload_buf_;
    Buffer tx_buf_;
    std::map<comm_id_t, Buffer> rx_bufs_;
    
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

#endif /* BLDCCONTROLLERCLIENT_H */
