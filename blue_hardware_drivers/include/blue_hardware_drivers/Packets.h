#ifndef PACKETS_H
#define PACKETS_H

#include <sstream>
#include <stdint.h>

#include "blue_hardware_drivers/comms_defs.h"
#include "blue_hardware_drivers/Buffer.h"

namespace blue_hardware_drivers {

class Packet {
  private:
    uint8_t server_id_;
    comm_fc_t func_code_;
  public:
    Packet (comm_id_t id, comm_fc_t fc) : server_id_( id ), func_code_ (fc){}

    virtual void dump(Buffer& buf);
};

/* Virtual Register Mutator/Accessor Packets */
class ReadRegPacket : public Packet {
  private:
    comm_addr_t read_start_addr_;
    uint8_t read_count_;
  public:
    ReadRegPacket (comm_id_t id, comm_addr_t addr, uint8_t count) :
      Packet( id, COMM_FC_REG_READ ), read_start_addr_( addr ), read_count_( count ){}

    void dump(Buffer& buf);
};

class WriteRegPacket : public Packet {
  private:
    comm_addr_t write_start_addr_;
    uint8_t write_count_;
    Buffer write_data_;
    uint8_t num_reg_;
  public:
    WriteRegPacket (comm_id_t id, comm_addr_t addr, uint8_t count, uint8_t* data, uint8_t num_reg = 1) :
      Packet( id, COMM_FC_REG_WRITE ), write_start_addr_( addr ), write_count_( count ), num_reg_( num_reg)
    {
      write_data_.init(write_count_);
      write_data_.write(data, write_count_);
    }

    void dump(Buffer& buf);
};

class ReadWriteRegPacket : public Packet {
  private:
    comm_addr_t read_start_addr_;
    uint8_t read_count_;
    comm_addr_t write_start_addr_;
    uint8_t write_count_;
    Buffer write_data_;
    uint8_t num_reg_;
  public:
    ReadWriteRegPacket (comm_id_t id, comm_addr_t r_addr, uint8_t r_count,
                                 comm_addr_t w_addr, uint8_t w_count, uint8_t* data, uint8_t num_reg = 1) :
                    Packet( id, COMM_FC_REG_READ_WRITE ),
                        read_start_addr_( r_addr ), read_count_( r_count ),
                        write_start_addr_( w_addr ), write_count_( w_count ), num_reg_( num_reg)
    {
      write_data_.init(write_count_);
      write_data_.write(data, write_count_);
    }

    void dump(Buffer& buf);
};
/* End of Virtual Register Mutator/Accessor Packets */

/* Flash Mutator/Accessor Packets */
class ReadFlashPacket : public Packet {
  private:
    comm_full_addr_t read_start_addr_;
    uint32_t read_count_;
  public:
    ReadFlashPacket (comm_id_t id, comm_full_addr_t addr, uint32_t count) :
      Packet( id, COMM_FC_FLASH_READ ), read_start_addr_( addr ), read_count_( count ){}

    void dump(Buffer& buf);
};

/* Move Instruction Pointer (Use for switching out of bootloader) */
class JumpToAddrPacket : public Packet {
  private:
    comm_full_addr_t jump_addr_;
  public:
    JumpToAddrPacket (comm_id_t id, comm_full_addr_t jump_addr) :
      Packet( id, COMM_FC_JUMP_TO_ADDR ), jump_addr_(jump_addr) {}

    void dump(Buffer& buf);
};

} // namespace blue_hardware_drivers

#endif
