#include "blue_hardware_drivers/Packets.h"

#include <string>
#include <sstream>
#include <iostream>

void Packet::dump(Buffer& buf) {
  buf.write(reinterpret_cast<uint8_t*> (&server_id_), sizeof(server_id_));
  buf.write(reinterpret_cast<uint8_t*> (&func_code_), sizeof(func_code_));
}

void ReadRegPacket::dump(Buffer& buf) {
  Packet::dump(buf);
  buf.write(reinterpret_cast<uint8_t*> (&read_start_addr_), sizeof(read_start_addr_));
  buf.write(reinterpret_cast<uint8_t*> (&read_count_), sizeof(read_count_));
}

void WriteRegPacket::dump(Buffer& buf) {
  Packet::dump(buf);
  buf.write(reinterpret_cast<uint8_t*> (&write_start_addr_), sizeof(write_start_addr_));
  buf.write(reinterpret_cast<uint8_t*> (&write_count_), sizeof(write_count_));
  buf.addBuf(write_data_);
}

void ReadWriteRegPacket::dump(Buffer& buf) {
  Packet::dump(buf);
  buf.write(reinterpret_cast<uint8_t*> (&read_start_addr_), sizeof(read_start_addr_));
  buf.write(reinterpret_cast<uint8_t*> (&read_count_), sizeof(read_count_));
  buf.write(reinterpret_cast<uint8_t*> (&write_start_addr_), sizeof(write_start_addr_));
  buf.write(reinterpret_cast<uint8_t*> (&write_count_), sizeof(write_count_));
  buf.addBuf(write_data_);
}

void ReadFlashPacket::dump(Buffer& buf) {
  Packet::dump(buf);
  buf.write(reinterpret_cast<uint8_t*> (&read_start_addr_), sizeof(read_start_addr_));
  buf.write(reinterpret_cast<uint8_t*> (&read_count_), sizeof(read_count_));
}

void JumpToAddrPacket::dump(Buffer& buf) {
  Packet::dump(buf);
  buf.write(reinterpret_cast<uint8_t*> (&jump_addr_), sizeof(jump_addr_));
}


