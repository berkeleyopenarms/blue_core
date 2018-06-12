#include "blue_hardware_drivers/Packets.h"

#include <string>
#include <sstream>
#include <iostream>

std::string Packet::dump() {
  std::stringstream buffer;
  buffer.write(reinterpret_cast<char*> (&server_id_), sizeof(server_id_));
  buffer.write(reinterpret_cast<char*> (&func_code_), sizeof(func_code_));
  
  return buffer.str();
}

void Packet::parse( std::stringstream & stream ) {
  if (result_ == nullptr)
    return;

  stream.read(result_, result_size_);
}

std::string ReadRegPacket::dump() {
  std::stringstream buffer;  
  buffer << Packet::dump();
  buffer.write(reinterpret_cast<char*> (&read_start_addr_), sizeof(read_start_addr_));
  buffer.write(reinterpret_cast<char*> (&read_count_), sizeof(read_count_));
  return buffer.str();
}

std::string WriteRegPacket::dump() {
  std::stringstream buffer;
  buffer << Packet::dump();
  buffer.write(reinterpret_cast<char*> (&write_start_addr_), sizeof(write_start_addr_));
  buffer.write(reinterpret_cast<char*> (&write_count_), sizeof(write_count_));
  buffer << write_data_.str(); 
  return buffer.str();
}

std::string JumpToAddrPacket::dump() {
  std::stringstream buffer;
  buffer << Packet::dump();
  buffer.write(reinterpret_cast<char*> (&jump_addr_), sizeof(jump_addr_));

  for (unsigned char c : buffer.str())
    printf("%02x:", c);
  std::cout << std::endl;

  return buffer.str();
}

std::string ReadWriteRegPacket::dump() {
  std::stringstream buffer;
  buffer << Packet::dump();
  buffer.write(reinterpret_cast<char*> (&read_start_addr_), sizeof(read_start_addr_));
  buffer.write(reinterpret_cast<char*> (&read_count_), sizeof(read_count_));
  buffer.write(reinterpret_cast<char*> (&write_start_addr_), sizeof(write_start_addr_));
  buffer.write(reinterpret_cast<char*> (&write_count_), sizeof(write_count_));
  buffer << write_data_.str(); 
  return buffer.str();
}
