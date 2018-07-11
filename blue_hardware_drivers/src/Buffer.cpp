#include "blue_hardware_drivers/Buffer.h"

#include <cstring>
#include <string>
#include <exception>

#include "stdint.h"

void Buffer::init(size_t len) {
  max_len_ = len;
  head_ = 0, tail_ = 0;
  buf_ = new uint8_t[max_len_];
}

template <typename T>
Buffer& Buffer::operator<<(const T val) {
  if (head_ + sizeof(val) > max_len_)
    throw std::exception();

  for (int i = 0; i < sizeof(T); i++) 
    buf_[head_++] = (uint8_t)(val >> (8 * i)); // Shifting for each byte

  return *this;
}

template <typename T>
Buffer& Buffer::operator>>(T& val) {
  if (head_ - sizeof(val) < 0)
    throw std::exception();

  for (int i = 0; i < sizeof(T); i++) 
    val = val | (((T) buf_[tail_++]) << (8 * i));

  return *this; 
}

void Buffer::write(const uint8_t * data, size_t len) {
  if (head_ + len > max_len_)
    throw std::exception();

  for (int i = 0; i < len; i++) {
    buf_[head_++] = data[i]; // Little Endian 
  }
}
template<typename T>
void Buffer::writeVar(const T &data) {
  if (head_ + sizeof(data) > max_len_)
    throw std::exception();

  const uint8_t* data_ptr = reinterpret_cast<const uint8_t*> (&data);
  for (int i = 0; i < sizeof(data); i++) {
    buf_[head_++] = data_ptr[i]; // Little Endian 
  }
}

template void Buffer::writeVar<uint8_t>(const uint8_t&);

void Buffer::read(uint8_t* data, size_t len) {
  if (head_ - len < 0)
    throw std::exception();

  for (int i = 0; i < len; i++) {
    data[i] = buf_[tail_++];
  }
}

void Buffer::addBuf(Buffer& buf) {
  if (head_ + buf.size() > max_len_)
    throw std::exception();
  
  std::memcpy(buf_ + head_, buf.ptr(), buf.size()); 
  head_ += buf.size();
}

void Buffer::clear() {
  head_ = 0;
  tail_ = 0;
}

uint8_t* Buffer::ptr() {
  return buf_;
}

bool Buffer::addHead(size_t len) {
  if (head_ + len > max_len_)
    return false; 
  head_ += len;
  return true;
}

size_t Buffer::size() {
  return head_;
}

std::string Buffer::str() {
  std::string msg = "";
  for (size_t i = 0; i < head_; i++) {
    msg += (char) buf_[i];
  }
  return msg;
}

std::string Buffer::remain_str() {
std::string msg = "";
  for (size_t i = tail_; i < head_; i++) {
    msg += (char) buf_[i];
  }
  return msg;
}

/*
// Test code expected to print out:
// d
// 10000
// 100000000
#include <iostream>
int main () {
  Buffer rx;
  rx.init(1024);
  rx << (uint8_t) 100 << (uint16_t) 10000 << (uint32_t) 100000000;
  uint8_t a = 0;
  uint16_t b = 0;
  uint32_t c = 0;
  rx >> a >> b >> c;
  std::cout << a << std::endl << b << std::endl << c << std::endl;

  rx.clear();

  std::cout << "Writing Data" << std::endl;
  rx.write( reinterpret_cast<uint8_t*> (&a), sizeof(a) );
  rx.write( reinterpret_cast<uint8_t*> (&b), sizeof(b) );
  rx.write( reinterpret_cast<uint8_t*> (&c), sizeof(c) );

  rx.addBuf(rx);

  std::cout << "Reading Data" << std::endl;
  rx.read ( reinterpret_cast<uint8_t*> (&a), sizeof(a) );
  rx.read ( reinterpret_cast<uint8_t*> (&b), sizeof(b) );
  rx.read ( reinterpret_cast<uint8_t*> (&c), sizeof(c) );

  std::cout << a << std::endl << b << std::endl << c << std::endl;

  rx.read ( reinterpret_cast<uint8_t*> (&a), sizeof(a) );
  rx.read ( reinterpret_cast<uint8_t*> (&b), sizeof(b) );
  rx.read ( reinterpret_cast<uint8_t*> (&c), sizeof(c) );

  std::cout << a << std::endl << b << std::endl << c << std::endl;
}
*/
