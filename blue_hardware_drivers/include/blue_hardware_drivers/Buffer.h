/* File: Buffer.h
 * Class meant to simplify adding data to a tx/rx buffer on embedded systems.
 * Operates similarly to stream operators.
 */

#ifndef BUFFER_H
#define BUFFER_H

#include <cstring>
#include <iostream>
#include "stdint.h"

namespace blue_hardware_drivers {

class Buffer {
  public:
    ~Buffer() {
      delete [] buf_;
    }

    void init(size_t len); 

    // Operators for loading in data
    template <typename T>
    Buffer& operator<<(const T val);
    // Operators for unloading data
    template <typename T>
    Buffer& operator>>(T& val);

    void read (uint8_t* data, size_t len);
    void write(const uint8_t* data, size_t len);

    template <typename T> void writeVar(const T &data); 

    void addBuf(Buffer& buf);

    void clear();

    size_t size();
    // If adding to the buffer via a pointer make sure to increment
    // the head accordingly. Otherwise there will be seg faults.
    // Add head first (will return false if not enough space).
    uint8_t* ptr();
    bool addHead(size_t len);

    std::string str();
    std::string remain_str();

  private:
    size_t max_len_;
    size_t head_;
    size_t tail_;
    uint8_t* buf_;
};

} // namespace blue_hardware_drivers

#endif
