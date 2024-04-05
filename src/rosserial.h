#pragma once

#include <ros/node_handle.h>
#include "robotArm.h"

namespace ros {

class Serial2Hardware {
 public:
  Serial2Hardware() {
    iostream = &ROSserial;
    baud_ = 38400;
  }

  void setBaud(long baud) { this->baud_ = baud; }

  int getBaud() { return baud_; }

  void init() { iostream->begin(baud_); }

  int read() { return iostream->read(); };

  void write(uint8_t* data, int length) {
    for (int i = 0; i < length; i++) {
      iostream->write(data[i]);
    }
  }

  void flush() {
    iostream->flush();
  }
  unsigned long time() { return millis(); }

 protected:
  HardwareSerial* iostream;
  long baud_;
};

typedef NodeHandle_<Serial2Hardware, 5, 5, 512, 512> MyNodeHandle;

}  // namespace ros
