// Just enough of the Arduino and Dynamixel2Arduino interfaces for src/main.cpp
// to compile on the host. tests/test_setup.cpp defines them over a simulated bus.
#pragma once
#include <cstdint>
#include <string>

namespace ControlTableItem {
enum : uint8_t {
  ID, BAUD_RATE, DRIVE_MODE, OPERATING_MODE, HOMING_OFFSET, TORQUE_ENABLE,
  PROFILE_VELOCITY, PROFILE_ACCELERATION, GOAL_POSITION
};
}

void delay(unsigned long ms);

struct HostSerial {
  std::string output;
  void begin(unsigned long) {}
  void print(const char* text) { output += text; }
  void println(const char* text) { output += std::string(text) + '\n'; }
  void println(int value) { output += std::to_string(value) + '\n'; }
  void printf(const char* format, ...);
};
extern HostSerial Serial;

struct HardwareSerial {
  explicit HardwareSerial(int) {}
};

struct Dynamixel2Arduino {
  Dynamixel2Arduino(HardwareSerial&, int) {}
  void setPortProtocolVersion(float) {}
  void begin(uint32_t baud);
  bool ping(uint8_t id);
  bool writeControlTableItem(uint8_t item, uint8_t id, int32_t value, uint32_t timeout = 100);
  bool torqueOn(uint8_t id);
};
