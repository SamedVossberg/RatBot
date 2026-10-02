// Host stand-ins for the Arduino core, Dynamixel2Arduino and MAX1704X, so that
// src/q8Dynamixel.cpp compiles and runs against a simulated servo bus.
#pragma once
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

inline uint32_t fakeMillis = 0;
inline uint32_t millis() { return fakeMillis; }
inline void delay(unsigned long ms) { fakeMillis += ms; }

struct HostSerial {
  std::string output;
  void print(const char* text) { output += text; }
  void println(const char* text) { output += std::string(text) + '\n'; }
  void println(int value) { output += std::to_string(value) + '\n'; }
  void printf(const char* format, ...) {
    char text[256];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    output += text;
  }
};
inline HostSerial Serial;

namespace ControlTableItem {
enum : uint8_t {
  OPERATING_MODE, TORQUE_ENABLE, HARDWARE_ERROR_STATUS, POSITION_P_GAIN,
  PROFILE_ACCELERATION, PROFILE_VELOCITY, GOAL_POSITION, PRESENT_POSITION
};
}
const uint8_t OP_EXTENDED_POSITION = 4;
const uint8_t REBOOT = 0xFE;  // Logged in place of an item for reboot().

namespace DYNAMIXEL {
struct Packet { uint8_t* p_buf = nullptr; uint16_t buf_capacity = 0; bool is_completed = false; };
struct XELInfoBulkRead_t { uint8_t id; uint16_t addr, addr_length; uint8_t* p_recv_buf; };
struct InfoBulkReadInst_t { Packet packet; XELInfoBulkRead_t* p_xels; uint8_t xel_count; bool is_info_changed; };
struct XELInfoBulkWrite_t { uint8_t id; uint16_t addr, addr_length; uint8_t* p_data; };
struct InfoBulkWriteInst_t { Packet packet; XELInfoBulkWrite_t* p_xels; uint8_t xel_count; bool is_info_changed; };
struct XELInfoSyncRead_t { uint8_t id; uint8_t* p_recv_buf; };
struct InfoSyncReadInst_t {
  Packet packet; uint16_t addr, addr_length; XELInfoSyncRead_t* p_xels; uint8_t xel_count; bool is_info_changed;
};
struct XELInfoSyncWrite_t { uint8_t id; uint8_t* p_data; };
struct InfoSyncWriteInst_t {
  Packet packet; uint16_t addr, addr_length; XELInfoSyncWrite_t* p_xels; uint8_t xel_count; bool is_info_changed;
};
}  // namespace DYNAMIXEL

// The bus: each present servo is a register map. Every write is logged, bulk
// writes as one GOAL_POSITION entry per servo, torque broadcasts as ID 254.
struct Dynamixel2Arduino {
  struct Write { uint8_t item, id; int32_t value; };
  std::map<uint8_t, std::map<int, int32_t>> servos;
  std::vector<Write> log;
  std::vector<std::pair<uint8_t, int>> failing;  // (id, item) writes that fail
  int lastError = 0;
  int32_t headOffsetAtTorqueOn = 0;  // Goal minus present of ID 19 at torque-on

  void add(uint8_t id, int32_t present = 4096, int32_t goal = 4096) {
    servos[id] = {{ControlTableItem::OPERATING_MODE, 4}, {ControlTableItem::TORQUE_ENABLE, 0},
                  {ControlTableItem::HARDWARE_ERROR_STATUS, 0},
                  {ControlTableItem::PROFILE_VELOCITY, 0}, {ControlTableItem::PROFILE_ACCELERATION, 0},
                  {ControlTableItem::GOAL_POSITION, goal}, {ControlTableItem::PRESENT_POSITION, present}};
  }
  int32_t& reg(uint8_t id, int item) { return servos.at(id)[item]; }
  bool fails(uint8_t id, int item) const {
    for (auto& f : failing) if (f.first == id && f.second == item) return true;
    return false;
  }

  void begin(uint32_t) {}
  void setPortProtocolVersion(float) {}
  bool ping(uint8_t id) { return servos.count(id); }
  bool setOperatingMode(uint8_t id, uint8_t mode) {
    return writeControlTableItem(ControlTableItem::OPERATING_MODE, id, mode);
  }
  void torque(uint8_t id, int32_t on) {
    log.push_back({ControlTableItem::TORQUE_ENABLE, id, on});
    for (auto& s : servos) {
      if (id != 254 && s.first != id) continue;
      if (on && s.first == 19)
        headOffsetAtTorqueOn = s.second[ControlTableItem::GOAL_POSITION] -
                               s.second[ControlTableItem::PRESENT_POSITION];
      s.second[ControlTableItem::TORQUE_ENABLE] = on;
    }
  }
  bool torqueOn(uint8_t id) { torque(id, 1); return true; }
  bool torqueOff(uint8_t id) { torque(id, 0); return true; }
  bool writeControlTableItem(uint8_t item, uint8_t id, int32_t value, uint32_t = 100) {
    log.push_back({item, id, value});
    lastError = !servos.count(id) || fails(id, item) ? 3 : 0;
    if (lastError) return false;
    servos[id][item] = value;
    return true;
  }
  int32_t readControlTableItem(uint8_t item, uint8_t id, uint32_t = 100) {
    lastError = servos.count(id) ? 0 : 3;
    return lastError ? 0 : servos[id][item];
  }
  int getLastLibErrCode() const { return lastError; }
  uint8_t getLastStatusPacketError() const { return 0; }
  bool reboot(uint8_t id, uint32_t) {
    log.push_back({REBOOT, id, 0});
    if (!servos.count(id)) return false;
    // RAM comes back at defaults that need not match the present position.
    servos[id][ControlTableItem::TORQUE_ENABLE] = 0;
    servos[id][ControlTableItem::GOAL_POSITION] = 0;
    servos[id][ControlTableItem::PROFILE_VELOCITY] = 0;
    return true;
  }
  bool bulkWrite(DYNAMIXEL::InfoBulkWriteInst_t* info) {
    for (uint8_t i = 0; i < info->xel_count; ++i) {
      int32_t goal;
      memcpy(&goal, info->p_xels[i].p_data, sizeof(goal));
      writeControlTableItem(ControlTableItem::GOAL_POSITION, info->p_xels[i].id, goal);
    }
    return true;
  }
  int fastSyncRead(DYNAMIXEL::InfoSyncReadInst_t* info) { return info->xel_count; }
};

struct MAX1704X {
  float voltage() { return 7400; }
  float percent() { return 80; }
};
