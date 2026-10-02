// Host-side run of the first-setup program, src/main.cpp, with a simulated bus.
// Nine factory-new servos are attached one at a time in ID order, the head last.
// c++ -std=c++17 -Itests/stubs tests/test_setup.cpp -o /tmp/ratbot-setup-test && /tmp/ratbot-setup-test
#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <map>
#include "Arduino.h"

using namespace ControlTableItem;

struct Servo {
  uint32_t baud = 57600;
  std::map<int, int32_t> values = {{ID, 1}, {BAUD_RATE, 1}, {DRIVE_MODE, 0},
    {OPERATING_MODE, 3}, {HOMING_OFFSET, 0}, {TORQUE_ENABLE, 0},
    {PROFILE_VELOCITY, 0}, {PROFILE_ACCELERATION, 0}, {GOAL_POSITION, 2048}};
};
std::map<int, Servo> bus;   // Keyed by attachment order, not by ID.
uint32_t busBaud = 0;
struct Halted {};           // finishSetup() ends in an infinite loop.

HostSerial Serial;
void HostSerial::printf(const char* format, ...) {
  char text[256];
  va_list args;
  va_start(args, format);
  vsnprintf(text, sizeof(text), format, args);
  va_end(args);
  output += text;
}

void delay(unsigned long) {
  if (Serial.output.find("When finished, switch the robot off") != std::string::npos) throw Halted{};
}

Servo* find(uint8_t id) {
  Servo* found = nullptr;
  for (auto& entry : bus) {
    Servo& s = entry.second;
    if (s.baud == busBaud && s.values[ID] == id) {
      assert(found == nullptr && "two servos answer at the same ID");
      found = &s;
    }
  }
  return found;
}

void Dynamixel2Arduino::begin(uint32_t baud) { busBaud = baud; }
bool Dynamixel2Arduino::ping(uint8_t id) { return find(id) != nullptr; }
bool Dynamixel2Arduino::torqueOn(uint8_t id) {
  assert(id == 254);
  for (auto& entry : bus)
    if (entry.second.baud == busBaud) entry.second.values[TORQUE_ENABLE] = 1;
  return true;
}
bool Dynamixel2Arduino::writeControlTableItem(uint8_t item, uint8_t id, int32_t value, uint32_t) {
  Servo* s = find(id);
  if (!s) return false;
  // EEPROM is only writable with torque off.
  if (item == ID || item == BAUD_RATE || item == DRIVE_MODE || item == OPERATING_MODE ||
      item == HOMING_OFFSET) assert(s->values[TORQUE_ENABLE] == 0);
  s->values[item] = value;
  if (item == BAUD_RATE) s->baud = value == 3 ? 1000000 : 57600;
  if (item == GOAL_POSITION) assert(s->values[TORQUE_ENABLE] == 1);
  return true;
}

#include "../src/main.cpp"

void restart() {
  Serial.output.clear();
  count = 0;
  setup();
}

// Runs loop() until the program halts, attaching the next new servo whenever
// the previous one has left factory ID 1. Returns the loop count.
int run(int attachUpTo) {
  int loops = 0;
  try {
    for (; loops < 200; ++loops) {
      bool waiting = true;
      for (auto& entry : bus) if (entry.second.values[ID] == 1) waiting = false;
      if (waiting && static_cast<int>(bus.size()) < attachUpTo) bus[bus.size()] = Servo{};
      loop();
    }
  } catch (const Halted&) {
    return loops;
  }
  return -1;
}

void checkConfigured() {
  const int32_t drive[9] = {4, 4, 5, 5, 4, 4, 5, 5, 4};
  const int32_t offset[9] = {2048, 4096, -2048, -4096, 2048, 4096, -2048, -4096, 2048};
  const int32_t goal[9] = {4618, 5622, 4618, 5622, 4618, 5622, 4618, 5622, 4096};
  assert(bus.size() == 9);
  for (int i = 0; i < 9; ++i) {
    Servo& s = bus.at(i);
    assert(s.values[ID] == 11 + i);
    assert(s.baud == 1000000 && s.values[BAUD_RATE] == 3);
    assert(s.values[OPERATING_MODE] == 4);
    assert(s.values[DRIVE_MODE] == drive[i]);
    assert(s.values[HOMING_OFFSET] == offset[i]);
    assert(s.values[TORQUE_ENABLE] == 1);
    assert(s.values[PROFILE_VELOCITY] == 1000);
    assert(s.values[GOAL_POSITION] == goal[i]);
  }
}

int main() {
  // The eight leg servos alone do not finish the setup: it waits for the head.
  restart();
  assert(run(8) == -1);
  assert(bus.size() == 8);
  assert(Serial.output.find("Motor setup complete") == std::string::npos);
  assert(Serial.output.find("No new Dynamixel motors found.") != std::string::npos);
  for (auto& entry : bus) assert(entry.second.values[TORQUE_ENABLE] == 0);

  // Attaching the head as the ninth servo makes it ID 19, centred and held.
  assert(run(9) > 0);
  checkConfigured();
  assert(Serial.output.find("Changing ID to 19") != std::string::npos);
  assert(Serial.output.find("Motor ID 19 connection successful.") != std::string::npos);
  assert(Serial.output.find("ID 19 (head) is centred and held.") != std::string::npos);

  // Running the setup again on the finished robot finds all nine and re-centres.
  for (auto& entry : bus) {
    entry.second.values[TORQUE_ENABLE] = 0;
    entry.second.values[GOAL_POSITION] = 0;
  }
  restart();
  assert(run(9) > 0);
  checkConfigured();
  assert(Serial.output.find("Joint 19 already set up.") != std::string::npos);
  assert(Serial.output.find("New motor connected!") == std::string::npos);

  std::puts("Setup checks passed: nine servos in ID order, waits for the head, ID19 centred, rerun.");
  return 0;
}
