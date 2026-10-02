// Host-side checks of the head servo handling in q8Dynamixel, on a simulated bus.
// c++ -std=c++17 -Iinclude -Itests/stubs tests/test_head.cpp -o /tmp/ratbot-head-test && /tmp/ratbot-head-test
#include <cassert>
#include "Arduino.h"
#include "../src/q8Dynamixel.cpp"

MAX1704X FuelGauge;

const char* LEGS = "45.879,134.121,45.879,134.121,45.879,134.121,45.879,134.121";

// The controller hands parseData a mutable, null-terminated copy.
uint8_t send(q8Dynamixel& q8, const std::string& packet) {
  char buffer[100];
  assert(packet.size() < sizeof(buffer));
  strcpy(buffer, packet.c_str());
  return q8.parseData(buffer);
}

std::string legs(const char* tail) { return std::string(LEGS) + tail; }

int32_t ticks(float deg) { return static_cast<int>(deg / (360.0 / 4096.0) + 0.5) + 4096; }

int32_t duration(int32_t distance) { return (distance * 1000 + 1023) / 1024; }

std::vector<Dynamixel2Arduino::Write> writesTo(Dynamixel2Arduino& dxl, uint8_t id) {
  std::vector<Dynamixel2Arduino::Write> found;
  for (auto& w : dxl.log) if (w.id == id) found.push_back(w);
  return found;
}

int legGoalWrites(Dynamixel2Arduino& dxl) {
  int count = 0;
  for (auto& w : dxl.log) if (w.item == GOAL_POSITION && w.id >= 11 && w.id <= 18) ++count;
  return count;
}

void addLegs(Dynamixel2Arduino& dxl) {
  for (uint8_t id = 11; id <= 18; ++id) dxl.add(id);
}

int main() {
  Dynamixel2Arduino dxl;
  addLegs(dxl);
  // After power-up the head sits 10 deg right with a stale goal register.
  dxl.add(19, 3983, 1000);
  q8Dynamixel q8(dxl);
  q8.begin();

  // Torque-on parks the head where it is, then centres it no faster than 90 deg/s.
  dxl.log.clear();
  assert(send(q8, "0,0,0,0,0,0,0,0,0,0,1,0.0") == 0);
  assert(dxl.headOffsetAtTorqueOn == 0);
  assert(dxl.reg(19, GOAL_POSITION) == 4096);
  assert(dxl.reg(19, PROFILE_VELOCITY) == duration(4096 - 3983));
  assert(legGoalWrites(dxl) == 0);  // A torque change still sends no leg targets.

  // Re-sending the leg targets with each head step leaves the legs where they are.
  assert(send(q8, legs(",0,1000,1,0.0")) == 0);
  int32_t legGoal = dxl.reg(11, GOAL_POSITION);
  dxl.log.clear();
  assert(send(q8, legs(",0,1000,1,3.6")) == 0);
  assert(dxl.reg(19, GOAL_POSITION) == ticks(3.6));
  assert(dxl.reg(19, PROFILE_VELOCITY) == duration(ticks(3.6) - 4096));
  assert(dxl.reg(11, GOAL_POSITION) == legGoal);
  // Equal steps keep the profile, so only the goal is written.
  dxl.log.clear();
  assert(send(q8, legs(",0,1000,1,7.2")) == 0);
  auto head = writesTo(dxl, 19);
  assert(head.size() == 1 && head[0].item == GOAL_POSITION && head[0].value == ticks(7.2));
  // An unchanged angle writes nothing to the head.
  dxl.log.clear();
  assert(send(q8, legs(",0,1000,1,7.2")) == 0);
  assert(writesTo(dxl, 19).empty());

  // 90 deg either side is the limit, and a long swing takes proportionally longer.
  assert(send(q8, legs(",0,1000,1,135")) == 0);
  assert(dxl.reg(19, GOAL_POSITION) == ticks(90) && ticks(90) == 5120);
  assert(send(q8, legs(",0,1000,1,-400")) == 0);
  assert(dxl.reg(19, GOAL_POSITION) == ticks(-90));
  assert(dxl.reg(19, PROFILE_VELOCITY) == duration(5120 - ticks(-90)));
  assert(dxl.reg(19, PROFILE_VELOCITY) >= 1990);

  // Packets without the 12th field, from an older GUI, never touch the head.
  dxl.log.clear();
  assert(send(q8, legs(",0,1000,1")) == 0);
  assert(writesTo(dxl, 19).empty());
  assert(legGoalWrites(dxl) == 8);

  // With torque off the head field is ignored.
  dxl.log.clear();
  assert(send(q8, legs(",0,1000,0,30")) == 0);
  for (auto& w : writesTo(dxl, 19)) assert(w.item != GOAL_POSITION);
  assert(dxl.reg(19, TORQUE_ENABLE) == 0);

  // Turned by hand while limp: the next torque-on parks it at the new position.
  dxl.reg(19, PRESENT_POSITION) = 4500;
  assert(send(q8, "0,0,0,0,0,0,0,0,0,0,1,0.0") == 0);
  assert(dxl.headOffsetAtTorqueOn == 0);
  assert(dxl.reg(19, GOAL_POSITION) == 4096);
  assert(dxl.reg(19, PROFILE_VELOCITY) == duration(4500 - 4096));

  // A goal is never sent on an unconfirmed profile.
  dxl.failing = {{19, PROFILE_VELOCITY}};
  dxl.log.clear();
  assert(send(q8, legs(",0,1000,1,45")) == 0);
  for (auto& w : writesTo(dxl, 19)) assert(w.item != GOAL_POSITION);
  dxl.failing.clear();
  assert(send(q8, legs(",0,1000,1,45")) == 0);
  assert(dxl.reg(19, GOAL_POSITION) == ticks(45));
  assert(dxl.reg(19, PROFILE_VELOCITY) == duration(ticks(45) - 4096));

  // A head that answers but cannot be parked stays limp; the legs still get torque.
  assert(send(q8, legs(",0,1000,0")) == 0);
  dxl.failing = {{19, GOAL_POSITION}};
  dxl.headOffsetAtTorqueOn = 12345;
  assert(send(q8, "0,0,0,0,0,0,0,0,0,0,1,30.0") == 0);
  assert(dxl.reg(19, TORQUE_ENABLE) == 0 && dxl.headOffsetAtTorqueOn == 12345);
  for (uint8_t id = 11; id <= 18; ++id) assert(dxl.reg(id, TORQUE_ENABLE) == 1);
  dxl.failing.clear();
  dxl.log.clear();
  assert(send(q8, legs(",0,1000,1,30.0")) == 0);
  for (auto& w : writesTo(dxl, 19)) assert(w.item != GOAL_POSITION);
  // The next torque-on parks it and it follows again.
  assert(send(q8, legs(",0,1000,0")) == 0);
  assert(send(q8, "0,0,0,0,0,0,0,0,0,0,1,30.0") == 0);
  assert(dxl.headOffsetAtTorqueOn == 0 && dxl.reg(19, GOAL_POSITION) == ticks(30));
  // Not a number is ignored.
  dxl.log.clear();
  assert(send(q8, legs(",0,1000,1,nan")) == 0);
  assert(writesTo(dxl, 19).empty());

  // Recovery reboots the head as well and parks it again before torque-on.
  dxl.reg(19, HARDWARE_ERROR_STATUS) = 0x20;
  dxl.log.clear();
  Serial.output.clear();
  assert(send(q8, "0,0,0,0,0,0,0,0,5,0,1,0.0") == 0);
  assert(Serial.output.find("Servo 19 hardware error 0x20: overload") != std::string::npos);
  bool rebooted = false;
  for (auto& w : writesTo(dxl, 19)) rebooted |= w.item == REBOOT;
  assert(rebooted);
  assert(dxl.headOffsetAtTorqueOn == 0);
  assert(dxl.reg(19, TORQUE_ENABLE) == 1);

  // A robot without a head runs unchanged and reports no head faults.
  Dynamixel2Arduino bare;
  addLegs(bare);
  q8Dynamixel legsOnly(bare);
  legsOnly.begin();
  Serial.output.clear();
  assert(send(legsOnly, "0,0,0,0,0,0,0,0,0,0,1,0.0") == 0);
  assert(send(legsOnly, legs(",0,1000,1,45")) == 0);
  assert(legGoalWrites(bare) == 8);
  assert(send(legsOnly, "0,0,0,0,0,0,0,0,5,0,1,0.0") == 0);
  for (auto& w : bare.log) assert(w.id != 19 || w.item != REBOOT);
  for (auto& w : bare.log) assert(w.id != 19 || w.item != GOAL_POSITION);
  assert(Serial.output.find("Servo 19") == std::string::npos);

  std::puts("Head checks passed: parked before torque-on, 90 deg limit, 90 deg/s profile, "
            "legs untouched, old packets, torque off, recovery, no head.");
  return 0;
}
