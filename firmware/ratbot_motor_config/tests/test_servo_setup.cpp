// Host-side checks of servo setup, the identification wiggle and the
// read-only SCAN/POS lines, with a simulated bus.
// c++ -std=c++17 -Wall -Wextra tests/test_servo_setup.cpp -o /tmp/q8-setup-test && /tmp/q8-setup-test
#define Q8_SETUP_TEST
#include "test_squro_assembly.cpp"

namespace S = ServoSetup;
const int32_t DRIVE[] = {4, 4, 5, 5, 4, 4, 5, 5, 4};
const int32_t OFFSET[] = {2048, 4096, -2048, -4096, 2048, 4096, -2048, -4096, 2048};

// A bus with only factory-new servos at ID 1 (none, one, or one per speed).
void bare(std::vector<uint32_t> speeds) {
  reset();
  dxl.devices.clear();
  for (uint32_t speed : speeds) {
    Device d; d.baud = speed; d.values[BAUD_RATE] = speed == 1000000 ? 3 : 1;
    d.values[PRESENT_POSITION] = 2500; d.values[GOAL_POSITION] = 77;
    dxl.devices[1] = d;
  }
  dxl.writes.clear(); Serial.output.clear();
}
bool has(const char* text) { return Serial.output.find(text) != std::string::npos; }
// SETUP of id wrote nothing but configuration of ID1/id, and never torque-on.
void setupScope(uint8_t id) {
  for (auto w : dxl.writes) {
    assert(w.id == 1 || w.id == id);
    assert((w.item == TORQUE_ENABLE && w.value == 0) ||
           (w.item == OPERATING_MODE && w.value == 4) ||
           (w.item == DRIVE_MODE && w.value == DRIVE[id - 11]) ||
           (w.item == HOMING_OFFSET && w.value == OFFSET[id - 11]) ||
           (w.item == ID && w.id == 1 && w.value == id) ||
           (w.item == BAUD_RATE && w.id == id && w.value == 3) ||
           (w.item == GOAL_POSITION && w.id == id));
  }
}
void configuredAs(uint8_t id) {
  const Device& d = dxl.devices.at(id);
  assert(d.baud == 1000000 && d.values.at(ID) == id && d.values.at(BAUD_RATE) == 3);
  assert(d.values.at(OPERATING_MODE) == 4 && d.values.at(DRIVE_MODE) == DRIVE[id - 11]);
  assert(d.values.at(HOMING_OFFSET) == OFFSET[id - 11] && d.values.at(TORQUE_ENABLE) == 0);
  assert(d.values.at(GOAL_POSITION) == d.values.at(PRESENT_POSITION));  // Parked.
  assert(!dxl.devices.count(1));
}

int main() {
  // Left servos count normally, right ones reversed; the offsets put every
  // mounting target inside the window its servo reads in after power-up.
  for (uint8_t id = 11; id <= 19; ++id) {
    assert(S::drive(id) == DRIVE[id - 11] && S::offset(id) == OFFSET[id - 11]);
    const bool left = id == 11 || id == 12 || id == 15 || id == 16;
    assert(S::drive(id) == (id == 19 || left ? 4 : 5));
    const int32_t low = S::offset(id) < 0 ? -S::offset(id) : S::offset(id);
    const bool rear = id >= 15 && id <= 18, first = id % 2 == 1;
    const int32_t target = id == 19 ? A::HEAD_TARGET :
        (rear ? A::REAR_TARGETS[first ? 0 : 1] : A::FRONT_TARGETS[first ? 0 : 1]);
    assert(target >= low && target <= low + 4095);
  }

  // A factory-new servo, at either speed, becomes any of IDs 11-19.
  for (uint8_t id = 11; id <= 19; ++id) for (uint32_t speed : {57600u, 1000000u}) {
    bare({speed}); assert(S::setup(id)); configuredAs(id); setupScope(id);
    assert(has("SETUP OK") && has("new servo configured"));
    for (auto w : dxl.writes) assert(!(w.item == TORQUE_ENABLE && w.value != 0));
    // Repeating is harmless: the servo is checked, nothing is written.
    size_t count = dxl.writes.size(); Serial.output.clear();
    assert(S::setup(id)); assert(dxl.writes.size() == count && has("existing servo checked"));
  }
  // Configured servos already on the bus are never touched.
  prepare(); dxl.devices.erase(14); dxl.devices[1] = Device{};
  assert(S::setup(14)); configuredAs(14); setupScope(14);
  // Lost acknowledgments of the ID and speed changes are recovered by readback.
  bare({57600}); dxl.lostChangeAck = true; assert(S::setup(16)); configuredAs(16);

  // A reused servo with the right ID but stale settings and torque on is fixed,
  // torque off first.
  prepare(); dxl.writes.clear();
  dxl.devices[13].values[DRIVE_MODE] = 4; dxl.devices[13].values[HOMING_OFFSET] = 0;
  dxl.devices[13].values[OPERATING_MODE] = 3; dxl.devices[13].values[TORQUE_ENABLE] = 1;
  assert(S::setup(13)); setupScope(13); assert(dxl.writes.front().item == TORQUE_ENABLE);
  assert(dxl.devices[13].values[DRIVE_MODE] == 5 && dxl.devices[13].values[HOMING_OFFSET] == -2048);
  assert(dxl.devices[13].values[TORQUE_ENABLE] == 0);
  assert(dxl.devices[13].values[GOAL_POSITION] == dxl.devices[13].values[PRESENT_POSITION]);

  // Refusals write nothing: no new servo, two new servos, or an unhealthy one.
  bare({}); assert(!S::setup(11) && dxl.writes.empty() && has("no new servo"));
  bare({57600}); dxl.twoNew = true;  // A second factory servo answers at the other speed.
  assert(!S::setup(11) && dxl.writes.empty() && has("more than one new servo"));
  for (auto invalid : std::vector<std::pair<int,int>>{{MODEL_NUMBER,1200},
      {HARDWARE_ERROR_STATUS,32}, {PRESENT_INPUT_VOLTAGE,34},
      {PRESENT_INPUT_VOLTAGE,56}, {PRESENT_TEMPERATURE,45}}) {
    bare({57600}); dxl.devices[1].values[invalid.first] = invalid.second;
    assert(!S::setup(12) && dxl.writes.empty() && has("SETUP FAIL ID12"));
  }
  // A write that does not read back stops before the ID changes.
  for (int item : {OPERATING_MODE, DRIVE_MODE, HOMING_OFFSET}) {
    bare({57600}); dxl.failedItem = item;
    assert(!S::setup(17)); setupScope(17); assert(!dxl.devices.count(17));
  }
  bare({57600}); dxl.failedItem = ID; assert(!S::setup(17)); assert(!dxl.devices.count(17));
  // put() refuses anything outside the servo being configured and its values.
  bare({57600});
  assert(!S::put(1, 11, DRIVE_MODE, 5) && !S::put(1, 11, HOMING_OFFSET, 4096));
  assert(!S::put(1, 11, TORQUE_ENABLE, 1) && !S::put(12, 11, DRIVE_MODE, 4));
  assert(!S::put(1, 11, GOAL_POSITION, 0) && dxl.writes.empty());

  // SCAN and POS are read-only, one line each.
  prepare(); dxl.devices[12].values[DRIVE_MODE] = 5; dxl.devices.erase(13);
  dxl.devices[1] = Device{};
  command("SCAN\n");
  assert(has("SCAN 11=ok 12=bad 13=-- 14=ok 15=ok 16=ok 17=ok 18=ok 19=ok new=1 temp=-\n"));
  Serial.output.clear(); command("POS\n");
  assert(has("POS 11=5336 12=5812 13=-- 14=5236 15=3991 16=6688 17=4201 18=5844 19=3983\n"));
  assert(dxl.writes.empty());

  // The parser takes exactly one ID 11-19 after SETUP / WIGGLE.
  for (const char* bad : {"SETUP\n", "SETUP 1\n", "SETUP 10\n", "SETUP 20\n", "SETUP 110\n",
                          "SETUP11\n", "SETUP  11\n", "WIGGLE 1x\n", "WIGGLE 9\n"}) {
    bare({57600}); command(bad); assert(dxl.writes.empty() && has("Commands:"));
  }
  bare({57600}); command("SETUP 18\n"); configuredAs(18);

  // WIGGLE: +256 ticks and back on that servo only, capped, then released and
  // restored. Its place and direction are what the person watching confirms.
  for (uint8_t id = 11; id <= 19; ++id) {
    prepare(); dxl.follow = true;
    const int32_t start = dxl.devices[id].values[PRESENT_POSITION];
    command(("WIGGLE " + std::to_string(id) + "\n").c_str());
    assert(has(("WIGGLE DONE ID" + std::to_string(id)).c_str()));
    bool parked = false, out = false, back = false;
    for (auto w : dxl.writes) {
      assert(w.id == id);
      assert(w.item == TORQUE_ENABLE || w.item == GOAL_PWM || w.item == PROFILE_VELOCITY ||
             w.item == PROFILE_ACCELERATION || w.item == GOAL_POSITION);
      if (w.item == GOAL_PWM && w.value != 885) assert(w.value == A::WIGGLE_PWM);
      if (w.item == GOAL_POSITION) {
        if (w.value == start && !out) parked = true;
        else if (w.value == start + A::WIGGLE_TICKS) { assert(parked); out = true; }
        else { assert(w.value == start && out); back = true; }
      }
      if (w.item == TORQUE_ENABLE && w.value == 1) assert(parked);
    }
    assert(out && back);
    assert(dxl.devices[id].values[PRESENT_POSITION] == start);
    assert(!A::active && !A::pendingRestore && !A::blocked && A::count == 0);
    assert(dxl.devices[id].values[TORQUE_ENABLE] == 0 && dxl.devices[id].values[GOAL_PWM] == 885);
    assert(dxl.devices[id].values[PROFILE_VELOCITY] == 0);
  }
  // A horn that cannot move, or draws too much current, fails and releases,
  // and says why.
  prepare(); assert(!A::wiggle(15)); assert(has("WIGGLE FAIL ID15"));
  assert(has("WIGGLE ID15 stopped: goal not reached in time. pos=3991 goal=4247 start=3991"));
  assert(dxl.devices[15].values[TORQUE_ENABLE] == 0 && dxl.devices[15].values[GOAL_PWM] == 885);
  prepare(); dxl.follow = true; dxl.devices[16].values[PRESENT_CURRENT] = 251;
  assert(!A::wiggle(16)); assert(dxl.devices[16].values[TORQUE_ENABLE] == 0);
  prepare(); dxl.follow = true; dxl.devices[19].values[PRESENT_CURRENT] = 151;
  assert(!A::wiggle(19)); assert(dxl.devices[19].values[TORQUE_ENABLE] == 0);
  // Never on a servo that is not configured for its place, or mid-alignment.
  prepare(); dxl.devices[12].values[HOMING_OFFSET] = 0;
  assert(!A::wiggle(12) && dxl.writes.empty());
  prepare(); dxl.follow = true; command("ALIGNFL\n"); assert(A::active);
  size_t count = dxl.writes.size();
  command("WIGGLE 15\nSETUP 15\nSCAN\nPOS\n");
  assert(dxl.writes.size() == count && A::active && A::motors[0].id == 11);
  assert(!A::wiggle(15));
  command("RELEASE\n"); off(11);
  // A failed release blocks setup and wiggling until it is cleared.
  prepare(); A::begin(11); dxl.failedItem = TORQUE_ENABLE; A::release(); assert(A::blocked);
  dxl.failedItem = -1; count = dxl.writes.size();
  command("SETUP 15\nWIGGLE 15\n"); assert(dxl.writes.size() == count);
  assert(has("SETUP REFUSED") && has("WIGGLE REFUSED"));
  A::release(); off(11);

  // MOVEID: reused servos swapped left/right by turning the body over are
  // renumbered via a temporary ID, writing nothing but the ID.
  prepare();
  command("MOVEID 13 23\n");
  assert(has("MOVEID OK 13->23") && dxl.devices.count(23) && !dxl.devices.count(13));
  for (auto w : dxl.writes) assert(w.id == 13 && w.item == ID && w.value == 23);
  Serial.output.clear(); command("SCAN\n"); assert(has(" 13=-- ") && has(" temp=23\n"));
  command("MOVEID 11 13\nMOVEID 23 11\n");
  Serial.output.clear(); command("SCAN\n");
  assert(has("11=bad 12=ok 13=bad") && has(" temp=-\n"));  // Each carries its old place's settings...
  command("SETUP 11\nSETUP 13\n");
  Serial.output.clear(); command("SCAN\n");
  assert(has("SCAN 11=ok 12=ok 13=ok 14=ok 15=ok 16=ok 17=ok 18=ok 19=ok new=0 temp=-\n"));  // ...until SETUP.
  // The servo keeps its position; only its name changed.
  assert(dxl.devices[11].values[PRESENT_POSITION] == 3770 && dxl.devices[13].values[PRESENT_POSITION] == 5336);
  // A servo with torque on is switched off first; the ID is EEPROM.
  prepare(); dxl.devices[16].values[TORQUE_ENABLE] = 1;
  assert(S::moveId(16, 26)); assert(dxl.writes.front().item == TORQUE_ENABLE && dxl.writes.front().value == 0);
  // Never onto a taken ID or from a missing one; malformed commands send nothing.
  prepare(); assert(!S::moveId(11, 12) && dxl.writes.empty() && has("MOVEID FAIL 11->12"));
  prepare(); assert(!S::moveId(21, 11) && dxl.writes.empty() && has("no servo answers"));
  prepare(); dxl.devices[11].values[MODEL_NUMBER] = 1200; assert(!S::moveId(11, 21) && dxl.writes.empty());
  for (const char* bad : {"MOVEID 11 20\n", "MOVEID 10 21\n", "MOVEID 11 11\n", "MOVEID 31 11\n",
                          "MOVEID 11  21\n", "MOVEID 1121\n", "MOVEID 11 2\n"}) {
    prepare(); command(bad); assert(dxl.writes.empty() && has("Commands:"));
  }
  prepare(); A::begin(11); assert(A::active);
  size_t before = dxl.writes.size(); command("MOVEID 13 23\n"); assert(dxl.writes.size() == before);
  A::release();
  prepare(); A::begin(11); dxl.failedItem = TORQUE_ENABLE; A::release(); dxl.failedItem = -1;
  before = dxl.writes.size(); command("MOVEID 13 23\n");
  assert(dxl.writes.size() == before && has("MOVEID REFUSED")); A::release();

  // Releasing everything mid-renumbering: servos parked on temporary IDs are
  // switched off too, and the IDs they left empty are not a failure.
  prepare(); command("MOVEID 14 24\nMOVEID 15 25\n");
  dxl.devices[24].values[TORQUE_ENABLE] = 1;
  Serial.output.clear(); command("!");
  assert(!A::blocked && !has("NOT CONFIRMED") && has("restored"));
  assert(dxl.devices[24].values[TORQUE_ENABLE] == 0);
  prepare(); command("MOVEID 16 26\n"); dxl.devices[26].values[TORQUE_ENABLE] = 1;
  A::startupRelease(); assert(!A::blocked && dxl.devices[26].values[TORQUE_ENABLE] == 0);
  for (auto w : dxl.writes) assert(w.item == ID || (w.item == TORQUE_ENABLE && w.value == 0));

  puts("Servo setup checks passed: new and reused servos for IDs 11-19, scoped readback writes, "
       "refusals, SCAN/POS, parser, the identification wiggle with its guards, and renumbering.");
}
