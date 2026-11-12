#pragma once
#include <stdint.h>

// Servo configuration for a fresh RatBot build, one servo at a time.
//
// Every setting follows from where a servo sits on the robot - which leg pair,
// which end of it, left or right - and never from which PCB slot it plugs
// into. That is why the table holds with the body either way up: whichever
// slot ends up nearest the nose on the left needs the same settings. With
// every horn facing outward, left servos count normally (drive mode 4) and
// right servos reversed (5), so on both sides a positive angle turns a crank
// from pointing at the nose towards pointing down, and the top of a bare horn
// moves towards the nose.
//
// SETUP writes one servo only: the single factory-new servo on the bus (ID 1),
// or the servo that already has the requested ID. It never enables torque or
// moves anything, and reads every value back.
namespace ServoSetup {
constexpr uint8_t FIRST = 11, LAST = 19, FACTORY_ID = 1;
constexpr int32_t MODEL = 1190;              // ROBOTIS XL330-M077-T.
constexpr int32_t OPERATING = 4;             // Extended position control.
constexpr int32_t BAUD_1M = 3;               // Baud Rate register value.
constexpr uint32_t FINAL_BAUD = 1000000, FACTORY_BAUD = 57600;
// IDs 11-19: front-left nose end, front-left middle, front-right nose end,
// front-right middle, hind-left middle, hind-left tail end, hind-right middle,
// hind-right tail end, head. The offset sets the window power-up readings fall
// in: [2048, 6143] for +-2048 and [4096, 8191] for +-4096, and every mounting
// target lies inside its servo's window.
constexpr int32_t DRIVES[] = {4, 4, 5, 5, 4, 4, 5, 5, 4};
constexpr int32_t OFFSETS[] = {2048, 4096, -2048, -4096, 2048, 4096, -2048, -4096, 2048};

int32_t drive(uint8_t id) { return DRIVES[id - FIRST]; }
int32_t offset(uint8_t id) { return OFFSETS[id - FIRST]; }
bool read(uint8_t item, uint8_t id, int32_t& value) { return HeadConfig::read(item, id, value); }
bool matches(uint8_t item, uint8_t id, int32_t value) { return HeadConfig::matches(item, id, value); }

bool healthy(uint8_t id) {
  int32_t voltage, temperature;
  return matches(MODEL_NUMBER, id, MODEL) && matches(HARDWARE_ERROR_STATUS, id, 0) &&
         read(PRESENT_INPUT_VOLTAGE, id, voltage) && voltage >= 35 && voltage <= 55 &&
         read(PRESENT_TEMPERATURE, id, temperature) && temperature < 45;
}

// Configured for its place, at 1 Mbps, torque off and healthy.
bool configured(uint8_t id) {
  if (id < FIRST || id > LAST) return false;
  dxl.begin(FINAL_BAUD);
  return matches(ID, id, id) && matches(BAUD_RATE, id, BAUD_1M) &&
         matches(OPERATING_MODE, id, OPERATING) && matches(DRIVE_MODE, id, drive(id)) &&
         matches(HOMING_OFFSET, id, offset(id)) && matches(TORQUE_ENABLE, id, 0) &&
         matches(BUS_WATCHDOG, id, 0) && healthy(id);
}

// Factory-new servos (ID 1) at either speed. Reports the speed of the last one.
uint8_t candidates(uint32_t& baud) {
  uint8_t found = 0;
  const uint32_t speeds[] = {FACTORY_BAUD, FINAL_BAUD};
  for (uint32_t speed : speeds) {
    dxl.begin(speed);
    if (dxl.ping(FACTORY_ID)) { ++found; baud = speed; }
  }
  dxl.begin(FINAL_BAUD);
  return found;
}

// The only settings SETUP may write, on the servo being configured.
bool put(uint8_t on, uint8_t forId, uint8_t item, int32_t value) {
  if ((on != FACTORY_ID && on != forId) ||
      !((item == TORQUE_ENABLE && value == 0) ||
        (item == OPERATING_MODE && value == OPERATING) ||
        (item == DRIVE_MODE && value == drive(forId)) ||
        (item == HOMING_OFFSET && value == offset(forId)))) return false;
  int32_t before;
  if (!read(item, on, before)) return false;
  if (before == value) return true;
  bool ack = dxl.writeControlTableItem(item, on, value, uint32_t{30});
  delay(100);
  return ack && matches(item, on, value);
}

// Goal = present, with torque off: a later torque-on holds still.
bool park(uint8_t id) {
  int32_t present;
  if (!read(PRESENT_POSITION, id, present)) return false;
  if (matches(GOAL_POSITION, id, present)) return true;
  bool ack = dxl.writeControlTableItem(static_cast<uint8_t>(GOAL_POSITION), id, present, uint32_t{30});
  delay(100);
  return ack && matches(GOAL_POSITION, id, present);
}

bool fail(uint8_t id, const char* why) {
  Serial.printf("SETUP FAIL ID%u: %s\n", id, why);
  dxl.begin(FINAL_BAUD);
  return false;
}

bool setup(uint8_t id) {
  if (id < FIRST || id > LAST) return false;
  dxl.begin(FINAL_BAUD);
  const bool existing = dxl.ping(id);
  uint8_t from = id;
  uint32_t baud = FINAL_BAUD;
  if (!existing) {
    uint8_t found = candidates(baud);
    if (found == 0) return fail(id, "no new servo (ID1) found. Plug in the servo for this place.");
    if (found > 1) return fail(id, "more than one new servo (ID1) on the bus. Unplug all but one.");
    from = FACTORY_ID;
    dxl.begin(baud);
  }
  if (!healthy(from)) return fail(id, "model, communication, voltage or temperature check failed.");
  if (!put(from, id, TORQUE_ENABLE, 0) || !put(from, id, OPERATING_MODE, OPERATING) ||
      !put(from, id, DRIVE_MODE, drive(id)) || !put(from, id, HOMING_OFFSET, offset(id))) {
    return fail(id, "a setting did not read back. Torque was not enabled; repeat SETUP.");
  }
  // A reply can be lost when the address or speed changes, so both are
  // verified at the new address and speed, and repeating SETUP recovers.
  if (from != id) {
    dxl.writeControlTableItem(static_cast<uint8_t>(ID), from, int32_t{id}, uint32_t{30});
    delay(100);
    if (!matches(ID, id, id)) return fail(id, "the new ID did not read back; repeat SETUP.");
  }
  if (baud != FINAL_BAUD) {
    dxl.writeControlTableItem(static_cast<uint8_t>(BAUD_RATE), id, BAUD_1M, uint32_t{30});
    delay(100);
  }
  if (!configured(id)) return fail(id, "final settings did not read back at 1 Mbps; repeat SETUP.");
  if (!park(id)) return fail(id, "could not park the goal at the present position.");
  int32_t present = 0;
  read(PRESENT_POSITION, id, present);
  Serial.printf("SETUP OK ID%u: %s, drive %ld, offset %+ld, position %ld ticks.\n", id,
                existing ? "existing servo checked" : "new servo configured",
                static_cast<long>(drive(id)), static_cast<long>(offset(id)), static_cast<long>(present));
  return true;
}

// Renumbering. Reused servos keep the ID of the place they had before; after
// the body is turned over, a servo can sit where another ID belongs. MOVEID
// renames one servo over the bus, via a temporary ID 21-29 (destination + 10)
// when two IDs swap, so nothing has to be unplugged. SETUP then gives it the
// settings of its new place.
constexpr uint8_t TEMP_SHIFT = 10;
bool renumberable(uint8_t id) {
  return (id >= FIRST && id <= LAST) || (id >= FIRST + TEMP_SHIFT && id <= LAST + TEMP_SHIFT);
}

bool moveFail(uint8_t from, uint8_t to, const char* why) {
  Serial.printf("MOVEID FAIL %u->%u: %s\n", from, to, why);
  return false;
}

bool moveId(uint8_t from, uint8_t to) {
  if (!renumberable(from) || !renumberable(to) || from == to) return false;
  dxl.begin(FINAL_BAUD);
  if (!dxl.ping(from)) return moveFail(from, to, "no servo answers at the old ID.");
  if (dxl.ping(to)) return moveFail(from, to, "another servo already has the new ID.");
  int32_t torque;
  if (!matches(MODEL_NUMBER, from, MODEL) || !read(TORQUE_ENABLE, from, torque)) {
    return moveFail(from, to, "model or communication check failed.");
  }
  if (torque != 0) {  // The ID is in EEPROM: torque must be off.
    dxl.writeControlTableItem(static_cast<uint8_t>(TORQUE_ENABLE), from, int32_t{0}, uint32_t{30});
    delay(100);
    if (!matches(TORQUE_ENABLE, from, 0)) return moveFail(from, to, "torque-off did not read back.");
  }
  // The reply can be lost when the ID changes; the new ID is verified instead.
  dxl.writeControlTableItem(static_cast<uint8_t>(ID), from, int32_t{to}, uint32_t{30});
  delay(100);
  if (!matches(ID, to, to) || dxl.ping(from)) return moveFail(from, to, "the new ID did not read back.");
  Serial.printf("MOVEID OK %u->%u\n", from, to);
  return true;
}

// Read-only, one line: ok = configured for its place, bad = answers but is not,
// -- = no answer at 1 Mbps; new = factory-new servos on the bus; temp = servos
// left on a temporary ID by an interrupted renumbering.
void scan() {
  char line[200];
  int used = snprintf(line, sizeof(line), "SCAN");
  for (uint8_t id = FIRST; id <= LAST; ++id) {
    dxl.begin(FINAL_BAUD);
    const char* state = !dxl.ping(id) ? "--" : configured(id) ? "ok" : "bad";
    used += snprintf(line + used, sizeof(line) - used, " %u=%s", id, state);
  }
  uint32_t baud = FINAL_BAUD;
  used += snprintf(line + used, sizeof(line) - used, " new=%u temp=", candidates(baud));
  bool any = false;
  dxl.begin(FINAL_BAUD);
  for (uint8_t id = FIRST + TEMP_SHIFT; id <= LAST + TEMP_SHIFT; ++id) {
    if (!dxl.ping(id)) continue;
    used += snprintf(line + used, sizeof(line) - used, any ? ",%u" : "%u", id);
    any = true;
  }
  if (!any) snprintf(line + used, sizeof(line) - used, "-");
  Serial.println(line);
}

// Read-only, one line of present positions in ticks; -- = no answer.
void positions() {
  char line[160];
  int used = snprintf(line, sizeof(line), "POS");
  dxl.begin(FINAL_BAUD);
  for (uint8_t id = FIRST; id <= LAST; ++id) {
    int32_t value;
    if (read(PRESENT_POSITION, id, value)) {
      used += snprintf(line + used, sizeof(line) - used, " %u=%ld", id, static_cast<long>(value));
    } else {
      used += snprintf(line + used, sizeof(line) - used, " %u=--", id);
    }
  }
  Serial.println(line);
}
} // namespace ServoSetup
