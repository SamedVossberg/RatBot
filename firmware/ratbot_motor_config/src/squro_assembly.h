#pragma once

// Commanded bench alignment, one detached leg pair or the head at a time, and
// a short identification wiggle of one servo. No EEPROM writes (those are
// ServoSetup's), broadcast torque or wireless input.
namespace SquroAssembly {
constexpr uint32_t MOVE_MS = 5000, HOLD_MS = 300000, KEEP_MS = 10000;
// CAD angles from the nose direction, positive downward, converted like the
// robot firmware: int(deg*4096/360+0.5)+4096. Front: elbow crank 52.660 deg,
// shoulder crank 129.021 deg. Hind: elbow -41.286 deg, shoulder +57.000 deg.
constexpr int32_t FRONT_TARGETS[] = {4695, 5564};
constexpr int32_t REAR_TARGETS[] = {3627, 4745};
constexpr int32_t HEAD_TARGET = 4096;
// Largest move from where a bare horn happens to sit; turn it by hand first.
constexpr int32_t FRONT_TRAVEL = 1536, REAR_TRAVEL = 2048, HEAD_TRAVEL = 256;
// Hold disturbance before release. 32 ticks tripped while fitting every leg
// in October 2026; screwing a crank down nudges the horn by up to ~4 deg.
constexpr int32_t HOLD_GUARD = 128;
// "Reached" means within 2 deg for 200 ms. At the 15 % head cap the servo's own
// controller stalls ~9 ticks short against the neck's friction (2026-11-11);
// a degree does not matter, as horns take cranks and the head in 90 deg steps.
constexpr int32_t SETTLE_TICKS = 23;
// Identification wiggle: +22.5 deg and back at 15 % output.
constexpr int32_t WIGGLE_TICKS = 256, WIGGLE_PWM = 133;
constexpr uint32_t WIGGLE_MS = 400;
struct Motor {
  uint8_t id = 0;
  int32_t target = 0, start = 0, cap = 0, oldPwm = 0, oldDuration = 0, oldAccel = 0;
};
Motor motors[2];
uint8_t count = 0;
bool active = false, holding = false, pendingRestore = false, blocked = false;
uint32_t began = 0, holdBegan = 0, lastKeep = 0, lastPoll = 0, lastLog = 0, settledSince = 0;

bool read(uint8_t item, uint8_t id, int32_t& value) { return HeadConfig::read(item, id, value); }
bool matches(uint8_t item, uint8_t id, int32_t value) { return HeadConfig::matches(item, id, value); }

bool torqueOff(uint8_t id) {
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    dxl.writeControlTableItem(static_cast<uint8_t>(TORQUE_ENABLE), id, int32_t{0}, uint32_t{30});
    int32_t value = dxl.readControlTableItem(static_cast<uint8_t>(TORQUE_ENABLE), id, uint32_t{30});
    // A latched hardware alert must not hide successful torque-off.
    if (dxl.getLastLibErrCode() == 0 && (dxl.getLastStatusPacketError() & 0x7f) == 0 && value == 0) return true;
  }
  Serial.printf("ASSEMBLY ID%u TORQUE-OFF NOT CONFIRMED: SWITCH BATTERY POWER OFF.\n", id);
  return false;
}

// Torque off on every servo that answers: IDs 11-19 and the temporary IDs
// 21-29 of a renumbering. A servo that does not answer is not holding.
bool releaseAll() {
  dxl.begin(HeadConfig::FINAL_BAUD);
  bool off = true;
  for (uint8_t id = 11; id <= 29; ++id) {
    if (id != 20 && dxl.ping(id)) off = torqueOff(id) && off;
  }
  return off;
}

// At MCU restart, release any powered servo left holding by the previous run.
void startupRelease() {
  if (!releaseAll()) blocked = true;
}

bool ramWrite(Motor& m, uint8_t item, int32_t value) {
  if (m.id < 11 || m.id > 19) return false;
  if (item != TORQUE_ENABLE && item != GOAL_PWM && item != PROFILE_VELOCITY &&
      item != PROFILE_ACCELERATION && item != GOAL_POSITION) return false;
  if (item == TORQUE_ENABLE && value != 0 && value != 1) return false;
  if (item == GOAL_POSITION && value != m.start && value != m.target) return false;
  if (item == GOAL_PWM && (value < 0 ||
      (value > m.cap && !matches(TORQUE_ENABLE, m.id, 0)))) return false;
  if (!dxl.writeControlTableItem(item, m.id, value, uint32_t{30})) return false;
  return matches(item, m.id, value);
}

void release() {
  active = false; holding = false;
  bool off = true;
  if (count) {
    // Attempt both even if the first fails. Restore nothing until both are off.
    // A selected servo that stops answering is not confirmed off.
    for (uint8_t i = 0; i < count; ++i) off = torqueOff(motors[i].id) && off;
  } else {
    off = releaseAll();
  }
  if (!off) { blocked = true; return; }
  bool restored = true;
  if (pendingRestore) {
    for (uint8_t i = 0; i < count; ++i) {
      Motor& m = motors[i];
      restored = ramWrite(m, PROFILE_VELOCITY, m.oldDuration) && restored;
      restored = ramWrite(m, PROFILE_ACCELERATION, m.oldAccel) && restored;
      restored = ramWrite(m, GOAL_PWM, m.oldPwm) && restored;
    }
  }
  blocked = !restored;
  if (restored) { pendingRestore = false; count = 0; }
  Serial.println(restored ? "ASSEMBLY RELEASED: torque OFF; profiles/output restored." :
                           "ASSEMBLY RELEASED: torque OFF; restoration failed. Retry RELEASE before proceeding.");
}

bool preflight() {
  for (uint8_t id = 11; id <= 19; ++id) {
    if (!ServoSetup::configured(id)) {
      Serial.printf("ASSEMBLY REFUSED: ID%u configuration, torque, communication or health check failed.\n", id);
      return false;
    }
  }
  return true;
}

void begin(uint8_t firstId) {
  if (active || pendingRestore || blocked) {
    Serial.println("ASSEMBLY REFUSED: RELEASE the current alignment first."); return;
  }
  if (firstId != 11 && firstId != 13 && firstId != 15 && firstId != 17 && firstId != 19) return;
  if (!preflight()) return;
  count = firstId == 19 ? 1 : 2;
  for (uint8_t i = 0; i < count; ++i) {
    Motor& m = motors[i];
    m = Motor{}; m.id = firstId + i;
    bool rear = firstId == 15 || firstId == 17;
    m.target = firstId == 19 ? HEAD_TARGET : (rear ? REAR_TARGETS[i] : FRONT_TARGETS[i]);
    int32_t pwmLimit;
    // The detached rear outputs can start nearly half a turn from their CAD
    // reference (ID16 measured 6688 ticks). Refuse more than one half-turn.
    const int32_t travel = firstId == 19 ? HEAD_TRAVEL : (rear ? REAR_TRAVEL : FRONT_TRAVEL);
    if (!read(PRESENT_POSITION, m.id, m.start) ||
        m.start < m.target - travel || m.start > m.target + travel ||
        !read(GOAL_PWM, m.id, m.oldPwm) || m.oldPwm <= 0 ||
        !read(PWM_LIMIT, m.id, pwmLimit) || pwmLimit <= 0 ||
        !read(PROFILE_VELOCITY, m.id, m.oldDuration) ||
        !read(PROFILE_ACCELERATION, m.id, m.oldAccel)) {
      Serial.printf("ASSEMBLY REFUSED: ID%u position or RAM settings outside checked range.\n", m.id);
      count = 0; return;
    }
    m.cap = firstId == 19 ? 133 : 266;
    if (m.oldPwm < m.cap) m.cap = m.oldPwm;
    if (pwmLimit < m.cap) m.cap = pwmLimit;
  }
  // Every setting above was captured before any RAM write or torque-on.
  pendingRestore = true;
  active = true; holding = false; settledSince = 0;
  for (uint8_t i = 0; i < count; ++i) {
    Motor& m = motors[i];
    const char* failed = !ramWrite(m, GOAL_PWM, m.cap) ? "Goal PWM" :
                         !ramWrite(m, PROFILE_VELOCITY, MOVE_MS) ? "Profile Velocity" :
                         !ramWrite(m, PROFILE_ACCELERATION, MOVE_MS / 4) ? "Profile Acceleration" :
                         !ramWrite(m, GOAL_POSITION, m.start) ? "Goal Position (park)" : nullptr;
    if (failed) {
      Serial.printf("ASSEMBLY ABORT: preparation write/readback failed: ID%u %s.\n", m.id, failed);
      release(); return;
    }
  }
  for (uint8_t i = 0; i < count; ++i) {
    Motor& m = motors[i];
    const char* failed = !ramWrite(m, TORQUE_ENABLE, 1) ? "Torque Enable" :
                         !ramWrite(m, GOAL_POSITION, m.target) ? "Goal Position" : nullptr;
    if (failed) {
      Serial.printf("ASSEMBLY ABORT: torque/goal write/readback failed: ID%u %s.\n", m.id, failed);
      release(); return;
    }
    Serial.printf("ASSEMBLY MOVING ID%u: %ld -> %ld ticks over 5 seconds, PWM cap %ld.\n",
                  m.id, static_cast<long>(m.start), static_cast<long>(m.target), static_cast<long>(m.cap));
  }
  began = lastKeep = millis(); lastPoll = began - 50; lastLog = began - 250;
}

void tick() {
  if (!active) return;
  uint32_t now = millis();
  if (now - lastKeep >= KEEP_MS || (holding && now - holdBegan >= HOLD_MS)) {
    Serial.println("ASSEMBLY STOP: host heartbeat lost or five-minute hold expired."); release(); return;
  }
  if (now - lastPoll < 50) return;
  lastPoll = now;
  bool settled = true;
  bool log = now - lastLog >= (holding ? 1000u : 250u);
  for (uint8_t i = 0; i < count; ++i) {
    Motor& m = motors[i];
    int32_t position = 0, current = 0, pwm = 0, voltage = 0, temperature = 0, torque = 0, fault = 0;
    bool ok = read(HARDWARE_ERROR_STATUS, m.id, fault) && fault == 0 &&
              read(PRESENT_POSITION, m.id, position) && read(PRESENT_CURRENT, m.id, current) &&
              read(PRESENT_PWM, m.id, pwm) && read(PRESENT_INPUT_VOLTAGE, m.id, voltage) &&
              read(PRESENT_TEMPERATURE, m.id, temperature) && read(TORQUE_ENABLE, m.id, torque);
    int32_t low = (m.start < m.target ? m.start : m.target) - 64;
    int32_t high = (m.start > m.target ? m.start : m.target) + 64;
    int32_t maxCurrent = m.id == 19 ? 150 : 250;
    int32_t error = position > m.target ? position - m.target : m.target - position;
    if (!ok || position < low || position > high || current < -maxCurrent || current > maxCurrent ||
        pwm < -m.cap || pwm > m.cap || voltage < 35 || voltage > 55 || temperature >= 45 ||
        torque != 1 || (holding && error > HOLD_GUARD)) {
      Serial.printf("ASSEMBLY ABORT: ID%u telemetry/read check failed.\n", m.id);
      release();
      Serial.printf("  read_ok=%u pos=%ld target=%ld current_mA=%ld pwm=%ld voltage_0.1V=%ld temp=%ld torque=%ld fault=%ld\n",
                    ok ? 1u : 0u, static_cast<long>(position), static_cast<long>(m.target),
                    static_cast<long>(current), static_cast<long>(pwm), static_cast<long>(voltage),
                    static_cast<long>(temperature), static_cast<long>(torque), static_cast<long>(fault));
      return;
    }
    if (error > SETTLE_TICKS) settled = false;
    if (log) Serial.printf("ASSEMBLY %s ID%u pos=%ld current_mA=%ld pwm=%ld voltage=%.1f temp=%ld\n",
                          holding ? "holding" : "moving", m.id, static_cast<long>(position),
                          static_cast<long>(current), static_cast<long>(pwm), voltage * 0.1,
                          static_cast<long>(temperature));
  }
  if (log) lastLog = now;
  if (!holding) {
    if (now - began >= MOVE_MS && settled) {
      if (settledSince == 0) settledSince = now;
      if (now - settledSince >= 200) {
        holding = true; holdBegan = now;
        Serial.println("ASSEMBLY READY: measured targets reached; holding up to 5 minutes. RELEASE or ! stops.");
      }
    } else settledSince = 0;
    if (!holding && now - began >= MOVE_MS + 3000) {
      Serial.println("ASSEMBLY ABORT: targets not reached in time."); release();
    }
  }
}

// Waits for the wiggling servo to reach goal, checking it every 50 ms.
bool waitFor(const Motor& m, int32_t goal) {
  const int32_t maxCurrent = m.id == 19 ? 150 : 250;
  int32_t position = 0, current = 0, torque = 0, fault = 0;
  const char* why = "goal not reached in time";
  for (uint32_t waited = 50; waited <= WIGGLE_MS + 800; waited += 50) {
    delay(50);
    const bool ok = read(HARDWARE_ERROR_STATUS, m.id, fault) && read(PRESENT_POSITION, m.id, position) &&
                    read(PRESENT_CURRENT, m.id, current) && read(TORQUE_ENABLE, m.id, torque);
    if (!ok) why = "read failed";
    else if (fault != 0) why = "hardware error";
    else if (torque != 1) why = "torque dropped";
    else if (current < -maxCurrent || current > maxCurrent) why = "current limit";
    else if (position < m.start - 64 || position > m.target + 64) why = "position out of range";
    else {
      const int32_t error = position > goal ? position - goal : goal - position;
      if (waited >= WIGGLE_MS && error <= 12) return true;
      continue;
    }
    break;
  }
  Serial.printf("WIGGLE ID%u stopped: %s. pos=%ld goal=%ld start=%ld current_mA=%ld torque=%ld fault=%ld\n",
                m.id, why, static_cast<long>(position), static_cast<long>(goal), static_cast<long>(m.start),
                static_cast<long>(current), static_cast<long>(torque), static_cast<long>(fault));
  return false;
}

// Identification: one configured servo turns +22.5 deg and back at 15 %
// output, then releases. The top of an outward-facing horn moves towards the
// nose first, on either side, which checks place and direction together.
// Blocks for about two seconds; start it only with the horn free.
bool wiggle(uint8_t id) {
  if (active || pendingRestore || blocked) {
    Serial.println("WIGGLE REFUSED: RELEASE the current alignment first.");
    return false;
  }
  if (id < 11 || id > 19) return false;
  if (!ServoSetup::configured(id)) {
    Serial.printf("WIGGLE FAIL ID%u: not configured for its place, or a torque/health check failed.\n", id);
    return false;
  }
  Motor& m = motors[0];
  m = Motor{}; m.id = id;
  int32_t pwmLimit;
  if (!read(PRESENT_POSITION, id, m.start) || !read(GOAL_PWM, id, m.oldPwm) || m.oldPwm <= 0 ||
      !read(PWM_LIMIT, id, pwmLimit) || pwmLimit <= 0 ||
      !read(PROFILE_VELOCITY, id, m.oldDuration) || !read(PROFILE_ACCELERATION, id, m.oldAccel)) {
    Serial.printf("WIGGLE FAIL ID%u: could not read its settings.\n", id);
    return false;
  }
  m.target = m.start + WIGGLE_TICKS;
  m.cap = WIGGLE_PWM;
  if (m.oldPwm < m.cap) m.cap = m.oldPwm;
  if (pwmLimit < m.cap) m.cap = pwmLimit;
  count = 1;
  pendingRestore = true;
  Serial.printf("WIGGLE ID%u: +22.5 deg and back.\n", id);
  const bool moved = ramWrite(m, GOAL_PWM, m.cap) && ramWrite(m, PROFILE_VELOCITY, WIGGLE_MS) &&
                     ramWrite(m, PROFILE_ACCELERATION, WIGGLE_MS / 4) &&
                     ramWrite(m, GOAL_POSITION, m.start) && ramWrite(m, TORQUE_ENABLE, 1) &&
                     ramWrite(m, GOAL_POSITION, m.target) && waitFor(m, m.target) &&
                     ramWrite(m, GOAL_POSITION, m.start) && waitFor(m, m.start);
  release();
  const bool ok = moved && !blocked;
  if (ok) Serial.printf("WIGGLE DONE ID%u\n", id);
  else Serial.printf("WIGGLE FAIL ID%u: the move did not complete; check that the horn turns freely.\n", id);
  return ok;
}

// "SETUP 11" -> 11; anything but a prefix and an ID 11-19 -> 0.
uint8_t idArgument(const char* command, const char* prefix) {
  const size_t n = strlen(prefix);
  if (strncmp(command, prefix, n) != 0 || strlen(command) != n + 2) return 0;
  const char tens = command[n], ones = command[n + 1];
  if (tens != '1' || ones < '1' || ones > '9') return 0;
  return static_cast<uint8_t>(10 + (ones - '0'));
}

void setupServo(uint8_t id) {
  if (pendingRestore || blocked) {
    Serial.println("SETUP REFUSED: RELEASE the current alignment first.");
    return;
  }
  ServoSetup::setup(id);
}

// "MOVEID 13 21": two IDs, each 11-19 or 21-29. Returns false if malformed.
bool moveCommand(const char* command) {
  const char* prefix = "MOVEID ";
  const size_t n = strlen(prefix);
  if (strncmp(command, prefix, n) != 0 || strlen(command) != n + 5 || command[n + 2] != ' ') return false;
  auto two = [](const char* s) -> uint8_t {
    if (s[0] < '1' || s[0] > '2' || s[1] < '1' || s[1] > '9') return 0;
    return static_cast<uint8_t>((s[0] - '0') * 10 + (s[1] - '0'));
  };
  const uint8_t from = two(command + n), to = two(command + n + 3);
  if (!from || !to || from == to) return false;
  if (pendingRestore || blocked) Serial.println("MOVEID REFUSED: RELEASE the current alignment first.");
  else ServoSetup::moveId(from, to);
  return true;
}

// Used by both the Arduino loop and host tests, including overflow handling.
void consume(char c) {
  static char command[24];
  static uint8_t used = 0;
  static bool overflow = false;
  if (c == '!') { release(); used = 0; overflow = false; return; }
  if (c == '\r') return;
  if (c != '\n') {
    if (!overflow && used < sizeof(command) - 1) command[used++] = c;
    else overflow = true;
    return;
  }
  command[used] = '\0';
  if (!overflow && strcmp(command, "RELEASE") == 0) release();
  else if (!overflow && strcmp(command, "KEEP") == 0) lastKeep = millis();
  else if (active) Serial.println("ASSEMBLY active: KEEP, RELEASE or ! only.");
  else if (!overflow && strcmp(command, "CENTERHEAD") == 0) begin(19);
  else if (!overflow && strcmp(command, "ALIGNFL") == 0) begin(11);
  else if (!overflow && strcmp(command, "ALIGNFR") == 0) begin(13);
  else if (!overflow && strcmp(command, "ALIGNBL") == 0) begin(15);
  else if (!overflow && strcmp(command, "ALIGNBR") == 0) begin(17);
  else if (!overflow && strcmp(command, "STATUS") == 0) HeadConfig::status();
  else if (!overflow && strcmp(command, "LEGS") == 0) HeadConfig::legStatus();
  else if (!overflow && strcmp(command, "SCAN") == 0) ServoSetup::scan();
  else if (!overflow && strcmp(command, "POS") == 0) ServoSetup::positions();
  else if (!overflow && idArgument(command, "SETUP ")) setupServo(idArgument(command, "SETUP "));
  else if (!overflow && idArgument(command, "WIGGLE ")) wiggle(idArgument(command, "WIGGLE "));
  else if (!overflow && moveCommand(command)) {}
  else if (used) Serial.println("Commands: SCAN, SETUP <11-19>, WIGGLE <11-19>, MOVEID <from> <to>, POS, "
                                "CENTERHEAD, ALIGNFL, ALIGNFR, ALIGNBL, ALIGNBR, STATUS, LEGS, KEEP, RELEASE, !");
  used = 0; overflow = false;
}
} // namespace SquroAssembly
