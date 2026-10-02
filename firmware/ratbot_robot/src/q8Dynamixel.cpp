/*
  q8Dynamixel.h - Wrapper for the Robotis Dynamixel2Arduino library.
  Created by Eric Wu, April 21st, 2024.
  Released into the public domain.
*/

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <MAX1704X.h>
#include <q8Dynamixel.h>

// MAX17043.h defines the instance rather than just declaring it, so including
// that header here would be a second definition. main.cpp owns it.
extern MAX1704X FuelGauge;

using namespace ControlTableItem;

// Constructor that takes an object of type Dynamixel2Arduino as an argument
q8Dynamixel::q8Dynamixel(Dynamixel2Arduino& dxl) : _dxl(dxl) {
  // Constructor implementation
  // Initialize any other members if needed
}

void q8Dynamixel::begin(){
  _dxl.begin(_baudrate);
  _dxl.setPortProtocolVersion(_protocolVersion);
  setOpMode();

  // Fill the members of structure for bulkWrite using internal packet buffer
  _bw_infos.packet.p_buf = nullptr;
  _bw_infos.packet.is_completed = false;
  _bw_infos.p_xels = _info_xels_bw;
  _bw_infos.xel_count = 0;

  for (int i = 0; i < _idCount; i++){
    _bw_data_xel[i].goal_position = 0;
    _info_xels_bw[i].id = _DXL[i];
    _info_xels_bw[i].addr = 116; // Goal Position of X serise.
    _info_xels_bw[i].addr_length = 4; // Goal Position
    _info_xels_bw[i].p_data = reinterpret_cast<uint8_t*>(&_bw_data_xel[i]);
    _bw_infos.xel_count++;
  }
  _bw_infos.is_info_changed = true;

  // Fill the members of structure to fastSyncRead using external user packet buffer
  _sr_infos.packet.buf_capacity = _user_pkt_buf_cap;
  _sr_infos.packet.p_buf = _user_pkt_buf;
  _sr_infos.packet.is_completed = false;
  _sr_infos.addr = SR_START_ADDR;
  _sr_infos.addr_length = SR_ADDR_LEN;
  _sr_infos.p_xels = _info_xels_sr;
  _sr_infos.xel_count = 0;  

  for(int i=0; i < _idCount; i++){
    _info_xels_sr[i].id = _DXL[i];;
    _info_xels_sr[i].p_recv_buf = (uint8_t*)&_sr_data[i];
    _sr_infos.xel_count++;
  }
  _sr_infos.is_info_changed = true;

  setProfile(1000);
  expandArrays();
}

bool q8Dynamixel::checkComms(uint8_t ID){
  return _dxl.ping(ID);
}

bool q8Dynamixel::commStart(){
  // Replace this with an actual check of ESPNow comms later
  return _torqueFlag;
}

uint16_t q8Dynamixel::checkBattery(){
  return 1;
}

void q8Dynamixel::enableTorque(){
  // The broadcast reaches the head too, which then holds whatever its goal
  // register contains. Park that at the present position first.
  if (parkHead()){
    _dxl.torqueOn(BROADCAST_ID);
    return;
  }
  // A stale goal could swing the head, so it stays limp until the next try.
  Serial.println("[HEAD] Goal not confirmed, head left without torque");
  for (int i = 0; i < _idCount; i++){
    _dxl.torqueOn(_DXL[i]);
  }
}

void q8Dynamixel::disableTorque(){
  _dxl.torqueOff(BROADCAST_ID);
}

void q8Dynamixel::toggleTorque(bool flag){
  if(flag){
    enableTorque();
  } else{
    disableTorque();
  }
}

void q8Dynamixel::resetTorqueState(){
  // Reset the internal torque flag to match disabled state
  // Used after connection loss when torque was already disabled
  _torqueFlag = false;
}

void q8Dynamixel::setOpMode(){
  // Set operating mode. Torque off first if needed.
  if (!_torqueFlag){
    for (int i = 0; i < _idCount; i++){
      _dxl.setOperatingMode(_DXL[i], OP_EXTENDED_POSITION);
    }
  } else{
    disableTorque();
    for (int i = 0; i < _idCount; i++){
      _dxl.setOperatingMode(_DXL[i], OP_EXTENDED_POSITION);
    }
    enableTorque();
  }
}

bool q8Dynamixel::writeVerified(uint8_t item, uint8_t id, int32_t value){
  // Bound retries so a missing motor cannot block the command task indefinitely.
  for (uint8_t attempt = 0; attempt < 2; ++attempt){
    if (!_dxl.writeControlTableItem(item, id, value, _writeTimeout)) continue;
    int32_t actual = _dxl.readControlTableItem(item, id, _writeTimeout);
    // Bit 7 of the status error byte is the Alert flag: it only reports that the
    // servo has a latched hardware error, not that this write was rejected. A
    // shutdown servo raises it on every packet, so masking it keeps one faulted
    // motor from making every other write look like a failure.
    if (_dxl.getLastLibErrCode() == 0 &&
        (_dxl.getLastStatusPacketError() & 0x7f) == 0 && actual == value) return true;
  }
  Serial.printf("[PROFILE] Servo %u item %u verification failed (wanted %ld); retry pending\n",
                id, item, static_cast<long>(value));
  return false;
}

uint8_t q8Dynamixel::reportFaults(){
  // Latched hardware errors survive until the servo is rebooted, so a collapse
  // stays diagnosable after the fact.
  uint8_t faulted = 0;
  // Captured first: a rail that has already collapsed points at the pack or its
  // protection, while a healthy reading points downstream at switch or wiring.
  Serial.printf("[FAULT] Pack %.0f mV (%.1f%%) at fault time\n",
                FuelGauge.voltage(), FuelGauge.percent());
  for (int i = 0; i < _idCount; i++){
    if (reportFault(_DXL[i])) faulted++;
  }
  // The head only once it has answered, so a robot without one stays quiet.
  if (_headSeen && reportFault(_headId)) faulted++;
  return faulted;
}

bool q8Dynamixel::reportFault(uint8_t id){
  int32_t status = _dxl.readControlTableItem(HARDWARE_ERROR_STATUS, id, _writeTimeout);
  if (_dxl.getLastLibErrCode() != 0){
    Serial.printf("[FAULT] Servo %u unreachable\n", id);
    return true;
  }
  if (status == 0) return false;
  Serial.printf("[FAULT] Servo %u hardware error 0x%02lX:%s%s%s%s%s\n", id,
                static_cast<unsigned long>(status),
                (status & 0x01) ? " input-voltage" : "",
                (status & 0x04) ? " overheating" : "",
                (status & 0x08) ? " encoder" : "",
                (status & 0x10) ? " electrical-shock" : "",
                (status & 0x20) ? " overload" : "");
  return true;
}

void q8Dynamixel::recover(){
  // Explicit operator action only. Rebooting clears latched shutdown errors,
  // which is the only way back from an overload without a power cycle.
  Serial.println("[RECOVER] Clearing latched servo faults");
  if (reportFaults() == 0) Serial.println("[RECOVER] No latched faults found");
  for (int i = 0; i < _idCount; i++){
    _dxl.reboot(_DXL[i], 200);
  }
  if (_headSeen) _dxl.reboot(_headId, 200);
  delay(500);
  setOpMode();
  _profileValid = false;
  setProfile(_profile);
  enableTorque();
  _torqueFlag = true;
  _prevTorqueFlag = true;
  Serial.println(_profileValid ? "[RECOVER] Torque restored" :
                                 "[RECOVER] Torque restored, profile unconfirmed");
}

void q8Dynamixel::setProfile(uint16_t dur){
  _profile = dur;
  _lastProfileAttempt = millis();
  // Always attempt every motor. Returning early would leave some legs on the
  // new timing and the rest on the old one, which desynchronises the gait.
  bool allConfirmed = true;
  for (int i = 0; i < _idCount; i++){
    allConfirmed &= writeVerified(POSITION_P_GAIN, _DXL[i], 400);
    allConfirmed &= writeVerified(PROFILE_VELOCITY, _DXL[i], dur);
    allConfirmed &= writeVerified(PROFILE_ACCELERATION, _DXL[i], dur / 3);
  }
  // Cache only a setting confirmed on every motor, including calls from jump().
  _prevProfile = dur;
  bool wasValid = _profileValid;
  _profileValid = allConfirmed;
  // Edge-triggered: the retry runs every 250 ms and must not spam the bus.
  if (!allConfirmed && wasValid) reportFaults();
}

void q8Dynamixel::setGain(uint16_t p_gain){
  // for Time-based Extended Pos, Profile velocity is the move duration (ms).
  for (int i = 0; i < _idCount; i++){
    _dxl.writeControlTableItem(POSITION_P_GAIN, _DXL[i], p_gain);
  }
}

void q8Dynamixel::moveSingle(int32_t val){
  // 8 motors move to the same position
  for (int i = 0; i < _idCount; i++){
    _bw_data_xel[i].goal_position = val;
  }
  _bw_infos.is_info_changed = true;

  _dxl.bulkWrite(&_bw_infos);
}

void q8Dynamixel::bulkWrite(int32_t values[8]){
  // An unconfirmed profile must not stop the robot: freezing mid-stride leaves
  // the legs holding a loaded pose, which is what drives them into overload.
  // Whatever timing the servos currently hold is better than no commands.
  // 8 motors move to their respective positions
  for (int i = 0; i < _idCount; i++){
    _bw_data_xel[i].goal_position = values[i];
  }
  _bw_infos.is_info_changed = true;

  _dxl.bulkWrite(&_bw_infos);
}

bool q8Dynamixel::parkHead(){
  // Goal Position is RAM: after power-up, a reboot or a hand adjustment it need
  // not match where the head is. Writing it with torque off moves nothing.
  // Returns false only for a head that answers but could not be parked.
  _headReady = false;
  int32_t present = _dxl.readControlTableItem(PRESENT_POSITION, _headId, _writeTimeout);
  if (_dxl.getLastLibErrCode() != 0) return true;  // No head on the bus
  _headSeen = true;
  if (!writeVerified(GOAL_POSITION, _headId, present)) return false;
  _headGoal = present;
  _headProfile = -1;
  _headReady = true;
  return true;
}

void q8Dynamixel::moveHead(float deg){
  // Nothing to move without a head, or before it was parked at torque-on.
  if (!_headReady || deg != deg) return;  // deg != deg: not a number
  if (deg > _headLimitDeg) deg = _headLimitDeg;
  if (deg < -_headLimitDeg) deg = -_headLimitDeg;
  int32_t goal = _deg2Dxl(deg);
  if (goal == _headGoal) return;
  // Time-based profile: the move time grows with the distance, so the GUI's
  // small steps blend into a steady turn and no jump is faster than 90 deg/s.
  int32_t distance = goal > _headGoal ? goal - _headGoal : _headGoal - goal;
  int32_t duration = (distance * 1000 + _headTicksPerSecond - 1) / _headTicksPerSecond;
  if (duration != _headProfile){
    // A goal sent with an unconfirmed profile could move at full speed.
    _headProfile = -1;
    if (!_dxl.writeControlTableItem(PROFILE_VELOCITY, _headId, duration, _writeTimeout) ||
        !_dxl.writeControlTableItem(PROFILE_ACCELERATION, _headId, duration / 10, _writeTimeout)) return;
    _headProfile = duration;
  }
  if (_dxl.writeControlTableItem(GOAL_POSITION, _headId, goal, _writeTimeout)) _headGoal = goal;
}

uint16_t* q8Dynamixel::syncRead(){
  // Read relevant registers from all joints into a single array
  int recv_cnt;
  size_t offset = 0;
  uint16_t* byteArray = new uint16_t[_idCount * 2];

  recv_cnt = _dxl.fastSyncRead(&_sr_infos);
  if(recv_cnt == _idCount){
    for (size_t i = 0; i < _idCount; i++){
      // cast to uint16_t since values never exceed 65535 in robot configuration
      byteArray[i*2] = static_cast<uint16_t>(_sr_data[i].present_current + 10000);
      byteArray[i*2+1] = static_cast<uint16_t>(_sr_data[i].present_position);
    }
  } else {
    // Fill ByteArray with zeros
    for (size_t i = 0; i < _idCount*2; ++i){
      byteArray[i] = 0;
    }
  }

  // for (size_t i = 0; i < 4; ++i){
  //   Serial.print(byteArray[i]); Serial.print(" ");
  // } Serial.println();
  return byteArray;
}

void q8Dynamixel::jump(){
  // Crouching Position
  setProfile(500);
  delay(100);
  bulkWrite(_lowArray);
  delay(1000);

  // Jump
  setProfile(0);
  setGain(800);
  delay(100);
  bulkWrite(_highArray);
  delay(100);
  bulkWrite(_restArray);
  delay(5000);

  // Back to idle
  setProfile(500);
  delay(100);
  bulkWrite(_idleArray);
  delay(1000);
}

uint8_t q8Dynamixel::parseData(const char* myData) {
  char* token = strtok(const_cast<char*>(myData), ",");
  int index = 0;
  int check = 0;
  
  while (token != nullptr && index < 8) {  // First 8 contain joint positions
    _posArray[index++] = _deg2Dxl(std::atof(token));
    token = strtok(nullptr, ",");
  }
  if (token != nullptr) {                  // 9th value is for special token
    _specialCmd = std::atoi(token);
    token = strtok(nullptr, ",");
    if (_specialCmd == 1){           // Battery
      return 1;
    } else if (_specialCmd == 2){    // Record
      check = 2;
    } else if (_specialCmd == 3){    // Send recorded
      return 3;
    } else if (_specialCmd == 4){    // Jump
      jump();
      return 0;
    } else if (_specialCmd == 5){    // Clear latched faults and re-enable torque
      recover();
      return 0;
    }
  }
  if (token != nullptr) {                   // 10th value is vel/acc profiles
    _profile = std::atoi(token);
    token = strtok(nullptr, ",");
    if ((_profileValid && _profile != _prevProfile) ||
        (!_profileValid && millis() - _lastProfileAttempt >= 250)){
      Serial.print("[ROBOT] Profile changed: "); Serial.println(_profile);
      setProfile(_profile);
    }
  }
  bool torqueChanged = false;
  if (token != nullptr) {                    // 11th value is torque enable/disable
    _torqueFlag = (std::atoi(token) == 1);
    token = strtok(nullptr, ",");
    if (_torqueFlag != _prevTorqueFlag){
      Serial.println(_torqueFlag ? "[ROBOT] Torque on" : "[ROBOT] Torque off");
      toggleTorque(_torqueFlag);
      _prevTorqueFlag = _torqueFlag;
      torqueChanged = true;
    }
  }
  if (token != nullptr && _torqueFlag) {     // 12th value is the head angle, optional
    moveHead(std::atof(token));
  }
  if (torqueChanged) return 0;
  bulkWrite(_posArray);
  return check - 0;
}

int32_t q8Dynamixel::_deg2Dxl(float deg){
  // Dynamixel joint 0 to 360 deg is 0 to 4096
  const float friendlyPerDxl = 360.0 / 4096.0 / _gearRatio;
  int angleDxl = static_cast<int>(deg / friendlyPerDxl + 0.5) + _zeroOffset;
  return angleDxl;
}

float q8Dynamixel::_dxl2Deg(int32_t dxlRaw){
  // Dynamixel joint 0 to 360 deg is 0 to 4096
  const float friendlyPerDxl = 360.0 / 4096.0 / _gearRatio;
  float angleFriendly = (dxlRaw - _zeroOffset) * friendlyPerDxl;
  return angleFriendly;
}

void q8Dynamixel::expandArrays(){
  for (int i = 0; i < 4; i++){
    _idleArray[i*2] = _deg2Dxl(_idlePos[0]);
    _idleArray[i*2+1] = _deg2Dxl(_idlePos[1]);
  }
  for (int i = 0; i < 4; i++){
    _lowArray[i*2] = _deg2Dxl(_jumpLow[0]);
    _lowArray[i*2+1] = _deg2Dxl(_jumpLow[1]);
  }
  for (int i = 0; i < 4; i++){
    _highArray[i*2] = _deg2Dxl(_jumpHigh[0]);
    _highArray[i*2+1] = _deg2Dxl(_jumpHigh[1]);
  }
  for (int i = 0; i < 4; i++){
    _restArray[i*2] = _deg2Dxl(_jumpRest[0]);
    _restArray[i*2+1] = _deg2Dxl(_jumpRest[1]);
  }
}
