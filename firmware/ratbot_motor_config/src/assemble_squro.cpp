// RatBot assembly from a fresh build: servo setup one at a time, an
// identification wiggle, and the head and leg mounting poses. Normally driven
// by tools/assembly_console.py, which guides every step; see ASSEMBLE_SQURO.md.
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;
HardwareSerial motorSerial(0);
Dynamixel2Arduino dxl(motorSerial, 8);
#include "head_config.h"
#include "servo_setup.h"
#include "squro_assembly.h"

void setup() {
  Serial.begin(115200);
  delay(1500);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);
  SquroAssembly::startupRelease();
  Serial.println("RatBot assembly: no automatic movement; startup releases powered servos.");
  Serial.println("Run tools/assembly_console.py for the guided setup and leg assembly.");
  Serial.println("SCAN, POS: read-only. SETUP <11-19>: configure one servo. WIGGLE <11-19>: identify it.");
  Serial.println("CENTERHEAD: ID19 -> 4096. ALIGNFL/ALIGNFR: front elbow/shoulder -> 4695/5564.");
  Serial.println("ALIGNBL/ALIGNBR: hind elbow/shoulder -> 3627/4745.");
  Serial.println("Send KEEP at least every 10 seconds while active. RELEASE or ! stops. Hold limit: 5 minutes.");
}

void loop() {
  while (Serial.available()) SquroAssembly::consume(static_cast<char>(Serial.read()));
  SquroAssembly::tick();
  // The XIAO's built-in USB serial (core 2.0.17) can hold the end of a reply
  // until the next output; seen on the bench as a 5-10 s late SETUP/POS reply.
  // flush() sends it now; it returns at once when nothing is waiting and gives
  // up after its timeout when no host reads.
  Serial.flush();
  delay(2);
}
