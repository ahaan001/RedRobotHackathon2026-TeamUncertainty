/**
 * @file  Diagnostics.cpp
 * @brief Serial Monitor commands (115200 baud, newline terminated). Polled from
 *        the mechanism task, so they work in both auton and teleop modes.
 *
 *   h              help
 *   s <ch> <us>    raw servo pulse in microseconds (arm=0, jaw=1), clamped to limits
 *   S <ch> <us>    same but ignores the limits (500..2500) for finding end stops. Careful.
 *   a <preset>     arm preset index 0..7 (STOW FLOOR TOP CARRY L1 L2 L3 L4)
 *   j <preset>     jaw preset index 0..2 (OPEN RIM CLAMP)
 *   m <idx> <spd>  drive motor idx (0,2,3) at speed -100..100. Robot on blocks!
 *   x              stop all motors
 *   l              one line-sensor read: raw, normalized, position
 *   L              stream line-sensor reads at 10 Hz for 5 s
 *   c              calibrate line sensor for 5 s (slide it across the tape)
 *   p              print presets, current servo targets, mode
 *   b              run the place macro
 */
#include <Arduino.h>
#include "RedRobot.h"
#include "Robot.h"

namespace {
  char s_buf[48];
  size_t s_len = 0;
  uint32_t s_streamUntil = 0;
  uint32_t s_streamNext = 0;

  void help() {
    Serial.println("[DIAG] h | s ch us | S ch us | a n | j n | m idx spd | x | l | L | c | p | b");
  }

  void printReading(const LineFollow::Reading& r) {
    Serial.print("[LF] raw:");
    for (int i = 0; i < 6; i++) Serial.printf(" %5u", r.raw[i]);
    Serial.print("  norm:");
    for (int i = 0; i < 6; i++) Serial.printf(" %4d", r.norm[i]);
    Serial.printf("  pos=%4d sum=%4d sat=%d %s%s\n", r.position, r.sum, r.saturated,
                  r.onLine ? "ON " : "off", r.endMarker ? " END" : "");
  }

  void status() {
    Serial.printf("[DIAG] fw %s | mode %s | auton %s\n", FW_VERSION,
                  RedRobot::IsRemoteControlled() ? "TELEOP" : "AUTON",
                  Auton::stateName(Auton::state()));
    Serial.printf("[DIAG] arm target %d us current %d us preset %d | jaw target %d us preset %d | precision %s\n",
                  Mech::armTargetUs(), Mech::armCurrentUs(), (int)Mech::armPreset(),
                  Mech::jawTargetUs(), (int)Mech::jawPreset(), Drive::precision() ? "on" : "off");
    Serial.print("[DIAG] arm presets us:");
    for (int i = 0; i < ARM_PRESET_COUNT; i++) Serial.printf(" %d", ARM_PRESET_US[i]);
    Serial.print(" | jaw presets us:");
    for (int i = 0; i < JAW_PRESET_COUNT; i++) Serial.printf(" %d", JAW_PRESET_US[i]);
    Serial.println();
    LineFollow::printCalibration();
  }

  void handle(char* line) {
    char cmd = line[0];
    int a = 0, b = 0;
    int n = sscanf(line + 1, "%d %d", &a, &b);

    switch (cmd) {
      case 'h': help(); break;

      case 's':
        if (n == 2) {
          if (a == SERVO_ARM) Mech::setArmUs(b); else if (a == SERVO_JAW) Mech::setJawUs(b);
          else Serial.println("[DIAG] servo channel must be 0 or 1");
          Serial.printf("[DIAG] servo %d -> %d us (clamped to limits)\n", a, b);
        } else Serial.println("[DIAG] usage: s <ch> <us>");
        break;

      case 'S':
        if (n == 2 && a >= 0 && a <= 3) {
          b = clampInt(b, 500, 2500);
          RedRobot::SetServo((uint8_t)a, servoUnits(b));   // bypasses Mech slew and limits
          Serial.printf("[DIAG] RAW servo %d -> %d us (%d units). Mech task will not touch ch 2,3.\n",
                        a, b, servoUnits(b));
          if (a == SERVO_ARM || a == SERVO_JAW)
            Serial.println("[DIAG] note: Mech task will slew back to its own target on next change");
        } else Serial.println("[DIAG] usage: S <ch 0-3> <us 500-2500>");
        break;

      case 'a':
        if (n >= 1 && a >= 0 && a < ARM_PRESET_COUNT) { Mech::setArm((ArmPreset)a); Serial.printf("[DIAG] arm preset %d\n", a); }
        else Serial.println("[DIAG] usage: a <0-7>");
        break;

      case 'j':
        if (n >= 1 && a >= 0 && a < JAW_PRESET_COUNT) { Mech::setJaw((JawPreset)a); Serial.printf("[DIAG] jaw preset %d\n", a); }
        else Serial.println("[DIAG] usage: j <0-2>");
        break;

      case 'm':
        if (n == 2 && (a == 0 || a == 2 || a == 3)) {
          RedRobot::WriteDCMotor((uint8_t)a, (int16_t)clampInt(b, -100, 100));
          Serial.printf("[DIAG] motor %d -> %d\n", a, b);
        } else Serial.println("[DIAG] usage: m <0|2|3> <-100..100>");
        break;

      case 'x':
        RedRobot::WriteDCMotor(0, 0); RedRobot::WriteDCMotor(2, 0); RedRobot::WriteDCMotor(3, 0);
        Serial.println("[DIAG] motors stopped");
        break;

      case 'l': printReading(LineFollow::read()); break;

      case 'L':
        s_streamUntil = millis() + 5000;
        s_streamNext = 0;
        break;

      case 'c':
        Serial.println("[DIAG] calibrating 5 s: slide the sensor across the tape now");
        LineFollow::calibrate(5000);
        break;

      case 'p': status(); break;

      case 'b': Mech::startPlaceMacro(); Serial.println("[DIAG] place macro"); break;

      default:
        Serial.printf("[DIAG] unknown '%c'\n", cmd);
        help();
    }
  }
}

namespace Diag {

  void begin() {
    s_len = 0;
    s_streamUntil = 0;
  }

  void poll() {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        s_buf[s_len] = '\0';
        if (s_len > 0) handle(s_buf);
        s_len = 0;
      } else if (s_len < sizeof(s_buf) - 1) {
        s_buf[s_len++] = c;
      } else {
        s_len = 0;   // overflow, drop the line
      }
    }

    if (s_streamUntil != 0) {
      uint32_t now = millis();
      if (now >= s_streamUntil) { s_streamUntil = 0; return; }
      if (now >= s_streamNext) {
        s_streamNext = now + 100;
        printReading(LineFollow::read());
      }
    }
  }
}
