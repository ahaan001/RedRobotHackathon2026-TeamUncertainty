/**
 * @file  Robot.h
 * @brief Declarations for Team Uncertainty's modules. Include this from every
 *        .ino and .cpp in the sketch. The organizers' library stays in RedRobot.h.
 *
 * Threading model (see docs/ARCHITECTURE.md):
 *   - BLE callbacks (RemoteControl.ino) run on the NimBLE task. They only set
 *     targets and call Drive::; they never block.
 *   - Mech:: owns a FreeRTOS task at 50 Hz that slews servos, runs macros and
 *     polls the serial diagnostics. It is not the Arduino loop task, so its
 *     actuator calls pass the library's remote-control guard in both modes.
 *   - Auton:: runs inside AutonomousLoop() on the Arduino loop task and is
 *     non-blocking (millis based).
 */
#pragma once
#include <Arduino.h>
#include "Config.h"

namespace Drive {
  void arcade(int x, int y);          // x,y in -100..100 from Redpad
  void tank(int left, int right);     // -100..100 each, after scaling
  void raw(int left, int right);      // -100..100, no scaling, no expo (auton)
  void stop();
  void setPrecision(bool on);
  bool precision();
  void togglePrecision();
}

namespace Mech {
  void begin();                        // call after RedRobot::InitServo()
  // Arm
  void setArm(ArmPreset p);
  void setArmUs(int pulse_us);         // clamped to ARM_PULSE_MIN..MAX
  void armPresetUp();                  // FLOOR -> TOP -> CARRY -> L1 .. L4
  void armPresetDown();
  void setArmJog(int rate);            // -100..100, from right joystick Y; 0 stops
  int  armTargetUs();
  int  armCurrentUs();
  bool armAtTarget();
  ArmPreset armPreset();
  // Jaw
  void setJaw(JawPreset p);
  void setJawUs(int pulse_us);
  void jawOpen();
  void jawNextClosed();                // OPEN -> RIM -> CLAMP -> RIM ...
  int  jawTargetUs();
  bool jawAtTarget();
  JawPreset jawPreset();
  // Macros
  void startPlaceMacro();              // lower one step, open, lift back to level
  void cancelMacro();
  bool macroRunning();
  void stow();                         // arm STOW, drive stop
  // Teleop watchdog
  void noteCommand();                  // call from every BLE callback
}

namespace LineFollow {
  struct Reading {
    uint16_t raw[6];
    int      norm[6];      // 0 (white) .. 1000 (black), 0 if saturated
    int      sum;          // sum of norm
    int      position;     // 0..5000, 2500 = centered; valid only if onLine
    int      saturated;    // channels that see nothing
    bool     onLine;
    bool     endMarker;
  };
  void begin();
  const Reading& read();                       // one sensor read, 7..60 ms
  const Reading& last();
  void calibrate(uint32_t ms);                 // blocking, moves nothing; sets white/black
  void setCalibration(const uint16_t white[6], const uint16_t black[6]);
  void printCalibration();
  void steer(const Reading& r, int base, int kp, int& left, int& right);
  int  lastSteerSign();                        // +1 / -1, for line-loss recovery
}

namespace Auton {
  enum State : uint8_t {
    IDLE = 0, EXIT_BOX, FOLLOW, RECOVER, ALIGN, RELEASE, RETREAT, DONE, STOP_SAFE
  };
  void arm();            // start the clock and the routine (called when A is pressed)
  void step();           // call repeatedly from AutonomousLoop()
  void abort();          // stop everything, hand over to teleop
  bool isArmed();
  bool hasRun();
  State state();
  const char* stateName(State s);
}

namespace Diag {
  void begin();
  void poll();           // non-blocking serial command parser, called from Mech task
}
