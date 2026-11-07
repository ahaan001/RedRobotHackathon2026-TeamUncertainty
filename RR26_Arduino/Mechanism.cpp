/**
 * @file  Mechanism.cpp
 * @brief Arm and gripper control. A 50 Hz FreeRTOS task slews each servo
 *        toward its target, runs the place macro, polls serial diagnostics and
 *        (optionally) stops the drive if the controller goes quiet.
 *
 * Why a task: RedRobot::SetServo refuses calls from the Arduino loop task while
 * in remote-control mode, and BLE callbacks must not block. A task of our own
 * passes the guard in both modes and gives one code path for auton and teleop.
 *
 * Servos are limp after InitServo() until the first SetServo, then snap to it.
 * Put the arm in STOW by hand before power-on; begin() commands STOW at once.
 */
#include <Arduino.h>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "RedRobot.h"
#include "Robot.h"

namespace {
  TaskHandle_t s_task = nullptr;

  // Targets written by any task, read by the mech task.
  std::atomic<int> s_armTarget{ARM_PRESET_US[ARM_STOW]};
  std::atomic<int> s_jawTarget{JAW_PRESET_US[JAW_CLAMP]};
  std::atomic<int> s_armJog{0};                 // -100..100
  std::atomic<int> s_armPreset{(int)ARM_STOW};
  std::atomic<int> s_jawPreset{(int)JAW_CLAMP};
  std::atomic<uint32_t> s_lastCmdMs{0};

  // Owned by the mech task only.
  int s_armCur = ARM_PRESET_US[ARM_STOW];
  int s_jawCur = JAW_PRESET_US[JAW_CLAMP];
  int s_armWritten = -1;
  int s_jawWritten = -1;

  // Place macro
  enum MacroState : uint8_t { M_IDLE = 0, M_LOWER, M_SETTLE, M_OPEN, M_LIFT };
  std::atomic<int> s_macro{M_IDLE};
  int      s_macroLevelUs = 0;     // arm target before the drop, restored at the end
  uint32_t s_macroT = 0;

  int slewToward(int cur, int target, int maxStep) {
    if (cur < target) return min(cur + maxStep, target);
    if (cur > target) return max(cur - maxStep, target);
    return cur;
  }

  void macroStep(uint32_t now) {
    switch ((MacroState)s_macro.load()) {
      case M_IDLE:
        break;
      case M_LOWER:
        if (abs(s_armCur - s_armTarget.load()) <= 3) {
          s_macroT = now;
          s_macro.store(M_SETTLE);
        }
        break;
      case M_SETTLE:
        if (now - s_macroT >= MACRO_SETTLE_MS) {
          s_jawTarget.store(JAW_PRESET_US[JAW_OPEN]);
          s_jawPreset.store((int)JAW_OPEN);
          s_macroT = now;
          s_macro.store(M_OPEN);
        }
        break;
      case M_OPEN:
        if (abs(s_jawCur - s_jawTarget.load()) <= 3 && now - s_macroT >= MACRO_OPEN_WAIT_MS) {
          s_armTarget.store(s_macroLevelUs);   // back up to the level, jaws clear the flange
          s_macro.store(M_LIFT);
        }
        break;
      case M_LIFT:
        if (abs(s_armCur - s_armTarget.load()) <= 3) {
          s_macro.store(M_IDLE);
        }
        break;
    }
  }

  void mechTask(void*) {
    const TickType_t period = pdMS_TO_TICKS(MECH_TICK_MS);
    const int armStep = max(1, (int)(ARM_SLEW_US_PER_S * MECH_TICK_MS / 1000));
    const int jawStep = max(1, (int)(JAW_SLEW_US_PER_S * MECH_TICK_MS / 1000));
    TickType_t last = xTaskGetTickCount();

    // First write: snap to the boot pose. The arm must already be there by hand.
    RedRobot::SetServo(SERVO_ARM, servoUnits(s_armCur));
    RedRobot::SetServo(SERVO_JAW, servoUnits(s_jawCur));
    s_armWritten = s_armCur;
    s_jawWritten = s_jawCur;

    for (;;) {
      vTaskDelayUntil(&last, period);
      uint32_t now = millis();

      // Right-joystick jog moves the target, not the position, so slew still applies.
      int jog = s_armJog.load();
      if (jog != 0) {
        int delta = jog * (int)(ARM_JOG_US_PER_S * MECH_TICK_MS / 1000) / 100;
        int t = clampInt(s_armTarget.load() + delta, ARM_PULSE_MIN, ARM_PULSE_MAX);
        s_armTarget.store(t);
      }

      s_armCur = slewToward(s_armCur, clampInt(s_armTarget.load(), ARM_PULSE_MIN, ARM_PULSE_MAX), armStep);
      s_jawCur = slewToward(s_jawCur, clampInt(s_jawTarget.load(), JAW_PULSE_MIN, JAW_PULSE_MAX), jawStep);

      if (s_armCur != s_armWritten) {
        RedRobot::SetServo(SERVO_ARM, servoUnits(s_armCur));
        s_armWritten = s_armCur;
      }
      if (s_jawCur != s_jawWritten) {
        RedRobot::SetServo(SERVO_JAW, servoUnits(s_jawCur));
        s_jawWritten = s_jawCur;
      }

      macroStep(now);

      if (TELEOP_WATCHDOG_ENABLED && RedRobot::IsRemoteControlled()) {
        uint32_t lastCmd = s_lastCmdMs.load();
        if (lastCmd != 0 && now - lastCmd > TELEOP_CMD_TIMEOUT_MS) {
          Drive::stop();
          s_lastCmdMs.store(0);   // stop once, not every tick
        }
      }

      Diag::poll();
    }
  }
}

namespace Mech {

  void begin() {
    if (s_task) return;
    s_armCur = ARM_PRESET_US[ARM_STOW];
    s_jawCur = JAW_PRESET_US[JAW_CLAMP];
    s_armTarget.store(s_armCur);
    s_jawTarget.store(s_jawCur);
    Diag::begin();
    xTaskCreatePinnedToCore(mechTask, "mech", 4096, nullptr, 1, &s_task, 1);
    Serial.println("[MECH] task started, arm STOW, jaw CLAMP");
  }

  // ---- Arm ----
  void setArm(ArmPreset p) {
    if (p >= ARM_PRESET_COUNT) return;
    cancelMacro();
    s_armPreset.store((int)p);
    s_armTarget.store(ARM_PRESET_US[p]);
  }
  void setArmUs(int us) {
    cancelMacro();
    s_armTarget.store(clampInt(us, ARM_PULSE_MIN, ARM_PULSE_MAX));
  }
  void armPresetUp() {
    int p = s_armPreset.load();
    if (p < (int)ARM_FLOOR) p = (int)ARM_FLOOR; else if (p < (int)ARM_L4) p++;
    setArm((ArmPreset)p);
  }
  void armPresetDown() {
    int p = s_armPreset.load();
    if (p <= (int)ARM_FLOOR || p == (int)ARM_STOW) p = (int)ARM_FLOOR; else p--;
    setArm((ArmPreset)p);
  }
  void setArmJog(int rate)  { s_armJog.store(clampInt(rate, -100, 100)); }
  int  armTargetUs()        { return s_armTarget.load(); }
  int  armCurrentUs()       { return s_armCur; }
  bool armAtTarget()        { return abs(s_armCur - s_armTarget.load()) <= 3; }
  ArmPreset armPreset()     { return (ArmPreset)s_armPreset.load(); }

  // ---- Jaw ----
  void setJaw(JawPreset p) {
    if (p >= JAW_PRESET_COUNT) return;
    s_jawPreset.store((int)p);
    s_jawTarget.store(JAW_PRESET_US[p]);
  }
  void setJawUs(int us) { s_jawTarget.store(clampInt(us, JAW_PULSE_MIN, JAW_PULSE_MAX)); }
  void jawOpen()        { setJaw(JAW_OPEN); }
  void jawNextClosed() {
    // OPEN -> RIM -> CLAMP -> RIM -> CLAMP ...
    JawPreset p = (JawPreset)s_jawPreset.load();
    setJaw(p == JAW_RIM ? JAW_CLAMP : JAW_RIM);
  }
  int  jawTargetUs()    { return s_jawTarget.load(); }
  bool jawAtTarget()    { return abs(s_jawCur - s_jawTarget.load()) <= 3; }
  JawPreset jawPreset() { return (JawPreset)s_jawPreset.load(); }

  // ---- Macros ----
  void startPlaceMacro() {
    if (s_macro.load() != M_IDLE) return;
    s_macroLevelUs = s_armTarget.load();
    // Lower toward FLOOR by ARM_SET_DROP_US. Direction depends on which way the
    // pulse grows as the arm goes down, so compute it from the presets.
    int dir = (ARM_PRESET_US[ARM_FLOOR] < ARM_PRESET_US[ARM_L4]) ? -1 : 1;
    int t = clampInt(s_macroLevelUs + dir * ARM_SET_DROP_US, ARM_PULSE_MIN, ARM_PULSE_MAX);
    s_armTarget.store(t);
    s_macro.store(M_LOWER);
  }
  void cancelMacro()   { s_macro.store(M_IDLE); }
  bool macroRunning()  { return s_macro.load() != M_IDLE; }

  void stow() {
    cancelMacro();
    s_armJog.store(0);
    Drive::stop();
    setArm(ARM_STOW);
  }

  void noteCommand() { s_lastCmdMs.store(millis()); }
}
