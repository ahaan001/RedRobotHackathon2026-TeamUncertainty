/**
 * @file  Auton.cpp
 * @brief 20-second autonomous routine: exit the start box, follow the line to
 *        the trench, set the preloaded pipe down, back off, hand over to teleop.
 *
 * Non-blocking: step() is called repeatedly from AutonomousLoop() and uses
 * millis(). Every state has a timeout. A hard stop at AUTON_HARD_STOP_MS runs
 * regardless of state. X on the controller aborts at any time (the stock
 * handler calls StartRemoteControl and zeroes the drive from the BLE task).
 */
#include <Arduino.h>
#include <atomic>
#include "RedRobot.h"
#include "Robot.h"

namespace {
  std::atomic<bool> s_armed{false};
  std::atomic<bool> s_hasRun{false};
  Auton::State s_state = Auton::IDLE;
  uint32_t s_t0 = 0;          // routine start
  uint32_t s_tState = 0;      // state entry
  uint32_t s_lostSince = 0;
  bool s_subFlag = false;     // per-state one-shot flag

  void enter(Auton::State s) {
    s_state = s;
    s_tState = millis();
    s_subFlag = false;
    Serial.printf("[AUTON] %6lu ms -> %s\n", (unsigned long)(s_tState - s_t0), Auton::stateName(s));
  }

  void finish(Auton::State s) {
    Drive::stop();
    enter(s);
    s_armed.store(false);
    RedRobot::StartRemoteControl();
    Serial.println("[AUTON] handed over to remote control");
  }
}

namespace Auton {

  const char* stateName(State s) {
    switch (s) {
      case IDLE: return "IDLE";         case EXIT_BOX: return "EXIT_BOX";
      case FOLLOW: return "FOLLOW";     case RECOVER: return "RECOVER";
      case ALIGN: return "ALIGN";       case RELEASE: return "RELEASE";
      case RETREAT: return "RETREAT";   case DONE: return "DONE";
      case STOP_SAFE: return "STOP_SAFE";
    }
    return "?";
  }

  void arm() {
    if (s_armed.load()) return;
    s_t0 = millis();
    s_armed.store(true);
    s_hasRun.store(true);
    s_lostSince = 0;
    Serial.println("[AUTON] start");
    enter(EXIT_BOX);
  }

  void abort() {
    if (s_state != DONE && s_state != STOP_SAFE) finish(STOP_SAFE);
  }

  bool  isArmed() { return s_armed.load(); }
  bool  hasRun()  { return s_hasRun.load(); }
  State state()   { return s_state; }

  void step() {
    if (!s_armed.load()) { vTaskDelay(pdMS_TO_TICKS(5)); return; }

    uint32_t now = millis();
    uint32_t t = now - s_t0;
    uint32_t ts = now - s_tState;

    if (t >= AUTON_HARD_STOP_MS && s_state != DONE && s_state != STOP_SAFE) {
      Serial.println("[AUTON] hard stop");
      finish(DONE);
      return;
    }

    switch (s_state) {

      case IDLE:
        vTaskDelay(pdMS_TO_TICKS(5));
        break;

      case EXIT_BOX: {
        if (!s_subFlag) {
          s_subFlag = true;
          Mech::setArm(ARM_CARRY);                 // lift the preload clear of the floor
          Drive::raw(AUTON_EXIT_SPEED, AUTON_EXIT_SPEED);
        }
        if (AUTON_SIMPLE_EXIT_ONLY) {
          if (ts >= AUTON_EXIT_MAX_MS) finish(DONE);
          break;
        }
        const LineFollow::Reading& r = LineFollow::read();
        if ((ts >= AUTON_EXIT_MIN_MS && r.onLine) || ts >= AUTON_EXIT_MAX_MS) enter(FOLLOW);
        break;
      }

      case FOLLOW: {
        const LineFollow::Reading& r = LineFollow::read();
        if (r.endMarker && ts > 500) { Drive::stop(); enter(ALIGN); break; }
        if (r.onLine) {
          s_lostSince = 0;
          int l, rr;
          LineFollow::steer(r, LF_BASE_SPEED, LF_KP, l, rr);
          Drive::raw(l, rr);
        } else {
          if (s_lostSince == 0) s_lostSince = now;
          if (now - s_lostSince > LF_LOST_TIMEOUT_MS) enter(RECOVER);
        }
        if (ts > AUTON_FOLLOW_MAX_MS) finish(STOP_SAFE);   // lost; keep the pipe for teleop
        break;
      }

      case RECOVER: {
        int sgn = LineFollow::lastSteerSign();
        Drive::raw(LF_RECOVER_SPEED * sgn, -LF_RECOVER_SPEED * sgn);   // pivot toward the line
        const LineFollow::Reading& r = LineFollow::read();
        if (r.onLine) { s_lostSince = 0; enter(FOLLOW); break; }
        if (ts > LF_RECOVER_MS) finish(STOP_SAFE);
        break;
      }

      case ALIGN: {
        // Straight segment, then optional timed turn. Values come from the field walk.
        if (ts < AUTON_ALIGN_DRIVE_MS) {
          Drive::raw(AUTON_ALIGN_DRIVE_SPEED, AUTON_ALIGN_DRIVE_SPEED);
        } else if (ts < AUTON_ALIGN_DRIVE_MS + AUTON_ALIGN_TURN_MS) {
          Drive::raw(AUTON_ALIGN_TURN_SPEED, -AUTON_ALIGN_TURN_SPEED);
        } else {
          Drive::stop();
          enter(RELEASE);
        }
        break;
      }

      case RELEASE: {
        if (!s_subFlag) { s_subFlag = true; Mech::setArm(ARM_FLOOR); }
        if (ts >= AUTON_RELEASE_SETTLE_MS && Mech::jawPreset() != JAW_OPEN) Mech::jawOpen();
        if (ts >= AUTON_RELEASE_SETTLE_MS + AUTON_RELEASE_OPEN_MS) enter(RETREAT);
        break;
      }

      case RETREAT: {
        if (!s_subFlag) { s_subFlag = true; Drive::raw(-AUTON_RETREAT_SPEED, -AUTON_RETREAT_SPEED); }
        if (ts >= AUTON_RETREAT_MS) {
          Drive::stop();
          Mech::setArm(ARM_STOW);
          finish(DONE);
        }
        break;
      }

      case DONE:
      case STOP_SAFE:
        vTaskDelay(pdMS_TO_TICKS(5));
        break;
    }
  }
}
