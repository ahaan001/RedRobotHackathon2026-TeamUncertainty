/**
 * @brief Red Robot 2026, Team Uncertainty main sketch.
 *
 * Boot sequence: the organizers' library inits, then our mechanism task starts
 * and commands the STOW pose. With BOOT_INTO_TELEOP the robot sits idle in
 * remote-control mode until Redpad A is pressed, which starts the 20 s
 * autonomous routine (Auton.cpp). X exits autonomous at any time.
 *
 * Put the arm in STOW by hand before switching on: servos are limp until the
 * first command and then snap to it.
 *
 * Status: UNCOMPILED skeleton. SW1 compiles and flashes; see docs/SETUP.md.
 */

// Libraries with driver code (don't remove these!)
#include <Arduino.h>                // make sure this is first
#include "RedRobot.h"
#include "Config.h"
#include "Robot.h"

// This function runs ONCE, when the microcontroller boots up
void setup() {
  RedRobot::InitRedRobot();         // call this first
  RedRobot::InitDCMotors();
  RedRobot::InitServo();
  RedRobot::InitLineSensors();
  RedRobot::InitAuton();

  if (BOOT_INTO_TELEOP) {
    RedRobot::StartRemoteControl(); // idle: nothing moves until A (auton) or the joystick
  }

  LineFollow::begin();
  Mech::begin();                    // starts the 50 Hz servo task, commands STOW + CLAMP

  Serial.print("[TU] Team Uncertainty firmware ");
  Serial.println(FW_VERSION);
  Serial.println(BOOT_INTO_TELEOP ? "[TU] idle in teleop; press A to start auton, X to exit auton"
                                  : "[TU] autonomous starts now (organizer default)");
}

// Runs over and over while the robot is in autonomous mode (not remote controlled).
void AutonomousLoop() {
  if (!BOOT_INTO_TELEOP && !Auton::isArmed() && !Auton::hasRun()) {
    Auton::arm();                   // organizer default: start at boot
  }
  Auton::step();                    // non-blocking state machine
}
