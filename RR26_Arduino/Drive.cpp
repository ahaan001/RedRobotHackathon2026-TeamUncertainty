/**
 * @file  Drive.cpp
 * @brief Differential drive helpers on top of RedRobot::WriteDCMotor.
 */
#include <Arduino.h>
#include <atomic>
#include "RedRobot.h"
#include "Robot.h"

namespace {
  std::atomic<bool> s_precision{false};

  int applyDeadband(int v) {
    if (v > -DRIVE_DEADBAND && v < DRIVE_DEADBAND) return 0;
    return v;
  }

  // Squared input curve that keeps the sign: fine control near center, full speed at the ends.
  int expo(int v) {
    if (!DRIVE_EXPO) return v;
    return v * abs(v) / 100;
  }

  void writeMotors(int left, int right) {
    left  = clampInt(left,  -100, 100) * MOTOR_LEFT_SIGN;
    right = clampInt(right, -100, 100) * MOTOR_RIGHT_SIGN;
    RedRobot::WriteDCMotor(MOTOR_LEFT,  left);
    RedRobot::WriteDCMotor(MOTOR_RIGHT, right);
  }
}

namespace Drive {

  void arcade(int x, int y) {
    x = expo(applyDeadband(clampInt(x, -100, 100)));
    y = expo(applyDeadband(clampInt(y, -100, 100)));

    int left  = y + x;
    int right = y - x;

    int mag = max(abs(left), abs(right));
    if (mag > 100) {
      left  = left  * 100 / mag;
      right = right * 100 / mag;
    }

    int scale = s_precision.load() ? DRIVE_SCALE_PRECISION : DRIVE_SCALE_FULL;
    writeMotors(left * scale / 100, right * scale / 100);
  }

  void tank(int left, int right) {
    int scale = s_precision.load() ? DRIVE_SCALE_PRECISION : DRIVE_SCALE_FULL;
    left  = expo(applyDeadband(clampInt(left,  -100, 100)));
    right = expo(applyDeadband(clampInt(right, -100, 100)));
    writeMotors(left * scale / 100, right * scale / 100);
  }

  void raw(int left, int right) {
    writeMotors(left, right);
  }

  void stop() {
    writeMotors(0, 0);
  }

  void setPrecision(bool on) { s_precision.store(on); }
  bool precision()           { return s_precision.load(); }
  void togglePrecision()     { s_precision.store(!s_precision.load()); }

}
