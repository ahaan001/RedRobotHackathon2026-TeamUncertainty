/**
 * @brief Standalone QTR-8RC diagnostic. Prints the six raw decay times (us)
 *        and the read duration at 10 Hz. No motor calls.
 *
 * Library files are copies of RR26_Arduino/ (RedRobot.h, CONFIGURE-ME.h, src/).
 * Set TEAM_NUMBER / TEAM_NAME in CONFIGURE-ME.h or BLE init will complain.
 *
 * Expected (from the library code and the organizers' training data):
 *   white floor   ~50..200     black tape ~800..2000     sees nothing ~10000
 * If black reads LOWER than white on this board, set LINE_DARK_IS_HIGH=false
 * in RR26_Arduino/Config.h. Status: UNCOMPILED until SW2 compiles it.
 */
#include <Arduino.h>
#include "RedRobot.h"

void setup() {
  RedRobot::InitRedRobot();
  RedRobot::InitDCMotors();
  RedRobot::InitServo();
  RedRobot::InitLineSensors();
  RedRobot::InitAuton();
  Serial.println("[DIAG] QTR-8RC raw decay times (us), left to right. ~10000 = sees nothing.");
}

void AutonomousLoop() {
  uint16_t v[6];
  uint32_t t0 = micros();
  RedRobot::ReadLineSensor(v);
  uint32_t dt = micros() - t0;

  uint16_t lo = 65535, hi = 0;
  Serial.print("LF ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%5u ", v[i]);
    if (v[i] < lo) lo = v[i];
    if (v[i] > hi) hi = v[i];
  }
  Serial.printf("| min %5u max %5u | read %6lu us\n", lo, hi, (unsigned long)dt);
  delay(100);
}
