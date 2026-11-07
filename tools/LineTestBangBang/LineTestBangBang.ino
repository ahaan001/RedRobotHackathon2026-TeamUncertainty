/**
 * Bang-bang line test: read the six sensors as a list, look at L1 and L3 each step.
 *   L1 white -> right motor on.   L3 white -> left motor on.
 * Both white = straight. One black = the robot turns toward that sensor's side.
 *
 * Fix vs. the earlier sketch: RedRobot::InitRedRobot() must be called first. It creates
 * the actuator mutex that WriteDCMotor needs and starts Serial. Without it the board
 * asserts on the first motor command and reboots.
 *
 * Library files are copies of RR26_Arduino/ (RedRobot.h, CONFIGURE-ME.h, src/).
 * Open the Serial Monitor at 115200 to see the readings and the decision each step.
 */
#include <Arduino.h>
#include "RedRobot.h"

// ===== MOTORS (as currently wired: 0 = left, 2 = right; assembly guide says M2/M3) =====
const int LEFT_MOTOR    = 0;
const int RIGHT_MOTOR   = 2;
const int LEFT_FORWARD  = 1;      // -1 if that wheel runs backward
const int RIGHT_FORWARD = 1;
const int DRIVE_SPEED   = 30;     // percent when a motor is "on"
const int TURN_SPEED    = 0;      // speed of the "off" motor; 10-15 gives gentler turns

// ===== LINE SENSOR =====
// Index into the 6-element list = board input LF0..LF5.
// From your ORDER table, channel 1 is the RIGHT-side sensor and channel 3 the LEFT-side one
// (47 mm apart, wider than the 25 mm tape, so a centered line sits between them).
const int SENSOR_L1 = 1;
const int SENSOR_L3 = 3;

// White floor reads LOW (fast decay), black tape reads HIGH, "sees nothing" ~10000.
// Organizers' data: white ~50, black ~1000. Set WHITE_BELOW halfway between YOUR readings.
const int WHITE_BELOW   = 500;
const int SEES_NOTHING  = 9000;   // above this the sensor is off the floor: stop for safety
const bool BOTH_BLACK_STRAIGHT = true;   // at a tape crossing both go black: keep driving

const unsigned long TEST_TIME = 15000;

uint16_t sensors[6];
unsigned long startTime = 0;
bool stopped = false;

void setMotors(int left, int right) {
  RedRobot::WriteDCMotor(LEFT_MOTOR,  LEFT_FORWARD  * left);
  RedRobot::WriteDCMotor(RIGHT_MOTOR, RIGHT_FORWARD * right);
}

void setup() {
  RedRobot::InitRedRobot();      // REQUIRED first: mutex + Serial + display + BLE
  RedRobot::InitDCMotors();
  RedRobot::InitLineSensors();
  RedRobot::InitAuton();         // AutonomousLoop runs from loop()
  setMotors(0, 0);
  Serial.println("[LINE] 3 s to place the robot on the line");
  delay(3000);
  startTime = millis();
}

void AutonomousLoop() {
  if (stopped) { delay(100); return; }

  if (millis() - startTime >= TEST_TIME) {
    setMotors(0, 0);
    stopped = true;
    Serial.println("[LINE] time up, stopped");
    return;
  }

  RedRobot::ReadLineSensor(sensors);         // fills the list, 7..60 ms

  uint16_t l1 = sensors[SENSOR_L1];
  uint16_t l3 = sensors[SENSOR_L3];

  if (l1 >= SEES_NOTHING || l3 >= SEES_NOTHING) {
    setMotors(0, 0);
    stopped = true;
    Serial.printf("[LINE] sensor sees nothing (L1=%u L3=%u): off the floor? stopped\n", l1, l3);
    return;
  }

  bool l1White = l1 < WHITE_BELOW;
  bool l3White = l3 < WHITE_BELOW;

  int right = l1White ? DRIVE_SPEED : TURN_SPEED;
  int left  = l3White ? DRIVE_SPEED : TURN_SPEED;
  if (!l1White && !l3White && BOTH_BLACK_STRAIGHT) { left = DRIVE_SPEED; right = DRIVE_SPEED; }

  setMotors(left, right);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.printf("L1=%5u(%s) L3=%5u(%s) -> left %2d right %2d | all:", l1, l1White ? "W" : "B", l3, l3White ? "W" : "B", left, right);
    for (int i = 0; i < 6; i++) Serial.printf(" %5u", sensors[i]);
    Serial.println();
  }
  delay(10);
}
