
#include <Arduino.h>
#include "RedRobot.h"

// =====================================
// MOTOR CONFIGURATION
// =====================================

const int LEFT_MOTOR = 0;
const int RIGHT_MOTOR = 2;

// Confirmed directions
const int LEFT_FORWARD = -1;
const int RIGHT_FORWARD = 1;

// =====================================
// LINE FOLLOWING SETTINGS
// =====================================

const int BASE_SPEED = 25;
const int MIN_SPEED = 15;
const int MAX_SPEED = 35;

const float KP = 3.0;

// =====================================
// LINE SENSOR CONFIGURATION
// =====================================

uint16_t sensors[6];

// Physical left-to-right order
const int ORDER[6] = {
    0, 3, 4, 5, 1, 2
};

// Positions including missing sensors
const float POSITION[6] = {
    -3.5, -2.5, 0.5, 1.5, 2.5, 3.5
};

// White surface calibration
const int WHITE[6] = {
    1148, 881, 738, 1022, 917, 911
};

const int BLACK_THRESHOLD = 2300;
const int MIN_BLACK_SENSORS = 1;

const float CENTER = 1.10;

// =====================================
// LATTICE CROSSING SETTINGS
// =====================================

// How long to continue straight
// when the black line disappears
const unsigned long GAP_GRACE_MS = 150;

// Slower speed through the gap
const int GAP_SPEED = 20;

// =====================================
// TIMING SETTINGS
// =====================================

// Five-second autonomous test
const unsigned long AUTO_TIME = 20000;

// Delay before autonomous starts
const unsigned long START_DELAY = 3000;

unsigned long startTime = 0;
unsigned long gapStart = 0;
unsigned long allBlackSince = 0;

bool hasSeenLine = false;
bool finished = false;

// =====================================
// MOTOR CONTROL
// =====================================

void drive(int leftSpeed, int rightSpeed) {

    RedRobot::WriteDCMotor(
        LEFT_MOTOR,
        LEFT_FORWARD * leftSpeed
    );

    RedRobot::WriteDCMotor(
        RIGHT_MOTOR,
        RIGHT_FORWARD * rightSpeed
    );
}

void stopMotors() {
    drive(0, 0);
}

// =====================================
// SETUP
// =====================================

void setup() {

    Serial.begin(115200);

    // Initialize Bluetooth
    RedRobot::InitRedRobot();

    // Initialize motors and servo
    RedRobot::InitDCMotors();
    RedRobot::InitServo();

    // Initialize line sensors
    RedRobot::InitLineSensors();

    // Start autonomous mode
    RedRobot::InitAuton();

    stopMotors();

    delay(START_DELAY);

    startTime = millis();
}

// =====================================
// AUTONOMOUS LINE FOLLOWING
// =====================================

void AutonomousLoop() {

    // Bluetooth takeover
    if (RedRobot::IsRemoteControlled()) {
        return;
    }

    // Stay stopped after finishing
    if (finished) {
        delay(20);
        return;
    }

    unsigned long now = millis();

    // =================================
    // AUTONOMOUS TIME LIMIT
    // =================================

    if (now - startTime >= AUTO_TIME) {

        stopMotors();
        finished = true;
        return;
    }

    // =================================
    // READ SENSORS
    // =================================

    RedRobot::ReadLineSensor(sensors);

    // Check Bluetooth takeover again
    if (RedRobot::IsRemoteControlled()) {
        return;
    }

    float weightedSum = 0;
    float totalWeight = 0;

    int blackCount = 0;
    int saturatedCount = 0;

    // =================================
    // FIND BLACK LINE
    // =================================

    for (int i = 0; i < 6; i++) {

        int lf = ORDER[i];
        int reading = sensors[lf];

        if (reading >= 9000) {
            saturatedCount++;
        }

        if (reading >= BLACK_THRESHOLD) {

            blackCount++;

            float weight = constrain(
                (float)(reading - WHITE[lf]),
                0.0f,
                9000.0f
            );

            weightedSum +=
                POSITION[i] * weight;

            totalWeight += weight;
        }
    }

    now = millis();

    // =================================
    // NEW: CROSS A SMALL GAP
    // =================================

    if (blackCount < MIN_BLACK_SENSORS) {

        // Never start moving if the
        // robot has not detected a line
        if (!hasSeenLine) {

            stopMotors();
            finished = true;
            return;
        }

        // First reading with no line
        if (gapStart == 0) {
            gapStart = now;
        }

        // Continue straight briefly
        if (now - gapStart < GAP_GRACE_MS) {

            drive(GAP_SPEED, GAP_SPEED);
            return;
        }

        // No line after grace period:
        // stop permanently
        stopMotors();
        finished = true;
        return;
    }

    // =================================
    // LINE DETECTED AGAIN
    // =================================

    // Reset the gap timer so another
    // short gap can be crossed later
    gapStart = 0;

    hasSeenLine = true;

    // =================================
    // ALL SENSORS BLACK
    // =================================

    if (saturatedCount == 6) {

        if (allBlackSince == 0) {
            allBlackSince = now;
        }

        if (now - allBlackSince < 400) {
            drive(18, 18);
        }
        else {
            stopMotors();
            finished = true;
        }

        return;
    }

    allBlackSince = 0;

    // =================================
    // PROPORTIONAL LINE FOLLOWING
    // =================================

    if (totalWeight > 0) {

        float linePosition =
            weightedSum / totalWeight;

        float error =
            linePosition - CENTER;

        float correction = KP * error;

        int leftSpeed = constrain(
            (int)(BASE_SPEED + correction),
            MIN_SPEED,
            MAX_SPEED
        );

        int rightSpeed = constrain(
            (int)(BASE_SPEED - correction),
            MIN_SPEED,
            MAX_SPEED
        );

        drive(leftSpeed, rightSpeed);
    }
    else {

        stopMotors();
        finished = true;
    }

    delay(5);
}
