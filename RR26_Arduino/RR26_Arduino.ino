
#include <Arduino.h>
#include "RedRobot.h"

// =====================================
// MOTOR CONFIGURATION
// =====================================

const int LEFT_MOTOR = 0;
const int RIGHT_MOTOR = 2;

const int LEFT_FORWARD = -1;
const int RIGHT_FORWARD = 1;

// =====================================
// SPEED AND STEERING
// =====================================

const int BASE_SPEED = 35;
const int MIN_SPEED = 12;
const int MAX_SPEED = 55;

const float KP = 3.0;
const int GAP_SPEED = 25;

// =====================================
// LINE SENSOR CONFIGURATION
// =====================================

uint16_t sensors[6];

// Actual left-to-right order
const int ORDER[6] = {
    0, 3, 4, 5, 1, 2
};

const float POSITION[6] = {
    -3.5, -2.5, 0.5, 1.5, 2.5, 3.5
};

const int WHITE[6] = {
    1148, 881, 738, 1022, 917, 911
};

const int BLACK_THRESHOLD = 2300;
const int MIN_BLACK_SENSORS = 1;

const float CENTER = 1.10;

// =====================================
// GAP CROSSING
// =====================================

const unsigned long GAP_GRACE_MS = 150;

// =====================================
// AUTONOMOUS TIMING
// =====================================

// 20 seconds, starting at button press
const unsigned long AUTO_TIME = 20000;

unsigned long startTime = 0;
unsigned long gapStart = 0;
unsigned long allBlackSince = 0;

bool hasSeenLine = false;
bool finished = false;

// =====================================
// MOTOR FUNCTIONS
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
// START AUTONOMOUS FROM BLUETOOTH
// =====================================

// Called by RemoteControl.ino when
// Bluefruit button 3 is pressed.

void BeginAutonomousRun() {

    // Only allow starting from
    // Bluetooth/manual mode
    if (!RedRobot::IsRemoteControlled()) {
        return;
    }

    stopMotors();

    // Reset line-following state
    hasSeenLine = false;
    finished = false;

    gapStart = 0;
    allBlackSince = 0;

    // Start the 20-second timer NOW
    startTime = millis();

    // Enable AutonomousLoop
    RedRobot::InitAuton();

    Serial.println("AUTONOMOUS STARTED");
}

// =====================================
// SETUP
// =====================================

void setup() {

    Serial.begin(115200);

    // Initialize Bluetooth system
    RedRobot::InitRedRobot();

    // Initialize hardware
    RedRobot::InitDCMotors();
    RedRobot::InitServo();
    RedRobot::InitLineSensors();

    stopMotors();

    // IMPORTANT:
    // Start in Bluetooth mode.
    // Robot waits for button 3.
    RedRobot::StartRemoteControl();

    Serial.println("READY");
    Serial.println("Press Bluefruit 3 to start");
}

// =====================================
// AUTONOMOUS LOOP
// =====================================

void AutonomousLoop() {

    // No autonomous movement when
    // remote control is active
    if (RedRobot::IsRemoteControlled()) {
        return;
    }

    unsigned long now = millis();

    // =================================
    // END THE 20-SECOND PERIOD
    // =================================

    // Check BEFORE finished so that
    // Bluetooth becomes available even
    // if the robot reached the tape end.

    if (now - startTime >= AUTO_TIME) {

        stopMotors();
        finished = true;

        // Automatically enable Bluetooth
        RedRobot::StartRemoteControl();

        Serial.println("20 SECONDS COMPLETE");
        Serial.println("BLUETOOTH CONTROL ACTIVE");

        return;
    }

    // If tape ended, remain stopped
    // until 20 seconds have elapsed.
    if (finished) {
        delay(20);
        return;
    }

    // =================================
    // READ LINE SENSORS
    // =================================

    RedRobot::ReadLineSensor(sensors);

    if (RedRobot::IsRemoteControlled()) {
        return;
    }

    float weightedSum = 0;
    float totalWeight = 0;

    int blackCount = 0;
    int saturatedCount = 0;

    // =================================
    // DETECT THE BLACK LINE
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
    // CROSS SMALL WHITE/LATTICE GAP
    // =================================

    if (blackCount < MIN_BLACK_SENSORS) {

        if (!hasSeenLine) {
            stopMotors();
            finished = true;
            return;
        }

        if (gapStart == 0) {
            gapStart = now;
        }

        if (now - gapStart < GAP_GRACE_MS) {

            drive(GAP_SPEED, GAP_SPEED);
            return;
        }

        // Gap lasted too long:
        // stop driving permanently
        stopMotors();
        finished = true;
        return;
    }

    // Line detected again
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
