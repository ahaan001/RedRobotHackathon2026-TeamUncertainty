
#include <Arduino.h>
#include "RedRobot.h"

// Function defined in RR26_Arduino.ino
void BeginAutonomousRun();

// =====================================
// BLUETOOTH SPEED SETTINGS
// =====================================

const int REMOTE_MAX_SPEED = 60;
const int DPAD_FORWARD_SPEED = 45;
const int DPAD_TURN_SPEED = 30;
const int JOYSTICK_DEADZONE = 8;

bool useTankDrive = false;
int tankLeft = 0;
int tankRight = 0;

bool blueButtons[4] = {
    false, false, false, false
};

bool redButtons[5] = {
    false, false, false, false, false
};

// =====================================
// MOTOR CONTROL
// =====================================

void remoteDrive(int left, int right) {

    if (!RedRobot::IsRemoteControlled()) {
        return;
    }

    left = constrain(
        left, -REMOTE_MAX_SPEED, REMOTE_MAX_SPEED
    );

    right = constrain(
        right, -REMOTE_MAX_SPEED, REMOTE_MAX_SPEED
    );

    // Confirmed hardware directions
    RedRobot::WriteDCMotor(0, -left);
    RedRobot::WriteDCMotor(2, right);
}

void resetRemoteInputs() {

    for (int i = 0; i < 4; i++) {
        blueButtons[i] = false;
    }

    for (int i = 0; i < 5; i++) {
        redButtons[i] = false;
    }

    tankLeft = 0;
    tankRight = 0;
}

// =====================================
// START AUTONOMOUS
// =====================================

void startAutonomousFromButton() {

    if (!RedRobot::IsRemoteControlled()) {
        return;
    }

    // Stop manual commands first
    remoteDrive(0, 0);

    resetRemoteInputs();

    // Start 20-second autonomous run
    BeginAutonomousRun();
}

// =====================================
// EMERGENCY TAKEOVER / MANUAL MODE
// =====================================

void enableRemoteControl() {

    RedRobot::StartRemoteControl();

    remoteDrive(0, 0);

    resetRemoteInputs();

    Serial.println("REMOTE CONTROL ACTIVE");
}

// =====================================
// JOYSTICK SPEED CONVERSION
// =====================================

int joystickSpeed(int value) {

    if (abs(value) < JOYSTICK_DEADZONE) {
        return 0;
    }

    return constrain(
        value * REMOTE_MAX_SPEED / 100,
        -REMOTE_MAX_SPEED,
        REMOTE_MAX_SPEED
    );
}

// =====================================
// D-PAD DRIVING
// =====================================

void driveDPad(
    bool up,
    bool down,
    bool left,
    bool right
) {

    int forward = 0;
    int turn = 0;

    if (up) {
        forward += DPAD_FORWARD_SPEED;
    }

    if (down) {
        forward -= DPAD_FORWARD_SPEED;
    }

    if (left) {
        turn -= DPAD_TURN_SPEED;
    }

    if (right) {
        turn += DPAD_TURN_SPEED;
    }

    remoteDrive(
        forward + turn,
        forward - turn
    );
}

// =====================================
// BLUEFRUIT CONNECT
// =====================================

namespace Bluefruit {

    // BUTTON 1: Emergency takeover
    void Button1_Pressed(bool pressed) {

        if (pressed) {
            enableRemoteControl();
        }
    }

    // BUTTON 2: Servo 0
    void Button2_Pressed(bool pressed) {

        if (!RedRobot::IsRemoteControlled()) {
            return;
        }

        RedRobot::SetServo(
            0, pressed ? 45 : 0
        );
    }

    // BUTTON 3: START AUTONOMOUS
    void Button3_Pressed(bool pressed) {

        if (pressed) {
            startAutonomousFromButton();
        }
    }

    // BUTTON 4: Unused
    void Button4_Pressed(bool pressed) {
    }

    // D-PAD: Manual driving
    void DPad_Pressed(
        button_t button,
        bool pressed
    ) {

        if (!RedRobot::IsRemoteControlled()) {
            return;
        }

        if ((int)button < 0 ||
            (int)button > 3) {
            return;
        }

        blueButtons[button] = pressed;

        driveDPad(
            blueButtons[UP],
            blueButtons[DOWN],
            blueButtons[LEFT],
            blueButtons[RIGHT]
        );
    }
}

// =====================================
// REDPAD
// =====================================

namespace RR_Controller {

    // X: Emergency takeover
    void ButtonX_Pressed(bool pressed) {

        if (pressed) {
            enableRemoteControl();
        }
    }

    // B: START AUTONOMOUS
    void ButtonB_Pressed(bool pressed) {

        if (pressed) {
            startAutonomousFromButton();
        }
    }

    // Y: Arcade/tank toggle
    void ButtonY_Pressed(bool pressed) {

        if (!RedRobot::IsRemoteControlled()) {
            return;
        }

        if (pressed) {

            useTankDrive = !useTankDrive;

            resetRemoteInputs();

            remoteDrive(0, 0);
        }
    }

    // A: Servo 0
    void ButtonA_Pressed(bool pressed) {

        if (!RedRobot::IsRemoteControlled()) {
            return;
        }

        RedRobot::SetServo(
            0, pressed ? 45 : 0
        );
    }

    // D-PAD: Manual driving
    void DPad_Pressed(
        button_t button,
        bool pressed
    ) {

        if (!RedRobot::IsRemoteControlled()) {
            return;
        }

        if ((int)button < 0 ||
            (int)button > 4) {
            return;
        }

        redButtons[button] = pressed;

        driveDPad(
            redButtons[UP],
            redButtons[DOWN],
            redButtons[LEFT],
            redButtons[RIGHT]
        );
    }

    // Left joystick
    void Joystick1(int8_t x, int8_t y) {

        if (!RedRobot::IsRemoteControlled()) {
            return;
        }

        int forward = joystickSpeed(y);
        int turn = joystickSpeed(x);

        if (useTankDrive) {

            tankLeft = forward;

            remoteDrive(
                tankLeft,
                tankRight
            );
        }
        else {

            remoteDrive(
                forward + turn,
                forward - turn
            );
        }
    }

    // Right joystick for tank mode
    void Joystick2(int8_t x, int8_t y) {

        if (!RedRobot::IsRemoteControlled()) {
            return;
        }

        if (useTankDrive) {

            tankRight = joystickSpeed(y);

            remoteDrive(
                tankLeft,
                tankRight
            );
        }
    }
}
