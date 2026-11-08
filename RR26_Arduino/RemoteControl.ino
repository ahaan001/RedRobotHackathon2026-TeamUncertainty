
#include <Arduino.h>
#include "RedRobot.h"

// =====================================
// REMOTE CONTROL CONFIGURATION
// =====================================

const int REMOTE_MAX_SPEED = 60;
const int DPAD_FORWARD_SPEED = 45;
const int DPAD_TURN_SPEED = 30;
const int JOYSTICK_DEADZONE = 8;

bool useTankDrive = false;
int tankLeft = 0;
int tankRight = 0;

// =====================================
// MOTOR CONTROL
// =====================================

// Takes physical left/right wheel speeds.
// Corrects Motor 0's reversed direction.

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

    // LEFT: Motor 0
    RedRobot::WriteDCMotor(0, -left);

    // RIGHT: Motor 2
    RedRobot::WriteDCMotor(2, right);
}

// Convert joystick input -100..100
// into our preferred motor speed range.

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

// Common D-pad driving function

void driveDPad(
    bool up, bool down,
    bool left, bool right
) {

    int forward = 0;
    int turn = 0;

    if (up) forward += DPAD_FORWARD_SPEED;
    if (down) forward -= DPAD_FORWARD_SPEED;

    if (left) turn -= DPAD_TURN_SPEED;
    if (right) turn += DPAD_TURN_SPEED;

    remoteDrive(
        forward + turn,
        forward - turn
    );
}

// Switch from autonomous to Bluetooth

void enableRemoteControl() {

    if (RedRobot::IsRemoteControlled()) {
        return;
    }

    RedRobot::StartRemoteControl();

    // Stop autonomous motor movement
    remoteDrive(0, 0);

    tankLeft = 0;
    tankRight = 0;

    Serial.println("BLUETOOTH CONTROL ACTIVE");
}

// =====================================
// BLUEFRUIT CONNECT
// =====================================

namespace Bluefruit {

    // Button 1 exits autonomous mode
    void Button1_Pressed(bool pressed) {

        if (pressed) {
            enableRemoteControl();
        }
    }

    // D-pad controls driving
    void DPad_Pressed(
        button_t button,
        bool pressed
    ) {

        if (!RedRobot::IsRemoteControlled()) {
            return;
        }

        static bool buttons[4] = {
            false, false, false, false
        };

        if ((int)button < 0 || (int)button > 3) {
            return;
        }

        buttons[button] = pressed;

        driveDPad(
            buttons[UP],
            buttons[DOWN],
            buttons[LEFT],
            buttons[RIGHT]
        );
    }

    // Button 2 controls Servo 0
    // Press: 45 degrees
    // Release: 0 degrees
    void Button2_Pressed(bool pressed) {

        if (!RedRobot::IsRemoteControlled()) {
            return;
        }

        RedRobot::SetServo(
            0, pressed ? 45 : 0
        );
    }

    void Button3_Pressed(bool pressed) {
    }

    void Button4_Pressed(bool pressed) {
    }
}

// =====================================
// REDPAD CONTROLLER
// =====================================

namespace RR_Controller {

    // Button X exits autonomous mode
    void ButtonX_Pressed(bool pressed) {

        if (pressed) {
            enableRemoteControl();
        }
    }

    // Button Y toggles arcade/tank drive
    void ButtonY_Pressed(bool pressed) {

        if (!RedRobot::IsRemoteControlled()) {
            return;
        }

        if (pressed) {

            useTankDrive = !useTankDrive;

            tankLeft = 0;
            tankRight = 0;

            remoteDrive(0, 0);
        }
    }

    // Button A controls Servo 0
    void ButtonA_Pressed(bool pressed) {

        if (!RedRobot::IsRemoteControlled()) {
            return;
        }

        RedRobot::SetServo(
            0, pressed ? 45 : 0
        );
    }

    void ButtonB_Pressed(bool pressed) {
    }

    // RedPad D-pad
    void DPad_Pressed(
        button_t button,
        bool pressed
    ) {

        if (!RedRobot::IsRemoteControlled()) {
            return;
        }

        static bool buttons[5] = {
            false, false, false, false, false
        };

        if ((int)button < 0 || (int)button > 4) {
            return;
        }

        buttons[button] = pressed;

        driveDPad(
            buttons[UP],
            buttons[DOWN],
            buttons[LEFT],
            buttons[RIGHT]
        );
    }

    // Left joystick:
    // Arcade = forward and turning
    // Tank = left wheel only
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

    // Right joystick:
    // Used in tank drive mode
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
