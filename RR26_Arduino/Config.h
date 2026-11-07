/**
 * @file  Config.h
 * @brief Every tunable constant for the Team Uncertainty robot lives here.
 *
 * This is the only file that should change after the software freeze.
 * Values tagged PLACEHOLDER have not been measured on the robot yet.
 * Servo values are stored in microseconds of pulse width because that is what
 * we measure on the bench; servoUnits() converts to the library's -180..180 units.
 */
#pragma once
#include <stdint.h>

#define FW_VERSION "v1-skeleton-hs311 (uncompiled)"

// ---------------------------------------------------------------------------
// Boot behaviour
// ---------------------------------------------------------------------------
// true:  robot boots idle in remote-control mode; Redpad A starts autonomous.
// false: organizer default, autonomous starts the moment the board boots
//        (use this if the match must be run on Bluefruit, which has no spare button).
constexpr bool BOOT_INTO_TELEOP = true;

// ---------------------------------------------------------------------------
// Drivetrain
// ---------------------------------------------------------------------------
constexpr uint8_t MOTOR_LEFT        = 2;     // per assembly guide: left motor on M2
constexpr uint8_t MOTOR_RIGHT       = 3;     // right motor on M3
constexpr int     MOTOR_LEFT_SIGN   = 1;     // set to -1 instead of swapping wires
constexpr int     MOTOR_RIGHT_SIGN  = 1;
constexpr int     DRIVE_DEADBAND    = 8;     // joystick counts ignored around zero
constexpr bool    DRIVE_EXPO        = true;  // squared input curve, slow near center
constexpr int     DRIVE_SCALE_FULL      = 100;  // percent
constexpr int     DRIVE_SCALE_PRECISION = 40;   // percent, toggled with D-pad CENTER

// Optional: stop the drive if no BLE command for this long while in teleop.
// OFF until we confirm whether Redpad streams joystick packets or only sends changes.
constexpr bool     TELEOP_WATCHDOG_ENABLED = false;
constexpr uint32_t TELEOP_CMD_TIMEOUT_MS   = 1500;

// ---------------------------------------------------------------------------
// Servo channels
// ---------------------------------------------------------------------------
constexpr uint8_t SERVO_ARM = 0;   // shoulder, header "servo 0" (verify silkscreen)
constexpr uint8_t SERVO_JAW = 1;   // gripper, header "servo 1"

// Servos are Hitec HS-311: 180 deg over 600..2400 us, 0.107 deg/us, clockwise with
// increasing pulse, 3.0 kg.cm at 4.8 V, 800 mA stall. Limits stay inside the servo's range.
constexpr int ARM_PULSE_MIN = 700;
constexpr int ARM_PULSE_MAX = 2300;
constexpr int JAW_PULSE_MIN = 700;
constexpr int JAW_PULSE_MAX = 2300;
constexpr int SERVO_US_PER_DEG_X100 = 935;   // 9.35 us per degree (HS-311 datasheet)

// ---- Arm presets are written in degrees above horizontal and converted to pulses ----
// Bench procedure: fit the horn so the arm is horizontal near 1500 us, then find the exact
// pulse where it is horizontal (`s 0 <us>` with a spirit level or by eye) -> ARM_HORIZONTAL_US.
// If the arm goes DOWN when the pulse increases, set ARM_DIR = -1. PLACEHOLDER until done.
constexpr int ARM_HORIZONTAL_US = 1500;
constexpr int ARM_DIR = 1;
constexpr int armUs(int deg) { return ARM_HORIZONTAL_US + ARM_DIR * deg * SERVO_US_PER_DEG_X100 / 100; }

// Angles from cad/arm.py reach table for the measured pipe (76 mm tall, 76 mm flange),
// arm holes at 180 mm, mast pivot at 165 mm. Re-run `python cad/arm.py` if the pipe differs.
enum ArmPreset : uint8_t {
  ARM_STOW = 0,  // folded, inside the 12 in footprint
  ARM_FLOOR,     // hooks at lying-pipe body height (38 mm)
  ARM_TOP,       // hooks just under the top flange of an upright pipe on the floor
  ARM_CARRY,     // travel height, pipe clear of the floor
  ARM_L1,        // place on 1 pipe
  ARM_L2,        // place on 2 pipes
  ARM_L3,        // place on 3 pipes
  ARM_L4,        // place on 4 pipes (at the reach limit; verify before relying on it)
  ARM_PRESET_COUNT
};
constexpr int ARM_PRESET_DEG[ARM_PRESET_COUNT] = { 70, -12, -3, 5, 16, 33, 52, 75 };
constexpr int ARM_PRESET_US[ARM_PRESET_COUNT] = {
  armUs(ARM_PRESET_DEG[0]), armUs(ARM_PRESET_DEG[1]), armUs(ARM_PRESET_DEG[2]), armUs(ARM_PRESET_DEG[3]),
  armUs(ARM_PRESET_DEG[4]), armUs(ARM_PRESET_DEG[5]), armUs(ARM_PRESET_DEG[6]), armUs(ARM_PRESET_DEG[7])
};
constexpr int ARM_SET_DROP_US     = 3 * SERVO_US_PER_DEG_X100 / 100;  // lower 3 deg (~12 mm) to set a pipe down
constexpr int ARM_JOG_US_PER_S    = 250;  // right joystick full deflection jog rate
constexpr int ARM_SLEW_US_PER_S   = 350;  // ~37 deg/s, limits swing and current on the HS-311

// ---- Jaw presets: the jaw gear is driven 1:1, so jaw angle = servo angle ----
// From cad/gripper.py: CLAMP = hooks touching the pipe body (0 deg), RIM = +2.5 deg (72 mm gap:
// body passes, flange does not), OPEN = +11 deg (>= 86 mm). Bench: close the jaws on the body
// with `s 1 <us>` -> JAW_CLAMP_US. If a larger pulse CLOSES the jaws, set JAW_DIR = -1.
constexpr int JAW_CLAMP_US = 1500;   // PLACEHOLDER
constexpr int JAW_DIR = 1;
constexpr int jawUs(int degOpen) { return JAW_CLAMP_US + JAW_DIR * degOpen * SERVO_US_PER_DEG_X100 / 100; }

enum JawPreset : uint8_t {
  JAW_OPEN = 0,  // straddles the flange
  JAW_RIM,       // hooks under the flange, pipe hangs and swings
  JAW_CLAMP,     // squeezes the body for horizontal carry
  JAW_PRESET_COUNT
};
constexpr int JAW_PRESET_US[JAW_PRESET_COUNT] = { jawUs(11), jawUs(3), jawUs(0) };
constexpr int JAW_SLEW_US_PER_S = 1200;

// Mechanism task timing
constexpr uint32_t MECH_TICK_MS         = 20;    // 50 Hz
constexpr uint32_t MACRO_SETTLE_MS      = 300;   // wait after arm reaches SET position
constexpr uint32_t MACRO_OPEN_WAIT_MS   = 400;   // wait after jaws open before lifting

// ---------------------------------------------------------------------------
// Line sensor (Pololu QTR-8RC on J1, six channels)
// ---------------------------------------------------------------------------
// ReadLineSensor returns RC decay time in microseconds. By the library code and
// the organizers' example data, black tape reads about 1000, white floor about
// 50..100, and about 10000 means the sensor sees nothing (too high off the floor).
// Confirm with the `l` command and flip this if the real board disagrees.
constexpr bool     LINE_DARK_IS_HIGH   = true;
constexpr uint16_t LINE_SATURATED_US   = 5000;  // above this: channel sees nothing
constexpr int      LINE_SATURATED_MAX  = 3;     // more channels than this saturated = off floor
constexpr bool     LINE_REVERSED       = false; // flip if position runs 5000 -> 0 left to right
constexpr uint16_t LINE_WHITE_DEFAULT[6] = { 80, 80, 80, 80, 80, 80 };      // PLACEHOLDER
constexpr uint16_t LINE_BLACK_DEFAULT[6] = { 1000, 1000, 1000, 1000, 1000, 1000 }; // PLACEHOLDER
constexpr int      LINE_ON_THRESHOLD      = 300;  // sum of normalized (0..1000 each)
constexpr int      LINE_END_MARKER_LEVEL  = 500;  // a channel above this counts as dark
constexpr int      LINE_END_MARKER_COUNT  = 5;    // this many dark channels = end marker

// Line following
constexpr int      LF_BASE_SPEED     = 40;   // percent
constexpr int      LF_KP             = 30;   // percent steer at full-scale error
constexpr uint32_t LF_LOST_TIMEOUT_MS = 400;
constexpr uint32_t LF_RECOVER_MS      = 500;
constexpr int      LF_RECOVER_SPEED   = 30;

// ---------------------------------------------------------------------------
// Autonomous (20 s). PLACEHOLDER timings until the field walk.
// ---------------------------------------------------------------------------
constexpr bool     AUTON_SIMPLE_EXIT_ONLY  = false;  // true: drive out of the box and stop (20 pts, no risk)
constexpr uint32_t AUTON_HARD_STOP_MS      = 19500;
constexpr int      AUTON_EXIT_SPEED        = 50;
constexpr uint32_t AUTON_EXIT_MIN_MS       = 300;    // ignore the start-box tape edge for this long
constexpr uint32_t AUTON_EXIT_MAX_MS       = 1500;   // go to FOLLOW anyway after this
constexpr uint32_t AUTON_FOLLOW_MAX_MS     = 12000;
constexpr uint32_t AUTON_ALIGN_DRIVE_MS    = 400;    // straight after the end marker
constexpr int      AUTON_ALIGN_DRIVE_SPEED = 35;
constexpr int      AUTON_ALIGN_TURN_SPEED  = 40;     // positive = clockwise, PLACEHOLDER
constexpr uint32_t AUTON_ALIGN_TURN_MS     = 0;      // 0 = no turn, set after field walk
constexpr uint32_t AUTON_RELEASE_SETTLE_MS = 800;
constexpr uint32_t AUTON_RELEASE_OPEN_MS   = 400;
constexpr int      AUTON_RETREAT_SPEED     = 40;
constexpr uint32_t AUTON_RETREAT_MS        = 600;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
// Library SetServo takes -180..180 mapped linearly onto 500..2500 us.
inline int servoUnits(int pulse_us) { return (pulse_us - 1500) * 360 / 2000; }
inline int clampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
