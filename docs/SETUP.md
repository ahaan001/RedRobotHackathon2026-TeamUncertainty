# Setup and flashing

Source: organizers' "RedRobot Coding Setup" doc and the Robot Assembly Guide. Verified against the firmware source.

## Toolchain (once per laptop)

1. Arduino IDE 2.x.
2. Boards Manager → search `esp32` → install **esp32 by Espressif Systems**, v3.0 or newer. Let it finish completely.
3. Library Manager → search `NimBLE-Arduino` → install **NimBLE-Arduino by h2zero**, 2.x.
4. Clone this repo. Open `RR26_Arduino/RR26_Arduino.ino`.
5. Board: **Adafruit Feather ESP32-S3 2MB PSRAM**. Port: the auto-detected one after plugging in USB-C.

## First flash

1. `CONFIGURE-ME.h`: set `TEAM_NUMBER` and `TEAM_NAME` (15 chars max). The robot does nothing until both are set. The Bluetooth name becomes `RR-<number>-<name>`; phones cache the old name for a few minutes after a change.
2. **Robot on blocks.** Put the arm in its STOW pose by hand (servos snap to the first command).
3. Compile (checkmark), then Upload (arrow). Open Serial Monitor at 115200.
4. Expect: `Reset reason`, `[DC_MOTOR] Initializing`, `[SERVO] Initializing`, `[INIT] AutonomousLoop will run`, `[MECH] task started`, `[TU] Team Uncertainty firmware ...`.
5. Battery display must read above 7.0 V at rest. Below that, swap the pack.

If the board is not detected: USB cable may be power-only; battery may be low; or hold **BOOT**, press and release **RESET** (display freezes), then upload.

## Controller

- **Redpad** (primary): https://redpad.roboclub.org in Chrome (Android) or Bluefy (iOS), landscape, enter team number, Connect. Redpad auto-reconnects after a board reset.
- **Bluefruit Connect** (fallback): Connect → Controller → Control Pad. Does **not** auto-reconnect after a reset unless the app is on the first controls menu.

## Serial diagnostics

Type into the Serial Monitor with newline ending. `h` prints help. Most useful:

| Command | Does |
| --- | --- |
| `p` | mode, arm and jaw targets, presets, line calibration |
| `s 0 1500` | arm servo to 1500 µs (within `ARM_PULSE_MIN..MAX`) |
| `S 0 700` | raw pulse ignoring limits, for finding end stops. Go in 50 µs steps and stop at the first buzz |
| `a 1` / `j 0` | arm preset FLOOR / jaw preset OPEN |
| `m 2 30` / `x` | left motor 30% / stop all. Robot on blocks |
| `l` / `L` / `c` | one line read / 5 s stream / 5 s calibration (slide across the tape) |
| `b` | place macro |

Do not use `l`, `L` or `c` while autonomous is running; the sensor read is not re-entrant.

## Servo torque test (Gate 1)

1. Shoulder servo on header 0, horn fitted, a 100 mm arm (ruler, plywood strip) on the horn.
2. `s 0 1500` to center. Hang a bag of nuts at 100 mm from the shaft.
3. Add weight until the servo cannot hold position or buzzes continuously. Torque in kg·cm = mass in kg × 10.
4. Record in `TESTLOG.md`. Required: about 5.2 kg·cm before counterbalance for a 150 g pipe; see plan section B.

## Tags

Every version that goes on the robot: `git tag -a v<n>-<what> -m "<what changed>"` and push the tag. To go back: `git checkout v1-teleop`.
