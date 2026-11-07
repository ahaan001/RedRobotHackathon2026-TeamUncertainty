# Red Robot 2026 — Team Uncertainty

Competition robot for the CMU Roboclub Red Robot 2026 hackathon (Nov 7–8, 2026).
Firmware is built on the organizers' `RR26_Arduino` starter (CMU Robotics Club); the
original commit history is preserved on `main` and tagged `v0-baseline`.

Engineering plan (architecture, schedule, task split, interface spec):
https://claude.ai/artifact/7vSyZSk65XwyzHfFZ37b6g

## Layout

| Path | What |
| --- | --- |
| `RR26_Arduino/` | The sketch. Open `RR26_Arduino.ino` in Arduino IDE 2.x |
| `RR26_Arduino/src/` | Organizers' library (`RedRobot.cpp`, `ble.h`, `pinouts.h`). **`ble.h` and the two `Parse()` functions are protected wireless routines. Do not edit them.** |
| `RR26_Arduino/Config.h` | Every tunable constant (servo presets, speeds, thresholds, auton timings) |
| `RR26_Arduino/Robot.h` | Declarations for our modules |
| `RR26_Arduino/*.cpp` | `Drive`, `Mechanism` (servo task), `LineFollow`, `Auton`, `Diagnostics` |
| `RR26_Arduino/RemoteControl.ino` | Redpad and Bluefruit button mapping, nothing else |
| `tools/LineSensorDiag/` | Standalone sketch that prints raw QTR-8RC values |
| `cad/` | CadQuery scripts and exports (STL, STEP, DXF), BOM, assembly notes |
| `docs/` | Setup, controls, architecture, interface spec, test log |

## Branches

- `main` — only code that has driven the robot. Every flash that goes on the robot gets a tag.
- `feature/teleop` — drivetrain, mechanism task, controller mapping, diagnostics (SW1)
- `feature/autonomous` — line following and the 20 s routine (SW2)
- `feature/mechanisms` — servo presets, macros, lever routine (SW1 with HW2)

Merge to `main` by pull request after: it compiles, `git diff main -- RR26_Arduino/src/ble.h`
is empty, the other software engineer has read the diff, and the previous tag still exists.

## Status

`FW_VERSION` in `Config.h` says what is flashed. Anything marked **uncompiled** has been
written but not yet built on a machine with the ESP32 toolchain. Nothing is called tested
until a tagged version has driven the robot and the result is in `docs/TESTLOG.md`.

Quick start: `docs/SETUP.md`. Controls: `docs/CONTROLS.md`.
