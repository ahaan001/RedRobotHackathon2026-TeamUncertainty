# Firmware architecture

```
                   NimBLE task                      mech task (50 Hz, core 1)            Arduino loop task
 Redpad/Bluefruit ──► ble.h ──► Parse() ──► RemoteControl.ino ──┐   Mechanism.cpp                  loop() in RedRobot.cpp
   (protected: ble.h, Parse)                      │             │     slews servo targets             │
                                                  │ sets targets├──►  runs place macro          in auton mode only
                                                  │ Drive::      │     polls Diagnostics              ▼
                                                  ▼             │     optional drive watchdog   AutonomousLoop()
                                              Drive.cpp ◄───────┘          │                        │
                                                  │                        ▼                   Auton.cpp (state machine)
                                                  ▼                 RedRobot::SetServo              │  uses LineFollow, Drive, Mech
                                           RedRobot::WriteDCMotor                                   ▼
                                                                                              LineFollow.cpp ──► RedRobot::ReadLineSensor
```

## Why this shape

- `RedRobot::WriteDCMotor` and `SetServo` refuse calls **from the Arduino loop task while in
  remote-control mode**. BLE callbacks and our own FreeRTOS task are not the loop task, so they
  pass in both modes. Auton runs on the loop task and is only active in auton mode, so it passes too.
- BLE callbacks never block: they set a target or write the drive and return.
- Servo motion is slewed in one place (`Mechanism.cpp`), so arm speed, current and pipe swing are
  controlled regardless of who requested the move.
- `Config.h` is the only file that changes after the software freeze.

## Modes

| Mode | `IsRemoteControlled()` | Who drives actuators |
| --- | --- | --- |
| Idle after boot (`BOOT_INTO_TELEOP`) | true | nobody until a command |
| Autonomous (after A) | false | `Auton::step()` via Drive/Mech; Mech task |
| Teleop (after X or auton finish) | true | BLE callbacks via Drive/Mech; Mech task |

## Library facts the code relies on (verified in `src/RedRobot.cpp`)

- `SetServo(s, a)`: `a` in −180..180 → 500..2500 µs. 0 = 1500 µs. `servoUnits()` converts from µs.
- `WriteDCMotor(d, s)`: d ∈ {0, 2, 3}; 1 is rejected (pin shared with LEDCTRL / TXD0).
- `ReadLineSensor(v)`: 6 channels, raw decay µs, 10 ms timeout per channel; blocks 7–60 ms.
- `Brake()` is `static` in the header: not callable from other files. Use `Drive::stop()`.
- `InitServo()` sends no pulse until the first `SetServo`.
