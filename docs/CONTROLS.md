# Controls

Source of truth is `RR26_Arduino/RemoteControl.ino`. Change the mapping there, then update this file.

## Redpad (primary)

| Input | Function |
| --- | --- |
| Left joystick | Arcade drive, squared input curve, deadband 8. Full 100%, precision 40% |
| Right joystick Y | Arm jog (fine height). Releasing stops the jog |
| D-pad UP / DOWN | Arm preset up / down: FLOOR → TOP → CARRY → L1 → L2 → L3 → L4 |
| D-pad LEFT | Jaws OPEN |
| D-pad RIGHT | Jaws next closed preset: RIM, then CLAMP, then RIM ... |
| D-pad CENTER | Precision drive toggle |
| A | Start autonomous. Only works from idle teleop and only once per boot |
| B | Place macro: lower one step, open jaws, lift back to the level |
| X | Exit autonomous, enter teleop (stock) |
| Y | Stow: arm to STOW, drive stop |

## Bluefruit Connect (fallback)

| Input | Function |
| --- | --- |
| D-pad | Drive (forward/back 100, turn 50), stock behaviour |
| 1 | Exit autonomous (stock) |
| 2 | Jaws: open if closed, RIM if open |
| 3 | Arm preset up |
| 4 | Arm preset down |

Bluefruit has no spare button to start autonomous. If a match must be driven on Bluefruit,
set `BOOT_INTO_TELEOP = false` in `Config.h` so autonomous starts at boot (organizer default),
and reset the board at "GO".

## Match procedure (Redpad)

1. Arm in STOW by hand. Power on. Display ≥ 7.3 V.
2. Connect Redpad. Robot is idle in teleop.
3. At "GO": press **A**. Autonomous runs up to 19.5 s and hands over to teleop by itself.
4. If anything looks wrong during autonomous: press **X**.
5. Teleop: drive, D-pad for presets, B to place, Y to stow before driving fast.

## Driver rules

- Pause two seconds after lifting a pipe before driving; let the swing settle.
- Precision mode on before approaching the stack.
- Never cross to the opposing Steam Plant (−150).
- Lever only in the last 30 s (per slides; confirm with organizers).
