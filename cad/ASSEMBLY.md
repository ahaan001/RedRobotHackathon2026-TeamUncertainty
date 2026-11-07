# Assembly and pre-flight

Read `BOM.md` first. Frames: world X forward, Y lateral, Z up. Arm angle φ is measured above
horizontal; the fingers point 47° below the arm line.

## Configurations (one mast, one arm plate set, two pipe hypotheses)

| Pipe | Mast pivot hole | Arm gripper holes | Arm angles: floor pick → place on 2 | Ceiling |
| --- | --- | --- | --- | --- |
| Manual, 76 mm long, flange ~85 | 165 mm | 180 mm set | −11° → +33° (place on 3 at +52°) | 4-stack |
| Team estimate, 127 mm, flange 38 | 215 mm | 230 mm set | −25° → +49° | 3-stack (place on 3 out of reach) |

Pick the row that matches the measured pipe. The servo bolts at that pivot hole and the gripper at that hole set.

## Order

1. **Mast.** Dry-fit the side plates into the base slots, then the back brace and top brace. Glue every tab joint (wood glue or CA). Keep square while it sets.
2. **Shoulder servo.** Inside face of the LEFT plate, boss through the hole at the chosen pivot height, four tab screws. Shaft points outward (+Y).
3. **Mast to chassis.** Four M3 × 12 through the base slots into the kit top plate (drill the top plate to match, or use existing holes if they line up). Start with the mast as far **back** as the slots allow; see the tipping check below.
4. **Arm.** Two spacers between the plates at the 60 and 140 mm holes, M3 × 12 from each side into the nut traps. Horn screws through the left plate into the horn. Bushing in the right plate.
5. **Arm to servo.** Power the board, `s 0 1500` in the serial monitor (servo at center). Fit the horn on the spline so the arm points about **+20° above horizontal** at center. This puts the full −30° to +75° sweep inside the servo's travel. Tighten the horn screw. M3 × 25 with washers through the right mast plate, the bushing, nut on the outside.
6. **Gripper.** Jaw servo under the carrier, four tab screws, shaft up through the boss hole. Horn on the shaft. Driven jaw onto the horn (recess fits the round horn; drill the gear if the horn pattern differs). Idler: M3 × 16 from below (head in the nut trap), idler spacer, idler jaw, nut on top, snug but free to turn. Mesh the gears with both fingers parallel and the pads 30 mm apart (placeholder CLAMP gap).
7. **Gripper to arm.** Ears go inside the arm plates at the chosen hole set. M3 × 12 through the arm plate into the ear pivot hole, second one through the angle hole (middle of the three first).
8. **Pads.** Glue rubber or foam into the pad pockets; the dimple stays uncovered (cut a hole in the pad or let the rubber stretch into it).
9. **Counterbalance.** Rubber bands from the arm's rear hook hole to the top-brace notches. Add bands until the arm, with gripper and no pipe, floats roughly level when the servo is unpowered.
10. **Sensor bracket.** Two M3 through the baseplate front edge; QTR under the shelf with M2 screws, sensors down, header through the window. Check 3 to 5 mm to the floor with the robot on its wheels.
11. **Plow (Level 1).** Tab the walls into the base, glue, bolt to the baseplate front holes. Remove when the arm is on if they conflict.

## Pre-flight, every power-on

- Arm in STOW by hand (about +70°) before power. Jaws closed on nothing or on the preload.
- `p` in the serial monitor: targets match the presets, mode is TELEOP.
- `a 1` (FLOOR), `a 4` (L1), `a 0` (STOW): arm moves smoothly, no buzzing at the ends. If it buzzes, the preset is past the mechanical limit: fix `ARM_PULSE_MIN/MAX` in `Config.h`.
- `j 0`, `j 1`, `j 2`: jaws open, rim, clamp. Hand-check the gaps against the pipe.
- Rim test: hang a pipe from the jaws at RIM and lift the arm slowly. The pipe must swing to vertical and settle within 3 s. If it slips out, the RIM gap is too wide; if it will not swing, the dimples are binding or the gap is too tight.

## Tipping check (do this at Gate 1 with the real pipe)

Hold the pipe in the jaws, arm at the "place on 1" preset, robot on the floor. Push down gently
on the jaws with one finger (about 1 N). If the rear wheels unload or the caster lifts, the robot
is marginal. Fixes, in order: slide the mast to the rear of its slots; move the battery pack to
the rear of the bay; bolt the spare TT motor and the plow at the rear as ballast; shorten to the
180 mm hole set with the 265 mm pivot (less reach forward: 90 mm at floor pick).

## What changes when the measurements come in

Edit `params.py` (pipe block, servo block, chassis block), run `python build_all.py`, then
`python assembly_check.py`, and look at `out/assembly_check.png` before sending anything to a
machine. The jaws, carrier and arm plate are the parts that move; the mast does not.
