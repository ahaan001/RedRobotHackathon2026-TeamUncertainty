# Bill of materials and fabrication list

Generated from `params.py` on 2026-11-07. **All pipe dimensions are placeholders** (team
estimate: flange OD 38 mm, length 127 mm, 200 g; the manual says 3 in × 3 in). Everything in
`out/` regenerates from `python build_all.py` once the measurements in `docs/TESTLOG.md` are filled.
Do not print the gripper or cut the arm before the pipe is measured; the mast, sensor bracket
and plow do not depend on the pipe and can be made now.

## 3D prints (PLA, 15% infill, draft layer, flat side down)

| File | Qty | Approx. mass | Depends on pipe? | Notes |
| --- | --- | --- | --- | --- |
| `out/sensor_bracket.stl` | 1 | 6 g | no | Depends on baseplate height (placeholder 18 mm). Shim with washers if off |
| `out/arm_spacer.stl` | 2 | 5 g each | no | M3 nut trap each end, M3 × 12 screw from each side |
| `out/arm_pivot_bushing.stl` | 1 | 1 g | no | 8 mm OD into the right arm plate, M3 × 25 bolt through it into the mast |
| `out/jaw_driven.stl` | 1 | 7 g | **yes** | Gear + finger + horn recess. Print flat (gear face down) |
| `out/jaw_idler.stl` | 1 | 8 g | **yes** | Mirror of the driven jaw, plain M3 bore |
| `out/idler_spacer.stl` | 1 | <1 g | no | Under the idler gear, matches the horn boss height |
| `out/gripper_carrier.stl` | 1 | 23 g | partly (ear geometry) | Servo under the plate, ears bolt inside the arm plates |

Two print jobs: **A** = carrier + both jaws + idler spacer (~38 g, ~1.3 h). **B** = bracket + 2 spacers + bushing (~17 g, ~40 min). Both within the 100 g / 2.5 h limit.

## Laser cuts (1/8 in plywood, confirm thickness; DXF in `out/`)

| File | Qty | Size | Depends on pipe? | Notes |
| --- | --- | --- | --- | --- |
| `mast_side_left.dxf` | 1 | 60 × 229 | no | Servo boss hole + 4 tab holes at three pivot heights (165 / 215 / 265 mm above floor) |
| `mast_side_right.dxf` | 1 | 60 × 229 | no | M3 free-pivot hole at the same three heights |
| `mast_base.dxf` | 1 | 110 × 92 | no | Four 20 mm slots to the kit top plate; tab slots for the side plates |
| `mast_back_brace.dxf` | 1 | 52 × 131 | no | Two tabs each side |
| `mast_top_brace.dxf` | 1 | 52 × 24 | no | Rubber-band notches |
| `arm_plate.dxf` | 2 | 321 × 92 | **partly** | Gripper holes at both 180 and 230 mm; horn PCD 16 mm (check against the real horn) |
| `plow_base.dxf` | 1 | 160 × 120 | no | Bolts to the baseplate front holes |
| `plow_wall.dxf` | 2 | 103 × 57 | no | Tab into the base |

## Hardware (kit bag and Roboclub stock)

| Item | Qty | Use |
| --- | --- | --- |
| M3 × 12 | 16 | mast base to top plate (4), servo tabs to mast (4), arm spacers (4), ear bolts (4) |
| M3 × 16 | 2 | idler pin (1), spare |
| M3 × 25 | 1 | right arm pivot through bushing and mast plate |
| M3 nuts | 20 | |
| M3 washers | 10 | both sides of the free pivot, under ear bolts |
| Servo horn screws (supplied with servos, ~M2) | 8 | horn to arm plate (4), horn to driven gear (4) |
| M2 × 8 + nuts | 2 | QTR-8RC to the sensor shelf |
| Rubber bands | 4 to 6 | counterbalance, arm rear hook to top brace notches |
| Inner tube or craft foam | 2 pcs 15 × 15 mm | jaw pads, glued in the pockets |
| Wood glue or CA | | mast tab joints |
| Zip ties | 6 | servo and sensor cable to the arm |

## Not yet generated

- Lever actuator: geometry unknown until measured.
- Dispense-button bumper: the plow front edge is the bumper until the button is measured.
- A dedicated ballast bracket, if the tipping check fails (see `ASSEMBLY.md`).
