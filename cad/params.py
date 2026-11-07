"""
Every dimension in millimeters. Edit this file, then run `python build_all.py`.

STATUS LEGEND
  MEASURED     measured on the real part by the team
  PLACEHOLDER  team estimate or Claude's assumption; replace before fabrication
  RULE         from the game manual or fabrication policy

Pipe measured 2026-11-07 15:10: flange OD 3 in, body OD 2.75 in, flange 0.5 in thick,
75 g. Standing height not yet measured; 3 in assumed per the manual. The flange is only
3.2 mm proud of the body, so the jaws hold the pipe by hooking under the top flange
(a geometric lip), not by friction dimples.
"""

# ---------------------------------------------------------------------------
# Pipe (game piece)                                     MEASURED except length
# ---------------------------------------------------------------------------
PIPE_LENGTH      = 76.2    # 3 in per the manual. NOT YET MEASURED: confirm standing height. DRIVES ARM REACH.
PIPE_FLANGE_OD   = 76.2    # 3 in        MEASURED 2026-11-07 15:10
PIPE_BODY_OD     = 69.85   # 2.75 in     MEASURED
PIPE_FLANGE_T    = 12.7    # 0.5 in      MEASURED
PIPE_MASS_G      = 75.0    # MEASURED

# ---------------------------------------------------------------------------
# Servos: Hitec HS-311 (label read 2026-11-07). Datasheet: 40 x 20 x 36.5 mm, 43 g,
# 3.0 kg.cm at 4.8 V / 3.5-3.7 at 6 V, 180 deg over 600-2400 us, 24T C1 spline, 800 mA stall.
# Tab hole spacing and shaft offset still PLACEHOLDER (standard-size values): holes are slotted.
# ---------------------------------------------------------------------------
SERVO_BODY_L     = 40.0    # along the tab direction (HS-311 datasheet)
SERVO_BODY_W     = 20.0
SERVO_BODY_H     = 36.5    # bottom of body to underside of mounting tabs
SERVO_TAB_L      = 54.5    # overall length across both tabs
SERVO_TAB_T      = 2.5     # tab thickness
SERVO_TAB_HOLE_X = 49.5    # hole spacing along length
SERVO_TAB_HOLE_Y = 10.0    # hole spacing across width
SERVO_TAB_HOLE_D = 3.4     # M3 clearance; cut as 3.4 x 6 slots to absorb the unknown tab spacing
SERVO_SHAFT_OFF  = 10.0    # shaft center offset from body center, along length. PLACEHOLDER
SERVO_BOSS_D     = 13.5    # raised boss around the output shaft
SERVO_BOSS_H     = 3.5     # boss height above the tab plane
SERVO_HORN_HUB_D = 8.0     # horn hub diameter that passes through a plate
SERVO_HORN_PCD   = 16.0    # bolt circle of the 4 horn screw holes (round / 4-arm horn)
SERVO_HORN_HOLE_D= 2.2     # horn screws
SERVO_STALL_KGCM = 3.1     # HS-311 at 5 V, interpolated from the datasheet

# ---------------------------------------------------------------------------
# Kit chassis. PLACEHOLDER: measure the plates
# ---------------------------------------------------------------------------
PLY_T                 = 3.2    # 1/8 in plywood, Roboclub stock. RULE (recommended)
KIT_PLATE_T           = 3.2    # kit wood thickness; measure
TOP_PLATE_H           = 75.0   # top surface of the kit top plate above the floor
TOP_PLATE_W           = 150.0  # width (lateral) of the kit top plate
BASEPLATE_UNDER_H     = 18.0   # underside of baseplate above the floor
BASEPLATE_FRONT_HOLE_SPACING = 20.0  # two holes we drill at the front edge for the sensor bracket

# ---------------------------------------------------------------------------
# Line sensor (Pololu QTR-8RC). Board 74.9 x 12.7 x 1.6 from the product page
# ---------------------------------------------------------------------------
QTR_BOARD_L       = 74.9
QTR_BOARD_W       = 12.7
QTR_BOARD_T       = 1.6
QTR_HOLE_SPACING  = 70.0   # PLACEHOLDER: measure the two mounting holes
QTR_HOLE_D        = 2.4    # #2 screws / M2
QTR_SENSE_HEIGHT  = 4.0    # emitter face above the floor. Pololu optimum 3, max 9.5
QTR_COMPONENT_H   = 3.0    # height of sensor packages below the board

# ---------------------------------------------------------------------------
# Arm geometry (derived from the pipe). See plan section B for the method.
# ---------------------------------------------------------------------------
CLEAR_ABOVE_STACK = 10.0
# Jaw contact (flange rim) heights the arm must reach:
H_FLOOR_PICK   = PIPE_FLANGE_OD / 2                       # lying pipe, near flange center
H_TOP_PICK     = PIPE_LENGTH                              # upright pipe, top flange
def h_place(n):  # jaw height to place onto a stack of n pipes
    return n * PIPE_LENGTH + CLEAR_ABOVE_STACK + PIPE_LENGTH
TARGET_STACK   = 3                                        # pipes placed by the arm (L1..L3 presets)

# The jaw contact (dimple) line sits GRIPPER_OFFSET beyond the arm-tip pivot, in the finger
# direction, which is GRIPPER_MOUNT_ANGLE_DEG below the arm line. The pivot-to-contact radius is
# therefore an effective link: L_eff = |L + G*e^(i*alpha)|, about 260 mm for L=180, G=103.
# contact_height(phi) = PIVOT_H + L*sin(phi) + G*sin(phi + alpha); see common.reach_table().
#   76 mm pipe (manual, flange ~85): L 180, pivot 165 -> floor -11 deg .. place on 3 +52 deg (4-stack)
#  127 mm pipe (team estimate):      L 230, pivot 215 -> floor -25 deg .. place on 2 +49 deg (3-stack)
ARM_LENGTH        = 180.0  # pivot to gripper-tip pivot hole, for the 76 mm pipe
ARM_LENGTH_ALT    = 230.0  # second hole set, only if the pipe turns out ~127 mm
PIVOT_H_OPTIONS   = [165.0, 215.0, 265.0]   # pivot heights cut into the mast; pick with the real pipe
PIVOT_H           = 165.0                   # for the 76 mm pipe
ARM_PLATE_W       = 30.0
ARM_REAR_EXT      = 45.0   # behind the pivot, for the counterbalance hook
ARM_PLATE_GAP     = 58.4   # inside distance between arm plates = MAST_INNER_GAP + 2*PLY_T + 2*3 (horn/washer)
GRIPPER_MOUNT_SPACING = 24.0   # two holes at the arm tip
GRIPPER_MOUNT_ANGLE_DEG = -47.0  # finger direction relative to the arm line (down = negative)
GRIPPER_EAR_PIVOT_Y     = -48.0  # gripper frame: arm pivot bolt position behind the gear line
GRIPPER_EAR_PIVOT_Z     = 10.0   # ... and above the carrier top

# ---------------------------------------------------------------------------
# Mast
# ---------------------------------------------------------------------------
MAST_INNER_GAP    = 46.0   # between side plates; servo body (36.5 + tabs) fits inside
MAST_PLATE_W      = 60.0
MAST_BASE_L       = 110.0  # base plate length (fore-aft)
MAST_BASE_W       = MAST_INNER_GAP + 2 * PLY_T + 2 * 20.0
MAST_BASE_BOLT_SPACING_X = 80.0
MAST_BASE_BOLT_SPACING_Y = 60.0
TAB_LEN           = 12.0
KERF              = 0.1    # laser kerf compensation per side for slots

# ---------------------------------------------------------------------------
# Gripper
# ---------------------------------------------------------------------------
GEAR_MODULE       = 1.5
GEAR_TEETH        = 24     # pitch diameter 36 = pivot spacing
GEAR_T            = 6.0
GEAR_BACKLASH     = 0.3
JAW_PIVOT_SPACING = GEAR_MODULE * GEAR_TEETH
FINGER_L          = 55.0   # pivot to pad (hook) center
FINGER_W          = 15.0   # finger height along the gear axis (world fore-aft)
FINGER_T          = 8.0    # finger wall thickness
PAD_SIZE          = 15.0   # hook depth along the finger
PAD_DEPTH         = 2.0    # inner-tube rubber or craft foam, in the hook face
HOOK_CLEAR        = 3.0    # wide part of the finger clears the flange by this per side
JAW_GAP_CLOSED    = PIPE_BODY_OD - 2.0    # hook faces when fully closed (CLAMP, pads squeezed)
JAW_GAP_RIM       = PIPE_BODY_OD + 2.0    # hooks under the flange, body free to swing (RIM)
JAW_GAP_OPEN_MIN  = PIPE_FLANGE_OD + 10.0 # must straddle the flange with margin (OPEN)
CARRIER_T         = 5.0
M3_CLEAR          = 3.4
M3_NUT_AF         = 5.6
M3_NUT_T          = 2.6
M2_CLEAR          = 2.4
TAB_SLOT_LEN      = 6.0    # servo tab holes cut as slots this long

# ---------------------------------------------------------------------------
# Plow (Level 1 fallback)
# ---------------------------------------------------------------------------
PLOW_W            = 160.0
PLOW_DEPTH        = 90.0
PLOW_WALL_H       = 45.0
PLOW_V_ANGLE_DEG  = 120.0

# ---------------------------------------------------------------------------
# Fabrication rules                                                      RULE
# ---------------------------------------------------------------------------
PRINT_MAX_G       = 100
PRINT_MAX_MIN     = 150
PLA_DENSITY_G_CM3 = 1.24
PRINT_INFILL      = 0.15
