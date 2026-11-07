"""
Every dimension in millimeters. Edit this file, then run `python build_all.py`.

STATUS LEGEND
  MEASURED     measured on the real part by the team
  PLACEHOLDER  team estimate or Claude's assumption; replace before fabrication
  RULE         from the game manual or fabrication policy

The two sources disagree on the pipe: the game manual says a 3 in x 3 in cylinder
with flanged ends; the team's first look says flange OD ~1.5 in, length ~5 in,
mass ~200 g. The team's numbers are used here because they come from the physical
pipe, but every pipe-dependent dimension regenerates from this file in minutes.
"""

# ---------------------------------------------------------------------------
# Pipe (game piece)                                                PLACEHOLDER
# ---------------------------------------------------------------------------
PIPE_LENGTH      = 127.0   # 5 in, team estimate. Manual: 76.2 (3 in). DRIVES ARM REACH.
PIPE_FLANGE_OD   = 38.0    # 1.5 in, team estimate. Manual implies ~80+.
PIPE_BODY_OD     = 32.0    # assumption: must be smaller than flange OD; measure
PIPE_FLANGE_T    = 6.0     # assumption; measure
PIPE_MASS_G      = 200.0   # team estimate; design case 300 g

# ---------------------------------------------------------------------------
# Servos (40 g class). PLACEHOLDER: read the label, measure the body
# ---------------------------------------------------------------------------
SERVO_BODY_L     = 40.7    # along the tab direction
SERVO_BODY_W     = 19.7
SERVO_BODY_H     = 36.5    # bottom of body to underside of mounting tabs
SERVO_TAB_L      = 54.5    # overall length across both tabs
SERVO_TAB_T      = 2.5     # tab thickness
SERVO_TAB_HOLE_X = 49.5    # hole spacing along length
SERVO_TAB_HOLE_Y = 10.0    # hole spacing across width
SERVO_TAB_HOLE_D = 3.4     # M3 clearance (tabs are usually ~4.2; M3 with washer is fine)
SERVO_SHAFT_OFF  = 10.3    # shaft center offset from body center, along length
SERVO_BOSS_D     = 13.5    # raised boss around the output shaft
SERVO_BOSS_H     = 3.5     # boss height above the tab plane
SERVO_HORN_HUB_D = 8.0     # horn hub diameter that passes through a plate
SERVO_HORN_PCD   = 16.0    # bolt circle of the 4 horn screw holes (round / 4-arm horn)
SERVO_HORN_HOLE_D= 2.2     # horn screws
SERVO_STALL_KGCM = 5.0     # at 5 V. PLACEHOLDER until the Gate 1 bench test

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
ARM_LENGTH        = 230.0  # pivot to gripper-tip pivot hole, used with the 127 mm pipe
ARM_LENGTH_ALT    = 180.0  # second hole set, used with the 76 mm pipe
PIVOT_H_OPTIONS   = [165.0, 215.0, 265.0]   # pivot heights cut into the mast; pick with the real pipe
PIVOT_H           = 215.0                   # expected with a 127 mm pipe
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
FINGER_L          = 55.0   # pivot to pad center
FINGER_W          = 15.0   # along the pipe axis
FINGER_T          = 8.0
PAD_SIZE          = 15.0
PAD_DEPTH         = 2.0    # inner-tube rubber or craft foam
DIMPLE_D          = 8.0
DIMPLE_DEPTH      = 3.0
JAW_GAP_CLOSED    = PIPE_BODY_OD - 2.0    # pad faces when fully closed (clamp has squeeze)
JAW_GAP_OPEN_MIN  = PIPE_FLANGE_OD + 12.0 # must straddle the flange with margin
CARRIER_T         = 5.0
M3_CLEAR          = 3.4
M3_NUT_AF         = 5.6
M3_NUT_T          = 2.6
M2_CLEAR          = 2.4

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
