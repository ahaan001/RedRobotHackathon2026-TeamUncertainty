"""
Gripper: two-jaw gear claw driven by the 40 g servo. Each jaw is a spur gear with a
finger; the two gears mesh so the jaws mirror. Pads on the finger tips carry a
conical dimple that seats the pipe's flange rim; the two dimples form the pendulum
axis that lets the pipe hang vertical.

Gripper frame (printed parts): X = jaw closing direction (world lateral), Y = finger
direction (toward the pipe), Z = gear axis (world fore-aft). The carrier plate lies in
XY at z in [-CARRIER_T, 0]; the servo hangs below it (z < -CARRIER_T) with its shaft
up through the plate at (+S/2, 0); the idler pin is at (-S/2, 0). Two ears on the
carrier (planes x = +-ARM_PLATE_GAP/2) bolt inside the arm plates.

Parts: jaw_driven, jaw_idler, carrier, idler_spacer. Pads: 15 x 15 x 2 mm inner tube
rubber or craft foam glued into the pockets.
"""
import math
import cadquery as cq
import params as P
from common import spur_gear, export_stl, export_step, bbox_str, print_mass_g

S = P.JAW_PIVOT_SPACING            # 36 mm between gear centers
Z0 = P.SERVO_BOSS_H                # gears sit this high above the carrier top (z=0)
GEAR_R_ADD = P.GEAR_MODULE * (P.GEAR_TEETH + 2) / 2
PAD_Y = P.FINGER_L                 # pad center along the finger
FINGER_END_Y = PAD_Y + P.PAD_SIZE / 2 + 2.0
INNER_X_CLOSED = P.JAW_GAP_CLOSED / 2   # pad face x when closed (+ side)
EAR_X = P.ARM_PLATE_GAP / 2        # outer face of each ear
EAR_T = 4.0
EAR_Y0, EAR_Y1 = -72.0, -30.0      # ear extent along Y (behind the gears)
EAR_Z1 = 26.0                      # ear height above the carrier top
EAR_PIVOT = (-48.0, 10.0)          # (y, z) of the gripper pivot hole in the ear
ANGLE_HOLE_R = 20.0
CARRIER_Y0, CARRIER_Y1 = -75.0, 22.0
CARRIER_X = EAR_X                  # carrier half-width = ear outer face


def jaw(driven: bool):
    """Hook jaw: gear at origin, finger toward +Y on the +X side. Mirror for the other jaw.

    Closed geometry (both jaws at CLAMP): the hook faces are JAW_GAP_CLOSED apart; the wide
    part of the finger, between the hook and the gears, clears the flange by HOOK_CLEAR per
    side so the flange can sit above the hooks. The flange underside rests on the hook's top
    face (the face toward the carrier) when the pipe hangs.
    """
    gear = spur_gear(P.GEAR_MODULE, P.GEAR_TEETH, P.GEAR_T, bore_d=0, backlash=P.GEAR_BACKLASH)

    hook_in = INNER_X_CLOSED - S / 2                 # hook face x, relative to the gear center
    wide_in = (P.PIPE_FLANGE_OD / 2 + P.HOOK_CLEAR) - S / 2   # wide-section inner face
    outer = wide_in + P.FINGER_T                     # outer face
    y_hook0 = PAD_Y - P.PAD_SIZE / 2                 # hook top face (flange rests here)
    y_end = FINGER_END_Y
    y_wide0 = 34.0                                   # where the wide section begins (after the crank)
    # finger outline (XY), extruded FINGER_W along Z
    pts = [(-3.0, 0.0), (5.0, 0.0), (5.0, 12.0),
           (outer, y_wide0 - 2.0), (outer, y_end),
           (hook_in, y_end), (hook_in, y_hook0),
           (wide_in, y_hook0), (wide_in, y_wide0),
           (-3.0, 16.0)]
    finger = cq.Workplane("XY").polyline(pts).close().extrude(P.FINGER_W)
    body = gear.union(finger)

    # pad pocket on the hook face
    pocket = (cq.Workplane("XY")
              .center(hook_in + P.PAD_DEPTH / 2, (y_hook0 + y_end) / 2)
              .rect(P.PAD_DEPTH, y_end - y_hook0 - 2.0).extrude(P.FINGER_W))
    body = body.cut(pocket)
    # chamfer the hook's leading edge so it slides under the flange
    try:
        body = body.edges("|Z").edges(cq.selectors.NearestToPointSelector((hook_in, y_end, P.FINGER_W / 2))).chamfer(1.5)
    except Exception:
        pass

    if driven:
        body = body.faces("<Z").workplane().circle(11.0).cutBlind(-2.5)      # round horn recess
        for pcd in (14.0, 20.0):
            pts4 = [(pcd / 2 * math.cos(math.radians(a)), pcd / 2 * math.sin(math.radians(a)))
                    for a in ((0, 90, 180, 270) if pcd == 14.0 else (45, 135, 225, 315))]
            body = body.faces(">Z").workplane().pushPoints(pts4).hole(P.SERVO_HORN_HOLE_D)
        body = body.faces(">Z").workplane().hole(P.M3_CLEAR)
    else:
        body = body.faces(">Z").workplane().hole(P.M3_CLEAR + 0.1)
    return body


def idler_spacer():
    """Ring under the idler gear so both gears sit at the same height (Z0)."""
    return cq.Workplane("XY").circle(6.0).circle(P.M3_CLEAR / 2 + 0.05).extrude(Z0)


def carrier():
    L = CARRIER_Y1 - CARRIER_Y0
    plate = (cq.Workplane("XY").center(0, (CARRIER_Y0 + CARRIER_Y1) / 2)
             .rect(2 * CARRIER_X, L).extrude(P.CARRIER_T).translate((0, 0, -P.CARRIER_T)))
    plate = plate.edges("|Z").fillet(4.0)

    # servo boss hole and tab holes; servo length along Y, shaft offset toward +Y
    plate = plate.faces(">Z").workplane().center(S / 2, 0).hole(P.SERVO_BOSS_D + 0.6)
    cy = -P.SERVO_SHAFT_OFF
    tabs = [(S / 2 + dx, cy + dy)
            for dx in (-P.SERVO_TAB_HOLE_Y / 2, P.SERVO_TAB_HOLE_Y / 2)
            for dy in (-P.SERVO_TAB_HOLE_X / 2, P.SERVO_TAB_HOLE_X / 2)]
    plate = plate.faces(">Z").workplane().pushPoints(tabs).slot2D(P.TAB_SLOT_LEN, P.SERVO_TAB_HOLE_D, angle=90).cutThruAll()
    # idler pin hole with a nut trap underneath
    plate = plate.faces(">Z").workplane().center(-S / 2, 0).hole(P.M3_CLEAR)
    trap = (cq.Workplane("XY").center(-S / 2, 0)
            .polygon(6, P.M3_NUT_AF / math.cos(math.radians(30))).extrude(P.M3_NUT_T)
            .translate((0, 0, -P.CARRIER_T)))
    plate = plate.cut(trap)

    # ears: plates perpendicular to X at x = +-EAR_X, from y EAR_Y0..EAR_Y1, z -CARRIER_T..EAR_Z1
    for sx in (-1, 1):
        ear = (cq.Workplane("XY")
               .box(EAR_T, EAR_Y1 - EAR_Y0, EAR_Z1 + P.CARRIER_T, centered=(True, False, False))
               .translate((sx * (EAR_X - EAR_T / 2), EAR_Y0, -P.CARRIER_T)))
        ear = ear.edges("|X").fillet(3.0)
        # pivot hole and three angle holes (beta = -10, 0, +10 deg) through the ear along X
        py, pz = EAR_PIVOT
        holes = [(py, pz)]
        for beta in (-10.0, 0.0, 10.0):
            b = math.radians(beta)
            holes.append((py - ANGLE_HOLE_R * math.cos(b), pz + ANGLE_HOLE_R * math.sin(b)))
        for (hy, hz) in holes:
            cyl = cq.Workplane("YZ").center(hy, hz).circle(P.M3_CLEAR / 2).extrude(2 * EAR_X + 4).translate((-EAR_X - 2, 0, 0))
            ear = ear.cut(cyl)
        plate = plate.union(ear)

    # gussets from ear to plate are implicit (ear extends below plate top by CARRIER_T)
    # lightening slot in the plate between servo and idler regions, behind the gears
    plate = plate.faces(">Z").workplane().center(0, -55).rect(24, 20).cutThruAll()
    return plate


def open_gap_check():
    """Hook-face gap and wide-section gap vs. opening angle, for preset sanity checks."""
    out = []
    wide_in = (P.PIPE_FLANGE_OD / 2 + P.HOOK_CLEAR) - S / 2
    for deg in range(0, 16, 3):
        a = math.radians(deg)
        rx, ry = (INNER_X_CLOSED - S / 2), PAD_Y
        xh = S / 2 + rx * math.cos(a) + ry * math.sin(a)
        xw = S / 2 + wide_in * math.cos(a) + 40.0 * math.sin(a)
        out.append((deg, round(2 * xh, 1), round(2 * xw, 1)))
    return out


if __name__ == "__main__":
    jd = jaw(True)
    ji = jaw(False).mirror("YZ")
    car = carrier()
    sp = idler_spacer()
    for name, shape in (("jaw_driven", jd), ("jaw_idler", ji), ("gripper_carrier", car), ("idler_spacer", sp)):
        export_stl(shape, name)
        export_step(shape, name)
        print(f"{name}: {bbox_str(shape)}, ~{print_mass_g(shape):.0f} g PLA")
    print(f"pivot spacing {S} mm, gear addendum radius {GEAR_R_ADD:.1f}, pad gap closed {P.JAW_GAP_CLOSED} mm")
    print("(deg, hook gap, wide gap):", open_gap_check())
    print(f"needs: OPEN hook gap >= {P.JAW_GAP_OPEN_MIN:.1f}, RIM = {P.JAW_GAP_RIM:.1f} (flange {P.PIPE_FLANGE_OD} must not pass, body {P.PIPE_BODY_OD} must), CLAMP = {P.JAW_GAP_CLOSED:.1f}")
