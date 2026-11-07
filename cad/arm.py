"""
Arm: two identical 1/8 in plywood plates straddling the mast, joined by two printed
spacers. Pivot end: 8 mm center hole (servo horn hub on the left, printed bushing on
the right) plus four horn screw holes. Tip: gripper pivot hole and a 20 mm-radius
angle hole at both arm lengths. Rear extension carries the counterbalance hook.

Plate frame: X along the arm from the pivot (x=0) toward the tip, Y across, Z = PLY_T.
"""
import math
import cadquery as cq
import params as P
from common import export_dxf, export_step, export_stl, bbox_str, print_mass_g

T = P.PLY_T
W = P.ARM_PLATE_W
X0 = -P.ARM_REAR_EXT
X1 = P.ARM_LENGTH + 15.0
ANGLE_HOLE_R = 20.0


def arm_plate():
    plate = cq.Workplane("XY").center((X0 + X1) / 2, 0).rect(X1 - X0, W).extrude(T)
    plate = plate.edges("|Z").fillet(W / 2 - 0.5)
    # round pads at the two gripper mount positions so the angle hole has material around it
    for L in (P.ARM_LENGTH_ALT, P.ARM_LENGTH):
        plate = plate.union(cq.Workplane("XY").center(L, 0).circle(2 * ANGLE_HOLE_R + 6.0).extrude(T))

    wp = plate.faces(">Z").workplane(origin=(0, 0, 0))
    # pivot: hub hole + horn screw holes
    plate = wp.center(0, 0).hole(P.SERVO_HORN_HUB_D)
    for pcd, angs in ((14.0, (0, 90, 180, 270)), (20.0, (45, 135, 225, 315))):
        pts = [(pcd / 2 * math.cos(math.radians(a)), pcd / 2 * math.sin(math.radians(a))) for a in angs]
        plate = plate.faces(">Z").workplane(origin=(0, 0, 0)).pushPoints(pts).hole(P.SERVO_HORN_HOLE_D)

    # gripper mount at both lengths: pivot hole + angle hole in the anti-finger direction
    ang = math.radians(180.0 + P.GRIPPER_MOUNT_ANGLE_DEG)   # direction from gripper pivot toward the ear's angle hole
    for L in (P.ARM_LENGTH_ALT, P.ARM_LENGTH):
        plate = plate.faces(">Z").workplane(origin=(0, 0, 0)).center(L, 0).hole(P.M3_CLEAR)
        ax = L + ANGLE_HOLE_R * math.cos(ang)
        ay = ANGLE_HOLE_R * math.sin(ang)
        plate = plate.faces(">Z").workplane(origin=(0, 0, 0)).center(ax, ay).hole(P.M3_CLEAR)

    # spacer bolt holes
    for x in (60.0, 140.0):
        plate = plate.faces(">Z").workplane(origin=(0, 0, 0)).center(x, 0).hole(P.M3_CLEAR)
    # counterbalance hook hole on the rear extension
    plate = plate.faces(">Z").workplane(origin=(0, 0, 0)).center(X0 + 8.0, 0).hole(4.0)
    # lightening holes
    for x in (30.0, 90.0, 110.0, 165.0):
        plate = plate.faces(">Z").workplane(origin=(0, 0, 0)).center(x, 0).hole(12.0)
    return plate


def spacer():
    """Printed block between the arm plates with an M3 nut trap at each end, so two short
    M3 x 12 screws (one per side) clamp the plates. Print 2."""
    L = P.ARM_PLATE_GAP
    s = cq.Workplane("XY").box(12.0, 12.0, L, centered=(True, True, False))
    s = s.faces(">Z").workplane(origin=(0, 0, 0)).hole(P.M3_CLEAR)
    for z in (0.0, L - P.M3_NUT_T):
        trap = cq.Workplane("XY").polygon(6, P.M3_NUT_AF / math.cos(math.radians(30))).extrude(P.M3_NUT_T).translate((0, 0, z))
        s = s.cut(trap)
    return s


if __name__ == "__main__":
    p = arm_plate()
    export_dxf(p, "arm_plate")
    export_step(p, "arm_plate")
    s = spacer()
    export_stl(s, "arm_spacer")
    print(f"arm_plate: {bbox_str(p)} (cut 2). Gripper holes at L={P.ARM_LENGTH_ALT} and {P.ARM_LENGTH}")
    print(f"arm_spacer: {bbox_str(s)}, ~{print_mass_g(s):.0f} g each (print 2)")
    # reach check for both pipe hypotheses
    from common import reach_table, gripper_offset
    along, side = gripper_offset()
    print(f"  gripper offset from tip pivot to dimple line: {along:.0f} mm along fingers, {side:.0f} mm sideways")
    for label, L, H, plen, fod in ((f"measured pipe, primary holes", P.ARM_LENGTH, P.PIVOT_H, P.PIPE_LENGTH, P.PIPE_FLANGE_OD),
                                   (f"measured pipe, alt holes", P.ARM_LENGTH_ALT, P.PIVOT_H, P.PIPE_LENGTH, P.PIPE_FLANGE_OD),
                                   ("if pipe is 127 mm tall", P.ARM_LENGTH_ALT, 215.0, 127.0, P.PIPE_FLANGE_OD)):
        print(f"  {label}: L={L:.0f} pivot={H:.0f}")
        for name, h, phi, x in reach_table(L, H, plen, fod):
            if phi is None:
                print(f"     {name:12s} h={h:5.0f}  OUT OF REACH")
            else:
                print(f"     {name:12s} h={h:5.0f}  arm {phi:+5.1f} deg, contact {x:5.0f} mm ahead of pivot")
