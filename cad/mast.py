"""
Mast: laser-cut 1/8 in plywood. Two side plates stand on a base plate (tab and slot),
joined by a back brace and a top brace. The shoulder servo bolts to the INSIDE of the
left plate at any of three pivot heights; its boss passes through the plate and the
horn sits outside, carrying the left arm plate. The right plate has an M3 free pivot.

Plate frame for each 2D part: X across the plate, Y up the plate, extruded in Z by PLY_T.
World frame of the assembly: X forward, Y lateral, Z up. The mast's X (fore-aft) is the
plate width direction; the pivot sits MAST_PIVOT_X from the plate's front edge.
"""
import cadquery as cq
import params as P
from common import export_dxf, export_step, bbox_str, spur_gear  # noqa: F401

T = P.PLY_T
SLOT_W = T + 2 * P.KERF          # slot width for a tab of thickness T
BASE_TOP_Z = P.TOP_PLATE_H + T   # top surface of the mast base plate above the floor
MAST_PIVOT_X = 35.0              # pivot from the plate front edge
PLATE_W = P.MAST_PLATE_W
PLATE_H = max(P.PIVOT_H_OPTIONS) - BASE_TOP_Z + 30.0   # above the base plate
TAB_W = 15.0
TAB_POS = [12.0, PLATE_W - 12.0]   # tab centers across the plate width
BRACE_SLOT_H = 20.0                # brace tab length (vertical) through side plates
BACK_BRACE_W = P.MAST_INNER_GAP + 2 * T   # tabs go through both side plates
TOP_BRACE_W = BACK_BRACE_W


def pivot_y(h):
    return h - BASE_TOP_Z


def side_plate(left: bool):
    """Side plate standing on the base. Origin: bottom-front corner at (0,0); tabs below y=0."""
    pts = [(0, 0), (PLATE_W, 0), (PLATE_W, PLATE_H), (0, PLATE_H)]
    plate = cq.Workplane("XY").polyline(pts).close().extrude(T)
    # bottom tabs into the base plate
    for x in TAB_POS:
        tab = cq.Workplane("XY").box(TAB_W, P.TAB_LEN, T, centered=(True, False, False)).translate((x, -P.TAB_LEN, 0))
        plate = plate.union(tab)
    # back-brace slots (two per plate, at 1/4 and 3/4 height) and top-brace slot
    for yy in (PLATE_H * 0.22, PLATE_H * 0.62):
        s = cq.Workplane("XY").box(SLOT_W, BRACE_SLOT_H, T * 3, centered=(True, True, True)).translate((PLATE_W - 8.0, yy, T / 2))
        plate = plate.cut(s)
    s = cq.Workplane("XY").box(BRACE_SLOT_H, SLOT_W, T * 3).translate((PLATE_W / 2, PLATE_H - 8.0, T / 2))
    plate = plate.cut(s)

    wp = plate.faces(">Z").workplane()
    for h in P.PIVOT_H_OPTIONS:
        py = pivot_y(h)
        if left:
            # boss clearance + four servo tab holes (servo length vertical, shaft above body center)
            plate = plate.faces(">Z").workplane().center(MAST_PIVOT_X, py).hole(P.SERVO_BOSS_D + 0.6)
            cy = py - P.SERVO_SHAFT_OFF
            pts4 = [(MAST_PIVOT_X + dx, cy + dy)
                    for dx in (-P.SERVO_TAB_HOLE_Y / 2, P.SERVO_TAB_HOLE_Y / 2)
                    for dy in (-P.SERVO_TAB_HOLE_X / 2, P.SERVO_TAB_HOLE_X / 2)]
            plate = plate.faces(">Z").workplane().pushPoints(pts4).slot2D(P.TAB_SLOT_LEN, P.SERVO_TAB_HOLE_D, angle=90).cutThruAll()
        else:
            plate = plate.faces(">Z").workplane().center(MAST_PIVOT_X, py).hole(P.M3_CLEAR)
    # lightening holes between pivots
    for i in range(len(P.PIVOT_H_OPTIONS) - 1):
        ymid = (pivot_y(P.PIVOT_H_OPTIONS[i]) + pivot_y(P.PIVOT_H_OPTIONS[i + 1])) / 2
        plate = plate.faces(">Z").workplane().center(MAST_PIVOT_X - 14, ymid).hole(14.0)
    return plate


def base_plate():
    """Bolts to the kit top plate. Fore-aft slots so the mast can move to balance the robot."""
    L, W = P.MAST_BASE_L, P.MAST_BASE_W
    plate = cq.Workplane("XY").box(L, W, T, centered=(True, True, False))
    plate = plate.edges("|Z").fillet(6.0)
    # side plate slots: two plates at y = +-(gap/2 + T/2), tabs at x positions relative to the pivot column
    for sy in (-1, 1):
        y = sy * (P.MAST_INNER_GAP / 2 + T / 2)
        for x in TAB_POS:
            xx = x - PLATE_W / 2
            s = cq.Workplane("XY").box(TAB_W + 2 * P.KERF, SLOT_W, T * 3).translate((xx, y, T / 2))
            plate = plate.cut(s)
    # four bolt slots (20 mm long) to the kit top plate
    pts = [(sx * P.MAST_BASE_BOLT_SPACING_X / 2, sy * P.MAST_BASE_BOLT_SPACING_Y / 2) for sx in (-1, 1) for sy in (-1, 1)]
    plate = plate.faces(">Z").workplane().pushPoints(pts).slot2D(20.0, P.M3_CLEAR, angle=0).cutThruAll()
    return plate


def back_brace():
    """Spans between the side plates near the rear edge; two tabs each side."""
    H = PLATE_H * 0.62 - PLATE_H * 0.22 + BRACE_SLOT_H + 24.0
    W = P.MAST_INNER_GAP
    body = cq.Workplane("XY").box(W, H, T, centered=(True, True, False))
    y0 = -H / 2 + 12.0 + BRACE_SLOT_H / 2
    y1 = H / 2 - 12.0 - BRACE_SLOT_H / 2
    for sx in (-1, 1):
        for yy in (y0, y1):
            tab = cq.Workplane("XY").box(T, BRACE_SLOT_H - 2 * P.KERF, T, centered=(True, True, False)).translate((sx * (W / 2 + T / 2), yy, 0))
            body = body.union(tab)
    # lightening window
    body = body.faces(">Z").workplane().rect(W - 16, H - 40).cutThruAll()
    return body


def top_brace():
    """Short brace across the top with two rubber-band hook notches."""
    W = P.MAST_INNER_GAP
    H = 24.0
    body = cq.Workplane("XY").box(W, H, T, centered=(True, True, False))
    for sx in (-1, 1):
        tab = cq.Workplane("XY").box(T, BRACE_SLOT_H - 2 * P.KERF, T, centered=(True, True, False)).translate((sx * (W / 2 + T / 2), 0, 0))
        body = body.union(tab)
    # hook notches on the top edge
    for x in (-W / 4, W / 4):
        notch = cq.Workplane("XY").box(5.0, 6.0, T * 3).translate((x, H / 2 - 2.0, T / 2))
        body = body.cut(notch)
    return body


def pivot_bushing():
    """Printed bushing: 8 mm OD into the arm plate's 8 mm hole, 3.4 mm bore for the M3 pivot bolt."""
    b = cq.Workplane("XY").circle(P.SERVO_HORN_HUB_D / 2 - 0.1).extrude(T + 0.5)
    b = b.union(cq.Workplane("XY").circle(7.0).extrude(2.0).translate((0, 0, T + 0.5)))
    return b.faces(">Z").workplane().hole(P.M3_CLEAR)


if __name__ == "__main__":
    parts = {
        "mast_side_left": side_plate(True),
        "mast_side_right": side_plate(False),
        "mast_base": base_plate(),
        "mast_back_brace": back_brace(),
        "mast_top_brace": top_brace(),
    }
    for name, shape in parts.items():
        export_dxf(shape, name)
        export_step(shape, name)
        print(f"{name}: {bbox_str(shape)}")
    from common import export_stl
    b = pivot_bushing()
    export_stl(b, "arm_pivot_bushing")
    print(f"arm_pivot_bushing: {bbox_str(b)} (print 1)")
    print(f"pivot heights above floor: {P.PIVOT_H_OPTIONS}, base top at {BASE_TOP_Z} mm, plate height {PLATE_H:.0f} mm")
