"""
Plow (Level 1 fallback): a plywood V that bolts to the front of the baseplate and pushes
lying pipes into the trench, presses the dispense button and bumps the lever.
Three laser-cut pieces: a horizontal base with two slots, and two walls that tab in.
Walls are PLOW_WALL_H tall so a pipe (flange OD up to ~90 mm) cannot ride over them.
"""
import math
import cadquery as cq
import params as P
from common import export_dxf, export_step, bbox_str

T = P.PLY_T
SLOT_W = T + 2 * P.KERF
half = math.radians(P.PLOW_V_ANGLE_DEG / 2)
WALL_L = math.hypot(P.PLOW_W / 2 - 30.0, P.PLOW_DEPTH)   # apex corner to front corner


def base():
    """Trapezoid base: V opening at the front (+Y), rear tongue bolts under the baseplate."""
    w_front = P.PLOW_W
    apex_half = 30.0
    pts = [(-w_front / 2, P.PLOW_DEPTH), (w_front / 2, P.PLOW_DEPTH), (apex_half, 0), (apex_half, -30), (-apex_half, -30), (-apex_half, 0)]
    b = cq.Workplane("XY").polyline(pts).close().extrude(T)
    # wall slots along each V edge, inset 6 mm from the edge
    for sx in (-1, 1):
        x0, y0 = sx * apex_half, 0.0
        x1, y1 = sx * w_front / 2, P.PLOW_DEPTH
        ex, ey = x1 - x0, y1 - y0
        L = math.hypot(ex, ey)
        ux, uy = ex / L, ey / L
        nx, ny = -sx * uy, sx * ux          # inward normal
        ang = math.degrees(math.atan2(ey, ex))
        for frac in (0.3, 0.7):
            cx = x0 + ux * L * frac + nx * 6.0
            cy = y0 + uy * L * frac + ny * 6.0
            slot = (cq.Workplane("XY").box(16 + 2 * P.KERF, SLOT_W, T * 3)
                    .rotate((0, 0, 0), (0, 0, 1), ang).translate((cx, cy, T / 2)))
            b = b.cut(slot)
    b = b.faces(">Z").workplane().pushPoints([(-P.BASEPLATE_FRONT_HOLE_SPACING / 2, -15), (P.BASEPLATE_FRONT_HOLE_SPACING / 2, -15)]).hole(P.M3_CLEAR)
    return b


def wall():
    w = cq.Workplane("XY").rect(WALL_L, P.PLOW_WALL_H).extrude(T)
    for frac in (0.3, 0.7):
        x = -WALL_L / 2 + WALL_L * frac
        tab = cq.Workplane("XY").box(16 - 2 * P.KERF, P.TAB_LEN, T, centered=(True, False, False)).translate((x, -P.PLOW_WALL_H / 2 - P.TAB_LEN, 0))
        w = w.union(tab)
    return w


if __name__ == "__main__":
    b = base()
    w = wall()
    export_dxf(b, "plow_base"); export_step(b, "plow_base")
    export_dxf(w, "plow_wall"); export_step(w, "plow_wall")
    print(f"plow_base: {bbox_str(b)} (cut 1)   plow_wall: {bbox_str(w)} (cut 2)")
