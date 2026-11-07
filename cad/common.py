"""Shared CadQuery helpers: involute spur gear, slots, tab/slot joints, exports."""
import math
import os
import cadquery as cq
from cadquery import exporters
import params as P

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(OUT, exist_ok=True)


# ---------------------------------------------------------------------------
# Involute spur gear (external), sampled profile. Adequate for slow PLA gears.
# ---------------------------------------------------------------------------
def spur_gear_profile(module, teeth, pressure_angle_deg=20.0, backlash=0.0, points_per_flank=8):
    m, z = module, teeth
    pa = math.radians(pressure_angle_deg)
    r_pitch = m * z / 2.0
    r_base = r_pitch * math.cos(pa)
    r_add = r_pitch + m
    r_ded = r_pitch - 1.25 * m
    r_start = max(r_base, r_ded)

    # half tooth thickness angle at the pitch circle, reduced by backlash
    tooth_thick = math.pi * m / 2.0 - backlash
    half_angle_pitch = tooth_thick / (2.0 * r_pitch)

    def inv(a):
        return math.tan(a) - a

    pa_pitch = math.acos(r_base / r_pitch)
    # angular offset so the flank passes through the pitch point at the right thickness
    phi0 = half_angle_pitch + inv(pa_pitch)

    def flank(r):
        a = math.acos(min(1.0, r_base / r))
        return phi0 - inv(a)  # angle of the flank point at radius r (right flank, positive side)

    pts = []
    pitch_angle = 2 * math.pi / z
    for i in range(z):
        base = i * pitch_angle
        # root point before the tooth
        pts.append((r_ded * math.cos(base - flank(r_start) - 0.0), r_ded * math.sin(base - flank(r_start))))
        # left flank up (negative side), from r_start to r_add
        for k in range(points_per_flank + 1):
            r = r_start + (r_add - r_start) * k / points_per_flank
            ang = base - flank(r)
            pts.append((r * math.cos(ang), r * math.sin(ang)))
        # right flank down (positive side)
        for k in range(points_per_flank, -1, -1):
            r = r_start + (r_add - r_start) * k / points_per_flank
            ang = base + flank(r)
            pts.append((r * math.cos(ang), r * math.sin(ang)))
        pts.append((r_ded * math.cos(base + flank(r_start)), r_ded * math.sin(base + flank(r_start))))
    return pts


def spur_gear(module, teeth, thickness, bore_d=0.0, backlash=0.0):
    pts = spur_gear_profile(module, teeth, backlash=backlash)
    g = cq.Workplane("XY").polyline(pts).close().extrude(thickness)
    if bore_d > 0:
        g = g.faces(">Z").workplane().hole(bore_d)
    return g


# ---------------------------------------------------------------------------
# 2D helpers for laser-cut plates (sketched on XY, extruded by PLY_T for STEP/STL,
# exported as DXF from the XY face)
# ---------------------------------------------------------------------------
def slot_cutter(length, width, depth):
    """A stadium-shaped slot solid, length along X, centered at origin."""
    return cq.Workplane("XY").slot2D(length, width).extrude(depth)


def plate_from_points(points, thickness=P.PLY_T):
    return cq.Workplane("XY").polyline(points).close().extrude(thickness)


def export_dxf(shape, name, thickness=P.PLY_T):
    """Export the outline (with holes) of a flat plate extruded along Z as DXF.

    Takes every face lying on the bottom plane (unions can leave coplanar faces unmerged)
    and writes all of their edges, then re-reads the file to verify it is not empty.
    """
    path = os.path.join(OUT, f"{name}.dxf")
    zmin = shape.val().BoundingBox().zmin
    edges = []
    for f in shape.faces().vals():
        bb = f.BoundingBox()
        if abs(bb.zmin - zmin) < 1e-3 and abs(bb.zmax - zmin) < 1e-3:
            edges.extend(f.Edges())
    if not edges:
        raise RuntimeError(f"{name}: no bottom-plane faces found for DXF export")
    exporters.export(cq.Workplane("XY").add(edges), path)
    try:
        import ezdxf
        n = len(ezdxf.readfile(path).modelspace())
    except ImportError:
        n = -1
    if n == 0:
        raise RuntimeError(f"{name}: DXF export produced no entities")
    print(f"  dxf {name}: {len(edges)} edges written, {n} entities read back")
    return path


def export_stl(shape, name, tolerance=0.05):
    path = os.path.join(OUT, f"{name}.stl")
    exporters.export(shape, path, tolerance=tolerance, angularTolerance=0.1)
    return path


def export_step(shape, name):
    path = os.path.join(OUT, f"{name}.step")
    exporters.export(shape, path)
    return path


def volume_cm3(shape):
    return shape.val().Volume() / 1000.0


def print_mass_g(shape, infill=P.PRINT_INFILL, shell_fraction=0.45):
    """Rough PLA mass: shells+top/bottom are ~45% of volume at this part size, infill fills the rest."""
    v = volume_cm3(shape)
    eff = shell_fraction + (1 - shell_fraction) * infill
    return v * eff * P.PLA_DENSITY_G_CM3


def bbox_str(shape):
    bb = shape.val().BoundingBox()
    return f"{bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm"


# ---------------------------------------------------------------------------
# Arm reach model (used by arm.py and assembly_check.py)
# ---------------------------------------------------------------------------
def gripper_offset():
    """Distance from the arm-tip pivot to the dimple (contact) line, along the finger direction,
    and its sideways component (positive = toward +Z_g)."""
    along = P.FINGER_L - P.GRIPPER_EAR_PIVOT_Y
    side = (P.SERVO_BOSS_H + P.FINGER_W / 2) - P.GRIPPER_EAR_PIVOT_Z
    return along, side


def contact_point(phi_deg, L=P.ARM_LENGTH, pivot_h=P.PIVOT_H, pivot_x=0.0):
    """World (x, z) of the dimple line for arm angle phi (deg above horizontal)."""
    phi = math.radians(phi_deg)
    a = math.radians(P.GRIPPER_MOUNT_ANGLE_DEG)
    along, side = gripper_offset()
    fx, fz = math.cos(phi + a), math.sin(phi + a)          # finger direction
    # Z_g = (-y) x f  = (-sin(theta), 0, cos(theta)) with theta = phi + a
    zx, zz = -math.sin(phi + a), math.cos(phi + a)
    x = pivot_x + L * math.cos(phi) + along * fx + side * zx
    z = pivot_h + L * math.sin(phi) + along * fz + side * zz
    return x, z


def arm_angle_for_height(h, L=P.ARM_LENGTH, pivot_h=P.PIVOT_H):
    """Solve contact z == h for phi in [-70, 75] deg by bisection; None if unreachable."""
    lo, hi = -70.0, 75.0
    f = lambda p: contact_point(p, L, pivot_h)[1] - h
    if f(lo) > 0 or f(hi) < 0:
        return None
    for _ in range(60):
        mid = (lo + hi) / 2
        if f(mid) < 0:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def reach_table(L=P.ARM_LENGTH, pivot_h=P.PIVOT_H, pipe_len=P.PIPE_LENGTH, flange_od=P.PIPE_FLANGE_OD):
    targets = [("floor pick", flange_od / 2), ("top pick", pipe_len)]
    for n in (1, 2, 3):
        targets.append((f"place on {n}", n * pipe_len + P.CLEAR_ABOVE_STACK + pipe_len))
    rows = []
    for name, h in targets:
        phi = arm_angle_for_height(h, L, pivot_h)
        if phi is None:
            rows.append((name, h, None, None))
        else:
            x, _ = contact_point(phi, L, pivot_h)
            rows.append((name, h, phi, x))
    return rows


def circle_report(shape, name, expected=None):
    """Count distinct circular/arc edge centers on the bottom plane; compare with expected."""
    centers = set()
    for e in shape.edges("%CIRCLE").vals():
        c = e.Center()
        centers.add((round(c.x, 1), round(c.y, 1), round(e.radius(), 2)))
    msg = f"  holes/arcs on {name}: {len(centers)}"
    if expected is not None:
        msg += f" (expected {expected})" + ("" if len(centers) == expected else "  <-- MISMATCH")
    print(msg)
    return centers
