"""
Assembly check: places mast, arm, gripper and a hanging pipe in world coordinates for
several arm angles and renders side views. Catches orientation and clearance mistakes
before anything is cut. World: X forward, Y lateral (left +), Z up from the floor.

Not a fabrication output. Run after any params change: python assembly_check.py
"""
import math
import os
import numpy as np
import cadquery as cq
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import params as P
from common import OUT, contact_point, arm_angle_for_height, gripper_offset
import mast as M
import arm as A
import gripper as G

T = P.PLY_T
BASE_TOP = M.BASE_TOP_Z
PIVOT_X = M.MAST_PIVOT_X - M.PLATE_W / 2     # pivot column relative to the mast base center (world x)


def to_mesh(shape, tol=0.4):
    tmp = os.path.join(OUT, "_tmp.stl")
    cq.exporters.export(shape, tmp, tolerance=tol, angularTolerance=0.3)
    m = trimesh.load(tmp, force="mesh")
    os.remove(tmp)
    return m


def place(shape, rot_axis=None, rot_deg=0.0, rot_center=(0, 0, 0), translate=(0, 0, 0), rot2=None):
    s = shape
    if rot2 is not None:
        s = s.rotate(rot2[0], rot2[1], rot2[2])
    if rot_axis is not None and rot_deg != 0:
        a0 = cq.Vector(*rot_center)
        a1 = a0 + cq.Vector(*rot_axis)
        s = s.rotate(a0, a1, rot_deg)
    return s.translate(cq.Vector(*translate))


def build_world(phi_deg, L, pivot_h, stack_n, pipe_len, flange_od, body_od):
    parts = []
    # mast base and side plates (plates drawn in the XZ plane at y = +-gap/2)
    base = M.base_plate().translate((0, 0, P.TOP_PLATE_H))
    parts.append(("base", base))
    for sx, left in ((1, True), (-1, False)):
        plate = M.side_plate(left)   # local X across (world x), Y up (world z), Z thickness (world y)
        plate = plate.rotate((0, 0, 0), (1, 0, 0), 90)                 # Y -> Z, Z -> -Y
        plate = plate.translate((-M.PLATE_W / 2, sx * (P.MAST_INNER_GAP / 2) + (T if sx > 0 else 0), BASE_TOP))
        parts.append((f"mast_{'L' if left else 'R'}", plate))
    # arm plates: local X along arm, Y across (world z when phi=0), Z thickness -> world y
    for sx in (1, -1):
        ap = A.arm_plate().rotate((0, 0, 0), (1, 0, 0), 90)            # XY plane -> XZ plane
        ap = ap.rotate((0, 0, 0), (0, 1, 0), -phi_deg)                  # rotate about world y (right-hand: +phi lifts the tip)
        ap = ap.translate((PIVOT_X, sx * (P.ARM_PLATE_GAP / 2) + (T if sx > 0 else 0), pivot_h))
        parts.append((f"arm_{sx}", ap))
    # gripper: frame X_g = -y, Y_g = finger dir, Z_g = (-y) x f
    phi = math.radians(phi_deg)
    a = math.radians(P.GRIPPER_MOUNT_ANGLE_DEG)
    theta = phi + a
    tip = np.array([PIVOT_X + L * math.cos(phi), 0.0, pivot_h + L * math.sin(phi)])
    f = np.array([math.cos(theta), 0.0, math.sin(theta)])
    zg = np.array([-math.sin(theta), 0.0, math.cos(theta)])
    xg = np.array([0.0, -1.0, 0.0])
    origin = tip - P.GRIPPER_EAR_PIVOT_Y * f - P.GRIPPER_EAR_PIVOT_Z * zg
    R = np.column_stack([xg, f, zg])   # columns = images of gripper axes
    # build a CadQuery transform from R and origin
    mat = cq.Matrix([[R[0, 0], R[0, 1], R[0, 2], origin[0]],
                     [R[1, 0], R[1, 1], R[1, 2], origin[1]],
                     [R[2, 0], R[2, 1], R[2, 2], origin[2]]])
    for name, shp in (("carrier", G.carrier()), ("jaw_d", G.jaw(True).translate((G.S / 2, 0, G.Z0))),
                      ("jaw_i", G.jaw(False).mirror("YZ").translate((-G.S / 2, 0, G.Z0)))):
        parts.append((name, cq.Workplane("XY").add(shp.val().transformGeometry(mat))))
    # pipe hanging from the dimple line
    cx, cz = contact_point(phi_deg, L, pivot_h, PIVOT_X)
    pipe = (cq.Workplane("XY").circle(body_od / 2).extrude(pipe_len)
            .union(cq.Workplane("XY").circle(flange_od / 2).extrude(P.PIPE_FLANGE_T))
            .union(cq.Workplane("XY").circle(flange_od / 2).extrude(P.PIPE_FLANGE_T).translate((0, 0, pipe_len - P.PIPE_FLANGE_T))))
    pipe = pipe.translate((cx, 0, cz - pipe_len + P.PIPE_FLANGE_T / 2))
    parts.append(("pipe", pipe))
    # stack on the floor below the contact point
    for n in range(stack_n):
        sp = (cq.Workplane("XY").circle(body_od / 2).extrude(pipe_len)).translate((cx, 0, n * pipe_len))
        parts.append((f"stack{n}", sp))
    # kit chassis stand-in: 150 x 200 x 3 top plate under the base, wheels
    chassis = cq.Workplane("XY").box(200, P.TOP_PLATE_W, P.KIT_PLATE_T, centered=(True, True, False)).translate((-40, 0, P.TOP_PLATE_H - P.KIT_PLATE_T))
    parts.append(("chassis", chassis))
    floor = cq.Workplane("XY").box(500, 300, 1, centered=(True, True, False)).translate((100, 0, -1))
    parts.append(("floor", floor))
    return parts, (cx, cz)


COLORS = {"pipe": (0.9, 0.5, 0.2), "stack": (0.8, 0.8, 0.3), "floor": (0.85, 0.85, 0.85), "chassis": (0.6, 0.45, 0.3),
          "base": (0.7, 0.55, 0.35), "mast": (0.7, 0.55, 0.35), "arm": (0.75, 0.6, 0.4), "carrier": (0.3, 0.5, 0.8),
          "jaw": (0.3, 0.7, 0.4)}


def draw(ax, parts, title, elev, azim):
    allv = []
    for name, shp in parts:
        m = to_mesh(shp)
        key = next((k for k in COLORS if name.startswith(k)), "arm")
        col = COLORS[key]
        pc = Poly3DCollection(m.vertices[m.faces], facecolors=col + (0.95,), edgecolors=(0, 0, 0, 0.05), linewidths=0.1)
        ax.add_collection3d(pc)
        allv.append(m.vertices)
    v = np.vstack(allv)
    lo, hi = v.min(axis=0), v.max(axis=0)
    c = (lo + hi) / 2
    r = (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(0, 2 * r)
    ax.view_init(elev=elev, azim=azim)
    ax.set_box_aspect((1, 1, 1))
    ax.set_title(title, fontsize=9)
    ax.set_xlabel("x fwd"); ax.set_zlabel("z")


def main():
    L, H = P.ARM_LENGTH, P.PIVOT_H
    pl, fod, bod = P.PIPE_LENGTH, P.PIPE_FLANGE_OD, P.PIPE_BODY_OD
    cases = []
    for name, h, stack in (("floor pick", fod / 2, 0), ("top pick (pipe on floor)", pl, 0),
                           ("place on 1", pl + P.CLEAR_ABOVE_STACK + pl, 1), ("place on 2", 2 * pl + P.CLEAR_ABOVE_STACK + pl, 2),
                           ("stow +70", None, 0)):
        phi = 70.0 if h is None else arm_angle_for_height(h, L, H)
        if phi is None:
            print(f"{name}: unreachable"); continue
        cases.append((name, phi, stack))
    fig = plt.figure(figsize=(5 * len(cases), 10))
    for i, (name, phi, stack) in enumerate(cases):
        parts, (cx, cz) = build_world(phi, L, H, stack, pl, fod, bod)
        ax = fig.add_subplot(2, len(cases), i + 1, projection="3d")
        draw(ax, parts, f"{name}: arm {phi:+.0f} deg, jaws at x={cx:.0f} z={cz:.0f}", elev=0, azim=-90)
        ax2 = fig.add_subplot(2, len(cases), len(cases) + i + 1, projection="3d")
        draw(ax2, parts, "3/4 view", elev=25, azim=-60)
        print(f"{name}: arm {phi:+.1f} deg, contact x={cx:.0f} mm (pivot at x={PIVOT_X:.0f}), z={cz:.0f} mm")
    out = os.path.join(OUT, "assembly_check.png")
    plt.tight_layout()
    plt.savefig(out, dpi=90)
    print("wrote", out)


if __name__ == "__main__":
    main()
