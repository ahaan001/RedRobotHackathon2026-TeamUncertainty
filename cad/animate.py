"""
Animated simulation of one scoring cycle, rendered from the real part geometry:
approach a lying pipe, hook under its flange, lift (pipe swings upright), drive to the
Steam Plant pocket, set it on the stack, open, back away. Also exports a positioned
STEP assembly for Onshape.

    python animate.py            -> out/cycle_simulation.mp4, out/assembly_place_on_1.step

World: X forward (arm direction), Y lateral, Z up from the floor. The robot moves along X.
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
from common import OUT, contact_point, arm_angle_for_height
import mast as M
import arm as A
import gripper as G

T = P.PLY_T
BASE_TOP = M.BASE_TOP_Z
PIVOT_X = M.MAST_PIVOT_X - M.PLATE_W / 2
L, HP = P.ARM_LENGTH, P.PIVOT_H
ALPHA = math.radians(P.GRIPPER_MOUNT_ANGLE_DEG)
PL, FOD, BOD, FT = P.PIPE_LENGTH, P.PIPE_FLANGE_OD, P.PIPE_BODY_OD, P.PIPE_FLANGE_T
HOOK_Z = PL - FT                      # pipe-local height of the hook contact line
FPS = 12

# ------------------------------------------------------------------ meshes (local frames)
def mesh_of(shape, tol=0.5):
    tmp = os.path.join(OUT, "_anim_tmp.stl")
    cq.exporters.export(shape, tmp, tolerance=tol, angularTolerance=0.35)
    m = trimesh.load(tmp, force="mesh")
    os.remove(tmp)
    return m


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def xf(mesh, R, t):
    return (mesh.vertices @ R.T) + np.asarray(t)


print("tessellating parts...")
mast_parts = []
base = M.base_plate().translate((0, 0, P.TOP_PLATE_H))
mast_parts.append(mesh_of(base))
for sx, left in ((1, True), (-1, False)):
    plate = M.side_plate(left).rotate((0, 0, 0), (1, 0, 0), 90)
    plate = plate.translate((-M.PLATE_W / 2, sx * (P.MAST_INNER_GAP / 2) + (T if sx > 0 else 0), BASE_TOP))
    mast_parts.append(mesh_of(plate))
arm_mesh = mesh_of(A.arm_plate())
carrier_mesh = mesh_of(G.carrier())
jawd_mesh = mesh_of(G.jaw(True))                       # gear at origin, finger +Y on +X side
jawi_mesh = mesh_of(G.jaw(False).mirror("YZ"))         # gear at origin, finger +Y on -X side
pipe_shape = (cq.Workplane("XY").circle(BOD / 2).extrude(PL)
              .union(cq.Workplane("XY").circle(FOD / 2).extrude(FT))
              .union(cq.Workplane("XY").circle(FOD / 2).extrude(FT).translate((0, 0, PL - FT))))
pipe_mesh = mesh_of(pipe_shape, tol=0.3)
# chassis stand-in (STEP numbers): world x = mcx - chassis_x
mcx = P.MAST_CENTER_REAR_X
chassis = []
chassis.append(mesh_of(cq.Workplane("XY").box(P.TOP_PLATE_L, P.TOP_PLATE_W, P.KIT_PLATE_T, centered=(True, True, False))
                       .translate((mcx - (171.4 - 10.2) / 2, 0, P.TOP_PLATE_H - P.KIT_PLATE_T))))
chassis.append(mesh_of(cq.Workplane("XY").box(P.BASEPLATE_L, P.TOP_PLATE_W, P.KIT_PLATE_T, centered=(True, True, False))
                       .translate((mcx - P.BASEPLATE_L / 2, 0, P.BASEPLATE_UNDER_H))))
for sy in (-1, 1):
    chassis.append(mesh_of(cq.Workplane("XZ").circle(32.8).extrude(26).translate((mcx - P.DRIVE_AXLE_X, sy * 82 + 13, 32.8)), tol=0.8))
chassis.append(mesh_of(cq.Workplane("XY").sphere(9.5).translate((mcx - P.CASTER_X, 0, 9.5)), tol=0.8))

# Steam Plant: wall + pocket block (2x4 lying flat, 38 tall x 89 deep, 3 in semicircle pocket)
X_WALL = 700.0
BLOCK_H, BLOCK_D = 38.0, 89.0
X_STACK = X_WALL - BLOCK_D            # pocket center on the block's front edge
block = (cq.Workplane("XY").box(BLOCK_D, 240, BLOCK_H, centered=(False, True, False)).translate((X_WALL - BLOCK_D, 0, 0)))
for yy in (-80, 0, 80):
    block = block.cut(cq.Workplane("XY").circle(FOD / 2 + 1).extrude(BLOCK_H + 1).translate((X_STACK, yy, 0)))
block_mesh = mesh_of(block, tol=0.8)
wall_mesh = mesh_of(cq.Workplane("XY").box(20, 240, 180, centered=(False, True, False)).translate((X_WALL, 0, 0)), tol=1)
floor_mesh = mesh_of(cq.Workplane("XY").box(1400, 320, 1, centered=(False, True, False)).translate((-500, 0, -1)), tol=5)
stack_pipe_world = xf(pipe_mesh, np.eye(3), (X_STACK, 0, 0))   # one pipe already in the pocket

# ------------------------------------------------------------------ poses
ARM_FLOOR = arm_angle_for_height(P.H_FLOOR_PICK, L, HP)
ARM_CARRY = 5.0
ARM_L1 = arm_angle_for_height(P.h_place(1), L, HP)
JAW_OPEN, JAW_RIM = 11.0, 3.0
SET_DROP_DEG = 3.0


def ease(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


def lerp(a, b, u):
    return a + (b - a) * ease(u)


def gripper_frame(phi_deg, robot_x):
    phi = math.radians(phi_deg)
    th = phi + ALPHA
    tip = np.array([robot_x + PIVOT_X + L * math.cos(phi), 0.0, HP + L * math.sin(phi)])
    f = np.array([math.cos(th), 0.0, math.sin(th)])
    zg = np.array([-math.sin(th), 0.0, math.cos(th)])
    xg = np.array([0.0, -1.0, 0.0])
    origin = tip - P.GRIPPER_EAR_PIVOT_Y * f - P.GRIPPER_EAR_PIVOT_Z * zg
    R = np.column_stack([xg, f, zg])
    return R, origin


def contact_world(phi_deg, robot_x):
    cx, cz = contact_point(phi_deg, L, HP, PIVOT_X)
    return np.array([robot_x + cx, 0.0, cz])


# Timeline (seconds)
T_APPROACH, T_CLOSE, T_LIFT, T_DRIVE, T_SET, T_OPEN, T_RETREAT = 2.0, 1.0, 2.5, 3.0, 1.0, 1.0, 1.8
T_END = T_APPROACH + T_CLOSE + T_LIFT + T_DRIVE + T_SET + T_OPEN + T_RETREAT

# Robot x such that the jaws straddle the lying pipe at pickup = 0; pipe pose fixed from that.
X_PICK = 0.0
c0 = contact_world(ARM_FLOOR, X_PICK)
PIPE_LYING_R = rot_y(math.radians(90.0))
PIPE_LYING_T = c0 - PIPE_LYING_R @ np.array([0, 0, HOOK_Z])
# Robot x so that the held pipe is over the stack at the L1 preset
X_PLACE = X_STACK - contact_point(ARM_L1, L, HP, PIVOT_X)[0]


def state(t):
    """Returns robot_x, arm_deg, jaw_deg, pipe (R, t) or None if on stack, held flag."""
    if t < T_APPROACH:
        u = t / T_APPROACH
        return lerp(X_PICK - 320, X_PICK, u), ARM_FLOOR, JAW_OPEN, (PIPE_LYING_R, PIPE_LYING_T), "lying"
    t -= T_APPROACH
    if t < T_CLOSE:
        return X_PICK, ARM_FLOOR, lerp(JAW_OPEN, JAW_RIM, t / T_CLOSE), (PIPE_LYING_R, PIPE_LYING_T), "lying"
    t -= T_CLOSE
    if t < T_LIFT:
        u = t / T_LIFT
        arm = lerp(ARM_FLOOR, ARM_CARRY, min(1.0, u * 1.6))
        tau = t
        psi = 90.0 * math.exp(-2.2 * tau) * math.cos(3.5 * tau)      # damped pendulum swing
        c = contact_world(arm, X_PICK)
        R = rot_y(math.radians(psi))
        return X_PICK, arm, JAW_RIM, (R, c - R @ np.array([0, 0, HOOK_Z])), "held"
    t -= T_LIFT
    if t < T_DRIVE:
        u = t / T_DRIVE
        rx = lerp(X_PICK, X_PLACE, u)
        arm = lerp(ARM_CARRY, ARM_L1, u)
        c = contact_world(arm, rx)
        psi = 4.0 * math.sin(6 * t) * (1 - u)                           # small residual sway
        R = rot_y(math.radians(psi))
        return rx, arm, JAW_RIM, (R, c - R @ np.array([0, 0, HOOK_Z])), "held"
    t -= T_DRIVE
    if t < T_SET:
        arm = lerp(ARM_L1, ARM_L1 - SET_DROP_DEG, t / T_SET)
        c = contact_world(arm, X_PLACE)
        # pipe lands on the stack: its bottom cannot go below the lower pipe's top
        z = max(c[2] - HOOK_Z, PL)
        return X_PLACE, arm, JAW_RIM, (np.eye(3), np.array([X_STACK, 0, z])), "placed"
    t -= T_SET
    if t < T_OPEN:
        return X_PLACE, ARM_L1 - SET_DROP_DEG, lerp(JAW_RIM, JAW_OPEN, t / T_OPEN), (np.eye(3), np.array([X_STACK, 0, PL])), "placed"
    t -= T_OPEN
    u = t / T_RETREAT
    arm = lerp(ARM_L1 - SET_DROP_DEG, ARM_L1 + 8, min(1.0, u * 2))
    rx = lerp(X_PLACE, X_PLACE - 160, max(0.0, (u - 0.3) / 0.7))
    return rx, arm, JAW_OPEN, (np.eye(3), np.array([X_STACK, 0, PL])), "placed"


# ------------------------------------------------------------------ drawing
COL = {"wood": (0.72, 0.56, 0.36), "ply": (0.78, 0.63, 0.42), "blue": (0.3, 0.5, 0.85), "green": (0.25, 0.65, 0.4),
       "pipe": (0.9, 0.55, 0.2), "stack": (0.85, 0.8, 0.3), "grey": (0.6, 0.6, 0.6), "wall": (0.35, 0.35, 0.35), "floor": (0.9, 0.9, 0.88)}


def world_parts(t):
    rx, arm, jaw, (pR, pT), flag = state(t)
    out = []
    shift = np.array([rx, 0, 0])
    for m in chassis:
        out.append((m.faces, m.vertices + shift, COL["grey"]))
    for m in mast_parts:
        out.append((m.faces, m.vertices + shift, COL["wood"]))
    # arm plates
    Ra = rot_y(math.radians(-arm)) @ rot_x(math.radians(90))
    for sx in (1, -1):
        ta = np.array([rx + PIVOT_X, sx * (P.ARM_PLATE_GAP / 2) + (T if sx > 0 else 0), HP])
        out.append((arm_mesh.faces, xf(arm_mesh, Ra, ta), COL["ply"]))
    # gripper
    Rg, og = gripper_frame(arm, rx)
    out.append((carrier_mesh.faces, xf(carrier_mesh, Rg, og), COL["blue"]))
    jr = math.radians(jaw)
    vd = (jawd_mesh.vertices @ rot_z(-jr).T) + np.array([G.S / 2, 0, G.Z0])
    vi = (jawi_mesh.vertices @ rot_z(jr).T) + np.array([-G.S / 2, 0, G.Z0])
    out.append((jawd_mesh.faces, vd @ Rg.T + og, COL["green"]))
    out.append((jawi_mesh.faces, vi @ Rg.T + og, COL["green"]))
    # pipes, field
    out.append((pipe_mesh.faces, xf(pipe_mesh, pR, pT), COL["pipe"]))
    out.append((pipe_mesh.faces, stack_pipe_world, COL["stack"]))
    out.append((block_mesh.faces, block_mesh.vertices, COL["wall"]))
    out.append((wall_mesh.faces, wall_mesh.vertices, COL["wall"]))
    out.append((floor_mesh.faces, floor_mesh.vertices, COL["floor"]))
    return out, (rx, arm, jaw, flag)


LIGHT = np.array([0.4, -0.7, 0.6]); LIGHT /= np.linalg.norm(LIGHT)


def phase_name(t):
    edges = [(T_APPROACH, "1. approach lying pipe, jaws open"), (T_CLOSE, "2. close to RIM: hooks under the flange"),
             (T_LIFT, "3. lift: pipe swings upright by gravity"), (T_DRIVE, "4. drive to the Steam Plant, arm to L1"),
             (T_SET, "5. lower 3 deg: pipe lands on the stack"), (T_OPEN, "6. open jaws"), (T_RETREAT, "7. lift and back away")]
    for d, name in edges:
        if t < d:
            return name
        t -= d
    return edges[-1][1]


def draw(ax, parts, elev, azim, xlim, title):
    for faces, verts, col in parts:
        tri = verts[faces]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= (np.linalg.norm(n, axis=1, keepdims=True) + 1e-9)
        sh = 0.45 + 0.55 * np.clip(n @ LIGHT, 0, 1)
        fc = np.stack([sh * col[0], sh * col[1], sh * col[2], np.ones_like(sh)], 1)
        ax.add_collection3d(Poly3DCollection(tri, facecolors=fc, edgecolors="none"))
    ax.set_xlim(*xlim); ax.set_ylim(-230, 230); ax.set_zlim(0, 460)
    ax.set_box_aspect((xlim[1] - xlim[0], 460, 460))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    ax.set_title(title, fontsize=10)


def main():
    nframes = int(T_END * FPS)
    frame_dir = os.path.join(OUT, "_frames")
    os.makedirs(frame_dir, exist_ok=True)
    print(f"rendering {nframes} frames...")
    for i in range(nframes):
        t = i / FPS
        parts, (rx, arm, jaw, flag) = world_parts(t)
        cx = rx + 170
        xlim = (cx - 400, cx + 400)
        fig = plt.figure(figsize=(16, 7))
        fig.suptitle(f"Team Uncertainty scoring cycle    {phase_name(t)}    t = {t:4.1f} s    arm {arm:+.0f} deg    jaws {jaw:.0f} deg", fontsize=12)
        ax1 = fig.add_subplot(1, 2, 1, projection="3d")
        draw(ax1, parts, 0, -90, xlim, "side view")
        ax2 = fig.add_subplot(1, 2, 2, projection="3d")
        draw(ax2, parts, 22, -52, xlim, "3/4 view")
        plt.subplots_adjust(left=0.0, right=1.0, top=0.92, bottom=0.0, wspace=0.0)
        fig.savefig(os.path.join(frame_dir, f"f{i:04d}.png"), dpi=90)
        plt.close(fig)
        if i % 12 == 0:
            print(f"  {i}/{nframes}")
    out = os.path.join(OUT, "cycle_simulation.mp4")
    os.system(f"ffmpeg -loglevel error -y -framerate {FPS} -i {frame_dir}/f%04d.png -c:v libx264 -pix_fmt yuv420p -vf 'scale=trunc(iw/2)*2:trunc(ih/2)*2' {out}")
    print("wrote", out)

    # positioned STEP assembly at the place-on-1 pose, for Onshape
    rx, arm, jaw, (pR, pT), _ = state(T_APPROACH + T_CLOSE + T_LIFT + T_DRIVE - 0.01)
    solids = []
    solids.append(base.translate((rx, 0, 0)).val())
    for sx, left in ((1, True), (-1, False)):
        plate = M.side_plate(left).rotate((0, 0, 0), (1, 0, 0), 90)
        plate = plate.translate((rx - M.PLATE_W / 2, sx * (P.MAST_INNER_GAP / 2) + (T if sx > 0 else 0), BASE_TOP))
        solids.append(plate.val())
    for sx in (1, -1):
        ap = A.arm_plate().rotate((0, 0, 0), (1, 0, 0), 90).rotate((0, 0, 0), (0, 1, 0), -arm)
        solids.append(ap.translate((rx + PIVOT_X, sx * (P.ARM_PLATE_GAP / 2) + (T if sx > 0 else 0), HP)).val())
    Rg, og = gripper_frame(arm, rx)
    mat = cq.Matrix([[Rg[0, 0], Rg[0, 1], Rg[0, 2], og[0]], [Rg[1, 0], Rg[1, 1], Rg[1, 2], og[1]], [Rg[2, 0], Rg[2, 1], Rg[2, 2], og[2]]])
    for shp in (G.carrier(), G.jaw(True).rotate((0, 0, 0), (0, 0, 1), -jaw).translate((G.S / 2, 0, G.Z0)),
                G.jaw(False).mirror("YZ").rotate((0, 0, 0), (0, 0, 1), jaw).translate((-G.S / 2, 0, G.Z0))):
        solids.append(shp.val().transformGeometry(mat))
    pm = cq.Matrix([[pR[0, 0], pR[0, 1], pR[0, 2], pT[0]], [pR[1, 0], pR[1, 1], pR[1, 2], pT[1]], [pR[2, 0], pR[2, 1], pR[2, 2], pT[2]]])
    solids.append(pipe_shape.val().transformGeometry(pm))
    solids.append(pipe_shape.translate((X_STACK, 0, 0)).val())
    solids.append(block.val())
    step_path = os.path.join(OUT, "assembly_place_on_1.step")
    cq.exporters.export(cq.Workplane("XY").add(cq.Compound.makeCompound(solids)), step_path)
    print("wrote", step_path)


if __name__ == "__main__":
    main()
