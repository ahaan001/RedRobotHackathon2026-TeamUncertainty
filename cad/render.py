"""Render every STL/STEP part in out/ to a PNG contact sheet for visual review (not for fabrication)."""
import glob
import os
import math
import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")


def render(mesh, ax, title, elev=28, azim=-50):
    v, f = mesh.vertices, mesh.faces
    tri = v[f]
    # simple shading by face normal
    n = mesh.face_normals
    light = np.array([0.4, -0.6, 0.7]); light /= np.linalg.norm(light)
    shade = 0.35 + 0.65 * np.clip(n @ light, 0, 1)
    colors = np.stack([shade * 0.55, shade * 0.7, shade * 0.95, np.ones_like(shade)], axis=1)
    pc = Poly3DCollection(tri, facecolors=colors, edgecolors=(0, 0, 0, 0.08), linewidths=0.2)
    ax.add_collection3d(pc)
    lo, hi = v.min(axis=0), v.max(axis=0)
    c = (lo + hi) / 2
    r = (hi - lo).max() / 2 * 1.05
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    ax.set_title(f"{title}\n{hi[0]-lo[0]:.0f} x {hi[1]-lo[1]:.0f} x {hi[2]-lo[2]:.0f} mm", fontsize=8)


def main():
    stls = sorted(glob.glob(os.path.join(OUT, "*.stl")))
    steps = sorted(p for p in glob.glob(os.path.join(OUT, "*.step")) if not os.path.exists(p.replace(".step", ".stl")))
    files = stls + steps
    n = len(files)
    cols = 4
    rows = math.ceil(n / cols)
    fig = plt.figure(figsize=(cols * 4, rows * 4))
    for i, path in enumerate(files):
        try:
            if path.endswith(".step"):
                import cadquery as cq
                shape = cq.importers.importStep(path)
                tmp = path.replace(".step", "_preview.stl")
                cq.exporters.export(shape, tmp, tolerance=0.1, angularTolerance=0.2)
                mesh = trimesh.load(tmp, force="mesh")
                os.remove(tmp)
            else:
                mesh = trimesh.load(path, force="mesh")
        except Exception as e:  # noqa: BLE001
            print("skip", path, e)
            continue
        ax = fig.add_subplot(rows, cols, i + 1, projection="3d")
        render(mesh, ax, os.path.basename(path))
    plt.tight_layout()
    sheet = os.path.join(OUT, "contact_sheet.png")
    plt.savefig(sheet, dpi=110)
    print("wrote", sheet)


if __name__ == "__main__":
    main()
