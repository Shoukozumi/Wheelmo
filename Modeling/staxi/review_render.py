"""Shaded multi-view renders of the CAD model, coloured by part.

usage: python review_render.py out.png [--scan-pose]
"""
import sys
import tempfile
from pathlib import Path

import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from build123d import export_stl

import model
from build import SCAN_POSE
from scan_tools import render

COLORS = {"metal": (0.72, 0.74, 0.78), "cushion": (0.20, 0.30, 0.55), "wheel": (0.22, 0.22, 0.24),
          "fork": (0.55, 0.57, 0.6), "armrest": (0.15, 0.15, 0.15),
          "stripe_white": (0.93, 0.93, 0.93), "stripe_black": (0.08, 0.08, 0.08),
          "plastic": (0.22, 0.22, 0.24)}


def tessellate(shape, density=2.5):
    """Dense surface samples (as degenerate triangles) so the point-splat renderer shows solid surfaces."""
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "m.stl"
        export_stl(shape, str(p), tolerance=0.3, angular_tolerance=0.2)
        m = trimesh.load(p, force="mesh")
    pts, fi = trimesh.sample.sample_surface(m, int(m.area * density / 1.0), seed=0)
    return np.repeat(pts[:, None, :], 3, axis=1), m.face_normals[fi]


def main():
    out = sys.argv[1]
    scan_pose = "--scan-pose" in sys.argv
    links = model.build(brake_squeezed=not scan_pose)
    T = model.link_world_transforms(links, SCAN_POSE if scan_pose else None)
    tris, nrms, cols = [], [], []
    for name, L in links.items():
        if name == "base_link":
            groups = [(p[0], "metal" if i == 0 else "cushion") for i, p in enumerate(L["mass_parts"])]
        elif "paint" in L:
            groups = L["paint"]
        else:
            kind = "wheel" if "wheel" in name else ("fork" if "fork" in name else "armrest")
            groups = [(L["shape"], kind)]
        for shape, kind in groups:
            t, n = tessellate(T[name] * shape)
            tris.append(t); nrms.append(n); cols.append(np.tile(COLORS[kind], (len(t), 1)))
    tri, nrm, col = np.concatenate(tris), np.concatenate(nrms), np.concatenate(cols)
    views = [("front-right 3/4", 35, 20), ("back-left 3/4", 215, 25), ("right side", -90, 0),
             ("front", 0, 0), ("back", 180, 0), ("top", -90, 89.9)]
    fig, axs = plt.subplots(2, 3, figsize=(27, 18))
    for ax, (name, az, el) in zip(axs.ravel(), views):
        img, _ = render(tri, nrm, col, az, el, res=1000)
        ax.imshow(img); ax.set_title(name, fontsize=20); ax.axis("off")
    fig.suptitle("Staxi Standard - CAD model" + (" (scan pose)" if scan_pose else " (brake bar zip-tied, casters straight)"),
                 fontsize=22)
    plt.tight_layout(); plt.savefig(out, dpi=45)
    print("wrote", out, len(tri), "triangles")


if __name__ == "__main__":
    main()
