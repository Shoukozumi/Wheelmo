"""Colour the scan by its distance to the CAD model (posed like the scan).

Green = the CAD surface is within ~12 mm (the scan's tubes are inflated a few mm, so this is a match),
yellow = 12-25 mm, red = >25 mm (geometry the CAD is missing or has misplaced).

usage: python deviation.py out.png [model_stl]
"""
import sys

import numpy as np
import trimesh
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from scan_tools import load_scan_mm, render, ROOT

out = sys.argv[1]
model_stl = sys.argv[2] if len(sys.argv) > 2 else str(ROOT / "out" / "overlay_scan_pose_mm.stl")

tri, nrm = load_scan_mm()
cad = trimesh.load(model_stl, force="mesh")
samples, _ = trimesh.sample.sample_surface_even(cad, 600_000, radius=1.0)
d, _ = cKDTree(samples).query(tri.mean(1), workers=-1)

cols = np.empty((len(tri), 3))
cols[d <= 12] = [0.30, 0.75, 0.35]
cols[(d > 12) & (d <= 25)] = [0.95, 0.80, 0.20]
cols[d > 25] = [0.90, 0.20, 0.15]
area = np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1) / 2
for lo, hi, name in ((0, 12, "match"), (12, 25, "near"), (25, 1e9, "off")):
    m = (d > lo) & (d <= hi)
    print(f"{name:6s} {100 * area[m].sum() / area.sum():5.1f}% of scan surface")

views = [("front-right 3/4", 35, 20), ("back-left 3/4", 215, 25), ("right side", -90, 0),
         ("front", 0, 0), ("back", 180, 0), ("top", -90, 89.9)]
fig, axs = plt.subplots(2, 3, figsize=(27, 18))
for ax, (name, az, el) in zip(axs.ravel(), views):
    img, _ = render(tri, nrm, cols, az, el, res=1000)
    ax.imshow(img); ax.set_title(name, fontsize=20); ax.axis("off")
fig.suptitle("Scan coloured by distance to CAD:  green <12 mm   yellow 12-25 mm   red >25 mm", fontsize=22)
plt.tight_layout(); plt.savefig(out, dpi=45)
print("wrote", out)
