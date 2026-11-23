"""Footrest close-up (left side): the CAD footrest and the frame around it (top row) and the scan coloured by its
distance to the CAD (bottom row), from the same angles. Distances are to the whole chair's CAD in the scan
pose (out/overlay_scan_pose_mm.stl, from build.py).

usage: python review_footrest.py out.png
"""
import sys

import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

import model
from build import SCAN_POSE
from review_render import COLORS, tessellate
from scan_tools import ROOT, load_scan_mm, render

RES = 420
BOX = np.array([[590, 0, 85], [770, 225, 300]])  # left footrest, the downtube's lower end, the frame corner
VIEWS = [("outside, level (the U face-on)", 90, 5), ("front-outside, from above", 30, 35),
         ("rear-inside, from above", -150, 40), ("front", 0, 0)]


def main():
    out = sys.argv[1]
    links = model.build(brake_squeezed=False)
    T = model.link_world_transforms(links, SCAN_POSE)
    tris, nrms, cols = [], [], []
    for name in ("base_link", "caster_fork_l", "caster_wheel_l"):
        L = links[name]
        for shape, kind in (L["paint"] if "paint" in L else [(L["shape"], "metal" if name == "base_link" else "fork")]):
            t, n = tessellate(T[name] * shape, density=6)
            keep = ((t[:, 0] >= BOX[0]) & (t[:, 0] <= BOX[1])).all(1)
            tris.append(t[keep]); nrms.append(n[keep]); cols.append(np.tile(COLORS[kind], (keep.sum(), 1)))
    tri, nrm, col = np.concatenate(tris), np.concatenate(nrms), np.concatenate(cols)
    scan, snrm = load_scan_mm()
    c = scan.mean(1)
    sel = ((c >= BOX[0]) & (c <= BOX[1])).all(1)
    scan, snrm = scan[sel], snrm[sel]
    cad = trimesh.load(str(ROOT / "out" / "overlay_scan_pose_mm.stl"), force="mesh")
    tmin, tmax = cad.triangles.min(1), cad.triangles.max(1)
    near = cad.submesh([np.where(((tmax >= BOX[0] - 20) & (tmin <= BOX[1] + 20)).all(1))[0]], append=True)
    pts, _ = trimesh.sample.sample_surface(near, 3_000_000, seed=0)
    pts = pts[((pts >= BOX[0] - 20) & (pts <= BOX[1] + 20)).all(1)]
    d, _ = cKDTree(pts).query(scan.mean(1), workers=-1)
    scol = np.empty((len(d), 3))
    scol[d <= 2] = [0.20, 0.65, 0.30]
    scol[(d > 2) & (d <= 4)] = [0.60, 0.80, 0.35]
    scol[(d > 4) & (d <= 8)] = [0.95, 0.80, 0.20]
    scol[d > 8] = [0.90, 0.20, 0.15]
    both = np.concatenate([tri, scan])
    fig, axs = plt.subplots(2, len(VIEWS), figsize=(9 * len(VIEWS), 18))
    for k, (title, az, el) in enumerate(VIEWS):
        a, e = np.radians(az), np.radians(el)
        fwd = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        right = np.cross([0, 0, 1], fwd); right /= np.linalg.norm(right)
        up = np.cross(fwd, right)
        cc = both.mean(1)
        u, v = cc @ right, cc @ up
        centre = np.array([(u.min() + u.max()) / 2, (v.min() + v.max()) / 2])
        scale = max(np.ptp(u), np.ptp(v)) / RES * 1.03
        for row, (tt, nn, cl) in enumerate(((tri, nrm, col), (scan, snrm, scol))):
            img, _ = render(tt, nn, cl, az, el, res=RES, scale=scale, center=centre)
            axs[row, k].imshow(img); axs[row, k].axis("off")
            axs[row, k].set_title(("CAD: " if row == 0 else "scan vs CAD: ") + title, fontsize=17)
    w = np.linalg.norm(np.cross(scan[:, 1] - scan[:, 0], scan[:, 2] - scan[:, 0]), axis=1)
    share = ", ".join(f"<= {t} mm {100 * w[d <= t].sum() / w.sum():.0f}%" for t in (2, 4, 8))
    fig.suptitle(f"Left footrest. Bottom row: scan coloured by distance to the CAD: green <= 2 mm, light green 2-4, "
                 f"yellow 4-8, red > 8 mm.   Scan surface {share}", fontsize=19)
    plt.tight_layout(); plt.savefig(out, dpi=45)
    print("wrote", out, "| scan surface", share)


if __name__ == "__main__":
    main()
