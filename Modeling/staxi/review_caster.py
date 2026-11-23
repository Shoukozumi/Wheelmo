"""Front-caster close-up: the CAD caster (top row) over the scanned right caster coloured by its
distance to the CAD (bottom row), from the same four angles. Distances are to the whole chair's CAD in
the scan pose (out/overlay_scan_pose_mm.stl, from build.py), so nearby frame tubes are scored fairly.

usage: python review_caster.py out.png
"""
import sys

import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

import model
import params as P
from build import SCAN_POSE
from review_render import COLORS, tessellate
from scan_tools import ROOT, load_scan_mm, render

SIDE = "r"  # the right caster: the scan saw it from more sides
RES = 450   # pixels per panel; the scan has ~1-2 mm triangles, so finer leaves gaps
VIEWS = [("front-right 3/4", 35, 20), ("right side (caster swivelled as scanned)", -90, 0), ("front", 0, 0),
         ("top", -90, 89.9)]


def main():
    out = sys.argv[1]
    links = model.build(brake_squeezed=False)
    T = model.link_world_transforms(links, SCAN_POSE)
    parts = [f"caster_fork_{SIDE}", f"caster_wheel_{SIDE}"]
    tris, nrms, cols = [], [], []
    for name in parts:
        L = links[name]
        for shape, kind in (L["paint"] if "paint" in L else [(L["shape"], "fork")]):
            t, n = tessellate(T[name] * shape, density=5)
            tris.append(t); nrms.append(n); cols.append(np.tile(COLORS[kind], (len(t), 1)))
    tri, nrm, col = np.concatenate(tris), np.concatenate(nrms), np.concatenate(cols)

    # scan around the same caster (below the frame housing), coloured by distance to the CAD
    scan, snrm = load_scan_mm()
    ax, ay = P.CASTER_AXIS
    c = scan.mean(1)
    sel = (np.hypot(c[:, 0] - ax, c[:, 1] + ay) < 110) & (c[:, 2] < P.CASTER_HOUSING_Z[0] + 2)
    scan, snrm = scan[sel], snrm[sel]
    cad = trimesh.load(str(ROOT / "out" / "overlay_scan_pose_mm.stl"), force="mesh")
    pts, _ = trimesh.sample.sample_surface_even(cad, 600_000, radius=1.0, seed=0)
    d, _ = cKDTree(pts).query(scan.mean(1), workers=-1)
    scol = np.empty((len(d), 3))
    scol[d <= 3] = [0.20, 0.65, 0.30]
    scol[(d > 3) & (d <= 6)] = [0.60, 0.80, 0.35]
    scol[(d > 6) & (d <= 12)] = [0.95, 0.80, 0.20]
    scol[d > 12] = [0.90, 0.20, 0.15]

    # one scale and centre per view for both rows, so CAD and scan line up panel by panel
    both = np.concatenate([tri, scan])
    fig, axs = plt.subplots(2, len(VIEWS), figsize=(9 * len(VIEWS), 18))
    for k, (title, az, el) in enumerate(VIEWS):
        a, e = np.radians(az), np.radians(el)
        fwd = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        right = np.cross([0, 0, 1], fwd) if abs(el) < 89.9 else np.cross(fwd, [np.cos(a), np.sin(a), 0])
        right /= np.linalg.norm(right)
        up = np.cross(fwd, right)
        cc = both.mean(1)
        u, v = cc @ right, cc @ up
        centre = np.array([(u.min() + u.max()) / 2, (v.min() + v.max()) / 2])
        scale = max(np.ptp(u), np.ptp(v)) / RES * 1.05
        for row, (tt, nn, cl) in enumerate(((tri, nrm, col), (scan, snrm, scol))):
            img, _ = render(tt, nn, cl, az, el, res=RES, scale=scale, center=centre)
            axs[row, k].imshow(img); axs[row, k].axis("off")
            axs[row, k].set_title(("CAD: " if row == 0 else "scan vs CAD: ") + title, fontsize=17)
    w = np.linalg.norm(np.cross(scan[:, 1] - scan[:, 0], scan[:, 2] - scan[:, 0]), axis=1)
    share = ", ".join(f"<= {t} mm {100 * w[d <= t].sum() / w.sum():.0f}%" for t in (3, 6, 12))
    fig.suptitle(f"Front caster: CAD (top) and scan coloured by distance to it (bottom): green <= 3 mm, "
                 f"light green 3-6, yellow 6-12, red > 12 mm.   Scan surface {share}", fontsize=19)
    plt.tight_layout(); plt.savefig(out, dpi=45)
    print("wrote", out, "| scan surface", share)


if __name__ == "__main__":
    main()
