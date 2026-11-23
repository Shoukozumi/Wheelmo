"""Rear-wheel close-up (left side): the CAD wheel, cog, parking brake and nearby frame (top row), the scan
coloured by its distance to the CAD (middle row), and the CAD with the brake bar squeezed, pawl lifted
(bottom row), from the same angles. Distances are to the whole chair's CAD in the scan pose
(out/overlay_scan_pose_mm.stl, from build.py).

usage: python review_rear.py out.png [latch.png]
latch.png: CAD close-up of the left latch (cog ring + pawl), brake released and squeezed.
"""
import sys

import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

from build123d import Pos

import model
import params as P
from build import SCAN_POSE
from review_render import COLORS, tessellate
from scan_tools import ROOT, load_scan_mm, render

RES = 520
BOX = np.array([[-140, 200, -10], [125, 365, 330]])  # left rear wheel, brake rod up to the holder
VIEWS = [("from the chair centre", -90, 0), ("inside, rear 3/4", -140, 22), ("back", 180, 0),
         ("outside, rear 3/4", 140, 15)]


def cad_triangles(brake_squeezed, pose):
    links = model.build(brake_squeezed=brake_squeezed)
    T = model.link_world_transforms(links, pose)
    tris, nrms, cols = [], [], []
    for name in ("base_link", "rear_wheel_l"):
        L = links[name]
        for shape, kind in (L["paint"] if "paint" in L else [(L["shape"], "metal")]):
            t, n = tessellate(T[name] * shape, density=5)
            keep = ((t[:, 0] >= BOX[0]) & (t[:, 0] <= BOX[1])).all(1)
            tris.append(t[keep]); nrms.append(n[keep]); cols.append(np.tile(COLORS[kind], (keep.sum(), 1)))
    return np.concatenate(tris), np.concatenate(nrms), np.concatenate(cols)


def latch_closeup(out):
    """CAD only: the left wheel's cog ring and the pawl (brown), brake released and squeezed."""
    _, _, paint = model.build_rear_wheel(-1)
    W = Pos(P.REAR_WHEEL_X, P.REAR_WHEEL_Y, P.REAR_AXLE_Z)
    rows = []
    for lift, lab in ((0.0, "brake released: pawl in a trough"), (P.BRAKE_LIFT, "brake squeezed (sim): pawl lifted")):
        tris, nrms, cols = [], [], []
        for shp, kind in paint:
            t, n = tessellate(W * shp, density=12)
            tris.append(t); nrms.append(n); cols.append(np.tile(COLORS[kind], (len(t), 1)))
        for k, part in enumerate(model.brake_parts(1, lift)):  # parts 5+ are the pawl
            t, n = tessellate(part, density=12)
            tris.append(t); nrms.append(n)
            cols.append(np.tile((0.75, 0.55, 0.35) if k >= 5 else COLORS["metal"], (len(t), 1)))
        tri, nrm, col = np.concatenate(tris), np.concatenate(nrms), np.concatenate(cols)
        c = tri.mean(1)
        keep = (c[:, 0] > -75) & (c[:, 0] < 55) & (c[:, 2] > 95) & (c[:, 2] < 275) & (c[:, 1] > 240) & (c[:, 1] < 300)
        rows.append((lab, tri[keep], nrm[keep], col[keep]))
    views = [("from the chair centre", -90, 0), ("inside, rear 3/4", -135, 25), ("back", 180, 0)]
    fig, axs = plt.subplots(2, 3, figsize=(27, 18))
    for i, (lab, tri, nrm, col) in enumerate(rows):
        for j, (vl, az, el) in enumerate(views):
            img, _ = render(tri, nrm, col, az, el, res=700)
            axs[i, j].imshow(img); axs[i, j].axis("off"); axs[i, j].set_title(f"{lab}: {vl}", fontsize=15)
    plt.tight_layout(); plt.savefig(out, dpi=45)
    print("wrote", out)


def main():
    out = sys.argv[1]
    if len(sys.argv) > 2:
        latch_closeup(sys.argv[2])
    rows = [cad_triangles(False, SCAN_POSE), None, cad_triangles(True, {})]
    scan, snrm = load_scan_mm()
    c = scan.mean(1)
    sel = ((c >= BOX[0]) & (c <= BOX[1])).all(1)
    scan, snrm = scan[sel], snrm[sel]
    cad = trimesh.load(str(ROOT / "out" / "overlay_scan_pose_mm.stl"), force="mesh")
    tmin, tmax = cad.triangles.min(1), cad.triangles.max(1)  # keep every triangle whose bbox meets the box
    near = cad.submesh([np.where(((tmax >= BOX[0] - 20) & (tmin <= BOX[1] + 20)).all(1))[0]], append=True)
    pts, _ = trimesh.sample.sample_surface(near, 3_000_000, seed=0)
    pts = pts[((pts >= BOX[0] - 20) & (pts <= BOX[1] + 20)).all(1)]
    d, _ = cKDTree(pts).query(scan.mean(1), workers=-1)
    scol = np.empty((len(d), 3))
    scol[d <= 2] = [0.20, 0.65, 0.30]
    scol[(d > 2) & (d <= 4)] = [0.60, 0.80, 0.35]
    scol[(d > 4) & (d <= 8)] = [0.95, 0.80, 0.20]
    scol[d > 8] = [0.90, 0.20, 0.15]
    rows[1] = (scan, snrm, scol)

    both = np.concatenate([rows[0][0], scan])
    fig, axs = plt.subplots(3, len(VIEWS), figsize=(9 * len(VIEWS), 27))
    titles = ("CAD, scan pose: ", "scan vs CAD: ", "CAD, brake squeezed: ")
    for k, (title, az, el) in enumerate(VIEWS):
        a, e = np.radians(az), np.radians(el)
        fwd = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        right = np.cross([0, 0, 1], fwd)
        right /= np.linalg.norm(right)
        up = np.cross(fwd, right)
        cc = both.mean(1)
        u, v = cc @ right, cc @ up
        centre = np.array([(u.min() + u.max()) / 2, (v.min() + v.max()) / 2])
        scale = max(np.ptp(u), np.ptp(v)) / RES * 1.03
        for row, (tt, nn, cl) in enumerate(rows):
            img, _ = render(tt, nn, cl, az, el, res=RES, scale=scale, center=centre)
            axs[row, k].imshow(img); axs[row, k].axis("off")
            axs[row, k].set_title(titles[row] + title, fontsize=17)
    w = np.linalg.norm(np.cross(scan[:, 1] - scan[:, 0], scan[:, 2] - scan[:, 0]), axis=1)
    share = ", ".join(f"<= {t} mm {100 * w[d <= t].sum() / w.sum():.0f}%" for t in (2, 4, 8))
    fig.suptitle(f"Left rear wheel + parking brake. Middle row: scan coloured by distance to the CAD: green <= 2 mm, "
                 f"light green 2-4, yellow 4-8, red > 8 mm.   Scan surface {share}", fontsize=19)
    plt.tight_layout(); plt.savefig(out, dpi=45)
    print("wrote", out, "| scan surface", share)


if __name__ == "__main__":
    main()
