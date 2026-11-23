"""Orthographic reference views of the scan (and optionally the CAD model) with a millimetre grid.

usage: python blueprint.py out.png VIEW [--slab AXIS MIN MAX] [--box xmin,ymin,zmin,xmax,ymax,zmax]
                                       [--model model.stl] [--grid 50] [--res 1100]
VIEW: right (looking at the chair's right side, +X to the right), front, top, back
"""
import argparse

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from scan_tools import load_scan_mm, load_stl, render

VIEWS = {"right": (-90, 0, 0, 2), "front": (0, 0, 1, 2), "top": (-90, 90, 0, 1), "back": (180, 0, 1, 2)}
AX = {"x": 0, "y": 1, "z": 2}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("view", choices=VIEWS)
    ap.add_argument("--slab", nargs=3)
    ap.add_argument("--box")
    ap.add_argument("--model", help="CAD model STL (model frame, mm) drawn in orange over the scan")
    ap.add_argument("--grid", type=float, default=50)
    ap.add_argument("--res", type=int, default=1100)
    a = ap.parse_args()

    tri, nrm = load_scan_mm()
    cols = np.tile([0.72, 0.72, 0.76], (len(tri), 1))
    if a.model:
        mt, mn = load_stl(a.model)
        tri = np.concatenate([tri, mt]); nrm = np.concatenate([nrm, mn])
        cols = np.concatenate([cols * 0 + [0.80, 0.80, 0.84], np.tile([0.95, 0.45, 0.1], (len(mt), 1))])
    c = tri.mean(1)
    keep = np.ones(len(tri), bool)
    if a.slab:
        k = AX[a.slab[0]]
        keep &= (c[:, k] >= float(a.slab[1])) & (c[:, k] <= float(a.slab[2]))
    if a.box:
        b = np.array([float(s) for s in a.box.split(",")])
        keep &= ((c >= b[:3]) & (c <= b[3:])).all(1)
    tri, nrm, cols = tri[keep], nrm[keep], cols[keep]

    az, el, hax, vax = VIEWS[a.view]
    img, to_px = render(tri, nrm, cols, az, el, res=a.res)
    fig, ax = plt.subplots(figsize=(14, 14))
    ax.imshow(img)
    # grid lines in model coordinates along the two visible axes
    cc = tri.mean(1)
    ref = cc.mean(0)
    for axis in (hax, vax):
        lo, hi = cc[:, axis].min(), cc[:, axis].max()
        for g in np.arange(np.floor(lo / a.grid) * a.grid, hi + a.grid, a.grid):
            p0, p1 = ref.copy(), ref.copy()
            p0[axis] = p1[axis] = g
            other = vax if axis == hax else hax
            p0[other], p1[other] = cc[:, other].min() - 100, cc[:, other].max() + 100
            px = to_px(np.stack([p0, p1]))
            major = abs(g % (a.grid * 2)) < 1e-6
            ax.plot(px[:, 0], px[:, 1], color="tab:blue", lw=0.8 if major else 0.4, alpha=0.6 if major else 0.3)
            if major:
                lbl = f"{'xyz'[axis]}={g:.0f}"
                if axis == hax:
                    ax.text(px[0, 0], a.res - 5, lbl, fontsize=8, color="tab:blue", ha="center")
                else:
                    ax.text(5, px[0, 1], lbl, fontsize=8, color="tab:blue", va="center")
    ax.set_xlim(0, a.res); ax.set_ylim(a.res, 0); ax.axis("off")
    ax.set_title(f"{a.view} view" + (f", slab {a.slab}" if a.slab else "") + (f", box {a.box}" if a.box else ""))
    plt.tight_layout(); plt.savefig(a.out, dpi=70)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
