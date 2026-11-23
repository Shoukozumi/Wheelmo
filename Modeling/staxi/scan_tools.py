"""Scan utilities: convert the cleaned scan to the model frame and render reference views.

Model frame (used by the CAD model and the URDF):
    X forward, Y left, Z up, millimetres.
    Origin on the floor, midway between the rear wheels, directly below the rear axle.
"""
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
SCAN_STL = ROOT / "scan" / "wheelchair_clean_aligned.stl"  # made by clean_scan.py
SCAN_MM = ROOT / "data" / "scan_mm.npz"

# Scale fitted to Staxi Standard published specs (width 27", seat-to-floor 21.7", 8" rear wheels).
SCALE_MM_PER_UNIT = 0.2935
REAR_AXLE_SCAN_Y = -57.0  # rear wheel hub centre, front/back position in scan units


def load_stl(path):
    with open(path, "rb") as f:
        f.seek(80)
        n = int(np.frombuffer(f.read(4), np.uint32)[0])
        rec = np.fromfile(f, dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")], count=n)
    return rec["v"].astype(np.float64), rec["n"].astype(np.float64)


def scan_to_model(p):
    """Scan frame (X = chair right, Y = forward, Z = up, scan units) -> model frame (mm)."""
    p = np.asarray(p, dtype=np.float64)
    out = np.empty_like(p)
    out[..., 0] = (p[..., 1] - REAR_AXLE_SCAN_Y) * SCALE_MM_PER_UNIT
    out[..., 1] = -p[..., 0] * SCALE_MM_PER_UNIT
    out[..., 2] = p[..., 2] * SCALE_MM_PER_UNIT
    return out


def load_scan_mm():
    """Triangles (N, 3, 3) and face normals (N, 3) of the scan in the model frame, cached."""
    if SCAN_MM.exists():
        d = np.load(SCAN_MM)
        return d["tri"], d["nrm"]
    tri, nrm = load_stl(SCAN_STL)
    tri = scan_to_model(tri)
    nrm = np.stack([nrm[:, 1], -nrm[:, 0], nrm[:, 2]], 1)
    SCAN_MM.parent.mkdir(parents=True, exist_ok=True)
    np.savez(SCAN_MM, tri=tri.astype(np.float32), nrm=nrm.astype(np.float32))
    return tri, nrm


def render(tri, nrm, colors, az, el, res=900, scale=None, center=None):
    """Shaded z-buffer splat of triangle centroids.

    az=0 looks at the chair front (camera on +X), az=90 at its left side (camera on +Y).
    Returns the image and a function mapping model-frame points to pixel coordinates.
    """
    a, e = np.radians(az), np.radians(el)
    fwd = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])  # scene -> camera
    right = np.cross([0, 0, 1], fwd) if abs(el) < 89.9 else np.cross(fwd, [np.cos(a), np.sin(a), 0])
    right /= np.linalg.norm(right)
    up = np.cross(fwd, right)
    c = tri.mean(1)
    u, v, d = c @ right, c @ up, c @ fwd
    if center is None:
        center = np.array([(u.min() + u.max()) / 2, (v.min() + v.max()) / 2])
    if scale is None:
        scale = max(np.ptp(u), np.ptp(v)) / res * 1.05
    half = res // 2
    iu = np.round((u - center[0]) / scale + half).astype(int)
    iv = np.round((v - center[1]) / scale + half).astype(int)
    ok = (iu >= 0) & (iu < res) & (iv >= 0) & (iv < res)
    order = np.argsort(d[ok])
    light = np.abs(nrm[ok] @ (fwd * 0.8 + up * 0.4 + right * 0.3))
    shade = np.clip(0.3 + 0.7 * light, 0, 1)
    img = np.ones((res, res, 3))
    img[iv[ok][order], iu[ok][order]] = colors[ok][order] * shade[order, None]

    def to_px(p):
        p = np.atleast_2d(p)
        return np.c_[(p @ right - center[0]) / scale + half, res - 1 - ((p @ up - center[1]) / scale + half)]

    return img[::-1], to_px


if __name__ == "__main__":
    tri, _ = load_scan_mm()
    c = tri.reshape(-1, 3)
    lo, hi = np.percentile(c, 0.5, 0), np.percentile(c, 99.5, 0)
    print("scan in model frame (mm): lo", lo.round(0), "hi", hi.round(0), "size", (hi - lo).round(0))
