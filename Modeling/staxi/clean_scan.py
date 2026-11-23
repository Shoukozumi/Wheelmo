"""Raw phone scan (OBJ) -> floor removed, floor-aligned, mirror-aligned wheelchair STL.

usage: python clean_scan.py [scan/model.obj] [scan/wheelchair_clean_aligned.stl]

Steps:
1. Find the floor: RANSAC plane fit on 200k sampled vertices, refined by SVD on the inliers.
2. Floor-aligned frame (Z up, floor at Z = 0); drop everything within 8 units of the floor.
3. Keep the wheelchair: the largest connected piece above the floor plus every sizable piece within
   900 units of it (the scan of the chair is many disconnected fragments).
4. Heading: the chair is left/right mirror-symmetric, so search the yaw (1 deg steps) that best matches
   the cloud with its own mirror image.
5. Write a binary STL: X = chair's right, Y = forward, Z = up, floor at Z = 0.

Output stays in scan units (not mm); scan_tools.py applies the scale fitted to the Staxi specs and
moves it into the model frame.
"""
import re
import sys
from pathlib import Path

import numpy as np
from scipy import sparse
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent


def load_obj(path):
    """Vertex positions and triangle indices of an OBJ (texture/normal indices are dropped)."""
    lines = Path(path).read_bytes().split(b"\n")
    v = b" ".join(line[2:] for line in lines if line.startswith(b"v "))
    f = b" ".join(line[2:] for line in lines if line.startswith(b"f "))
    f = re.sub(rb"/[0-9]*", b"", f)  # "12/34" -> "12"
    V = np.fromstring(v.decode(), sep=" ").reshape(-1, 3)
    F = np.fromstring(f.decode(), sep=" ", dtype=np.int64).reshape(-1, 3) - 1
    return V, F


def find_floor(V, rng):
    """RANSAC + SVD floor plane. Returns (centre, normal pointing toward the chair, heights)."""
    best = (0, None)
    sample = V[rng.choice(len(V), 200_000, replace=False)]
    for _ in range(400):
        p = sample[rng.choice(len(sample), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        if np.linalg.norm(n) < 1e-6:
            continue
        n /= np.linalg.norm(n)
        cnt = np.count_nonzero(np.abs((sample - p[0]) @ n) < 10)
        if cnt > best[0]:
            best = (cnt, (n, p[0]))
    n, p0 = best[1]
    inl = V[np.abs((V - p0) @ n) < 10]
    c = inl.mean(0)
    n = np.linalg.svd(inl - c, full_matrices=False)[2][-1]
    h = (V - c) @ n
    if np.median(h) < 0:  # the chair side is positive
        n, h = -n, -h
    print(f"floor: {np.count_nonzero(np.abs(h) < 10)} inliers of {len(V)}, flatness {h[np.abs(h) < 10].std():.2f}")
    return c, n, h


def isolate_chair(P, F):
    """Mask of vertices belonging to the wheelchair (floor removed, nearby fragments kept)."""
    above = P[:, 2] > 8
    Fk = F[above[F].all(1)]
    adj = sparse.coo_matrix((np.ones(len(Fk) * 3), (Fk[:, [0, 1, 2]].ravel(), Fk[:, [1, 2, 0]].ravel())),
                            shape=(len(P), len(P)))
    ncomp, lab = connected_components(adj, directed=False)
    sizes = np.bincount(lab[Fk[:, 0]], minlength=ncomp)
    order = np.argsort(sizes)[::-1]
    ctr = np.median(P[lab == order[0]][:, :2], 0)
    keep = [k for k in order[:2000] if sizes[k] > 200 and np.linalg.norm(np.median(P[lab == k][:, :2], 0) - ctr) < 900]
    print(f"chair: {len(keep)} connected pieces")
    return np.isin(lab, keep)


def symmetry_yaw(C):
    """Yaw (deg) that best matches the chair with its left/right mirror image, and the XY centre."""
    centre = np.median(C[:, :2], 0)
    xy, z = C[::20, :2] - centre, C[::20, 2]
    best = None
    for deg in np.arange(0, 180, 1.0):
        t = np.radians(deg)
        R = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
        q = xy @ R.T
        q[:, 0] -= np.median(q[:, 0])
        score = np.median(cKDTree(np.c_[q, z]).query(np.c_[-q[:, 0], q[:, 1], z][::5])[0])
        if best is None or score < best[0]:
            best = (score, deg)
    print(f"symmetry yaw {best[1]:.0f} deg, mirror residual {best[0]:.1f} units")
    return best[1], centre


def write_stl(path, V, T, header):
    tri = V[T]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(T), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tri
    with open(path, "wb") as f:
        f.write(header.encode().ljust(80, b" "))
        f.write(np.uint32(len(T)).tobytes())
        rec.tofile(f)


def main():
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "scan" / "model.obj"
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else HERE / "scan" / "wheelchair_clean_aligned.stl"
    V, F = load_obj(src)
    print(f"{src.name}: {len(V)} vertices, {len(F)} triangles")

    c, n, h = find_floor(V, np.random.default_rng(0))
    a = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(n, a)
    x /= np.linalg.norm(x)
    y = np.cross(n, x)
    P = np.stack([(V - c) @ x, (V - c) @ y, h], 1)  # floor-aligned, Z up

    keep = isolate_chair(P, F)
    yaw, centre = symmetry_yaw(P[keep])
    t = np.radians(yaw)
    R = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    Q = P.copy()
    Q[:, :2] = (P[:, :2] - centre) @ R.T
    Q[:, 0] -= np.median(Q[keep][:, 0])  # symmetry plane at X = 0

    Fk = F[keep[F].all(1)]
    used = np.unique(Fk)
    remap = -np.ones(len(P), np.int64)
    remap[used] = np.arange(len(used))
    Vk = Q[used].astype(np.float32)
    Vk[:, 2] -= Vk[:, 2].min()
    write_stl(out, Vk, remap[Fk], "wheelchair: floor removed, floor-aligned, unscaled scan units")
    print(f"wrote {out}: {len(used)} vertices, {len(Fk)} triangles, size {np.ptp(Vk, 0).round(0)} units")


if __name__ == "__main__":
    main()
