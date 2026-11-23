"""Turn the skeleton into ordered tube polylines (one per branch between junctions).

Keeps only skeleton voxels whose local radius looks like a tube (filters out cushion and
wheel shells), orders each branch from one end to the other, and simplifies it with
Ramer-Douglas-Peucker so the remaining vertices are the bend points.
"""
import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components, shortest_path

from scan_tools import ROOT

MIN_R, MIN_LEN, RDP_TOL = 6.5, 40.0, 5.0


def rdp(pts, tol):
    if len(pts) < 3:
        return pts
    a, b = pts[0], pts[-1]
    ab = b - a
    n = np.linalg.norm(ab)
    d = np.linalg.norm(np.cross(pts - a, ab), axis=1) / n if n > 1e-9 else np.linalg.norm(pts - a, axis=1)
    i = int(np.argmax(d))
    if d[i] <= tol:
        return np.stack([a, b])
    return np.concatenate([rdp(pts[: i + 1], tol)[:-1], rdp(pts[i:], tol)])


def order_branch(p, vox):
    """Order points of one thin branch: path between the two mutually farthest points."""
    from scipy.spatial import cKDTree
    t = cKDTree(p)
    pairs = t.query_pairs(vox * 1.8, output_type="ndarray")
    if len(pairs) == 0:
        return p
    w = np.linalg.norm(p[pairs[:, 0]] - p[pairs[:, 1]], axis=1)
    g = coo_matrix((w, (pairs[:, 0], pairs[:, 1])), shape=(len(p), len(p)))
    d0 = shortest_path(g, directed=False, indices=0)
    a = int(np.nanargmax(np.where(np.isfinite(d0), d0, -1)))
    da, pred = shortest_path(g, directed=False, indices=a, return_predecessors=True)
    b = int(np.nanargmax(np.where(np.isfinite(da), da, -1)))
    path = [b]
    while path[-1] != a and pred[path[-1]] >= 0:
        path.append(pred[path[-1]])
    return p[path[::-1]]


def main():
    d = np.load(ROOT / "data" / "skeleton.npz")
    p, r, vox = d["p"], d["radius"], float(d["vox"])
    keep = r >= MIN_R
    p, r = p[keep], r[keep]
    # rebuild voxel grid of tube skeleton, find junctions, split into branches
    ijk = np.round((p - p.min(0)) / vox).astype(int)
    grid = np.zeros(ijk.max(0) + 3, bool)
    grid[tuple((ijk + 1).T)] = True
    nb = ndimage.convolve(grid.astype(np.uint8), np.ones((3, 3, 3), np.uint8), mode="constant")[tuple((ijk + 1).T)] - 1
    body = grid.copy()
    body[tuple((ijk[nb >= 3] + 1).T)] = False
    lab, n = ndimage.label(body, structure=np.ones((3, 3, 3)))
    bl = lab[tuple((ijk + 1).T)]
    out = []
    for k in range(1, n + 1):
        m = bl == k
        if m.sum() < 4:
            continue
        q = order_branch(p[m], vox)
        L = np.linalg.norm(np.diff(q, axis=0), axis=1).sum()
        if L < MIN_LEN:
            continue
        poly = rdp(q, RDP_TOL)
        out.append(dict(id=len(out), length=L, radius=float(np.median(r[m])), poly=poly, pts=q))
    out.sort(key=lambda b: -b["length"])
    for i, b in enumerate(out):
        b["id"] = i
    np.save(ROOT / "data" / "tube_paths.npy", np.array(out, dtype=object), allow_pickle=True)
    for b in out:
        verts = " -> ".join(f"({v[0]:.0f},{v[1]:.0f},{v[2]:.0f})" for v in b["poly"])
        print(f"#{b['id']:2d} L={b['length']:5.0f} r={b['radius']:4.1f}  {verts}")


if __name__ == "__main__":
    main()
