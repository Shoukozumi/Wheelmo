"""Procedural triangle meshes for the wheelchair's visual geometry (pure numpy, no simulator needed).

* :func:`revolve` spins a 2D profile around the y axis: tyres, rims, hubs, the LiDAR puck.
* :func:`rounded_box` makes a box with filleted edges: cushions, the battery/motor shroud, armrest pads.

Meshes are closed, outward-facing and carry smooth per-vertex normals. A repeated profile point in
:func:`revolve` creates a hard edge.
"""

from __future__ import annotations

from typing import NamedTuple

import numpy as np


class TriMesh(NamedTuple):
    points: np.ndarray  # (N, 3) float32
    triangles: np.ndarray  # (M, 3) int32
    normals: np.ndarray  # (N, 3) float32, unit length

    def signed_volume(self) -> float:
        a, b, c = (self.points[self.triangles[:, i]].astype(np.float64) for i in range(3))
        return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


def _vertex_normals(points: np.ndarray, triangles: np.ndarray) -> np.ndarray:
    a, b, c = (points[triangles[:, i]] for i in range(3))
    face = np.cross(b - a, c - a)  # area-weighted
    normals = np.zeros_like(points)
    for i in range(3):
        np.add.at(normals, triangles[:, i], face)
    length = np.linalg.norm(normals, axis=1, keepdims=True)
    return normals / np.where(length > 1e-12, length, 1.0)


def _finalize(points: np.ndarray, triangles: np.ndarray, normals: np.ndarray | None = None) -> TriMesh:
    points = np.asarray(points, dtype=np.float64)
    triangles = np.asarray(triangles, dtype=np.int64)
    # drop vertices no triangle uses (left over from removed zero-area triangles)
    used = np.unique(triangles)
    remap = np.full(len(points), -1, dtype=np.int64)
    remap[used] = np.arange(len(used))
    points, triangles = points[used], remap[triangles]
    if normals is not None:
        normals = np.asarray(normals)[used]
    mesh = TriMesh(points, triangles, np.zeros_like(points))
    if mesh.signed_volume() < 0.0:  # make the winding outward-facing
        triangles = triangles[:, ::-1]
    if normals is None:
        normals = _vertex_normals(points, triangles)
    return TriMesh(points.astype(np.float32), triangles.astype(np.int32), np.asarray(normals, dtype=np.float32))


def revolve(profile: list[tuple[float, float]], segments: int = 48) -> TriMesh:
    """Revolve a closed 2D profile of (radius, y) points around the y axis.

    The profile is implicitly closed (last point connects back to the first). Points with radius 0 lie on
    the axis and close the surface there. Repeat a point to get a hard edge instead of a smooth crease.
    """
    prof = np.asarray(profile, dtype=np.float64)
    angles = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    cos, sin = np.cos(angles), np.sin(angles)
    # ring k, segment j -> (r cos, y, r sin): rotation about y
    points = np.stack(
        [prof[:, None, 0] * cos[None], np.broadcast_to(prof[:, None, 1], (len(prof), segments)), prof[:, None, 0] * sin[None]],
        axis=-1,
    ).reshape(-1, 3)
    triangles = []
    n = len(prof)
    for k in range(n):
        k2 = (k + 1) % n
        for j in range(segments):
            j2 = (j + 1) % segments
            a, b, c, d = k * segments + j, k * segments + j2, k2 * segments + j2, k2 * segments + j
            triangles.append((a, b, c))
            triangles.append((a, c, d))
    triangles = np.asarray(triangles)
    # drop triangles with zero area (bands between repeated profile points, fans at the axis)
    p = points[triangles]
    area = np.linalg.norm(np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0]), axis=1)
    return _finalize(points, triangles[area > 1e-12])


def rounded_box(size: tuple[float, float, float], radius: float, edge_segments: int = 3) -> TriMesh:
    """Axis-aligned box of full ``size`` centred at the origin, with all edges rounded by ``radius``."""
    half = 0.5 * np.asarray(size, dtype=np.float64)
    r = float(min(radius, 0.999 * half.min()))
    inner = half - r
    # sample each face densely inside the fillet band so the rounding is smooth
    band = r * np.tan(np.linspace(0.0, np.pi / 4.0, edge_segments + 1)[1:])
    coords = [np.concatenate([-(inner[a] + band[::-1]), [-inner[a], inner[a]], inner[a] + band]) for a in range(3)]
    points, normals, triangles = [], [], []
    for axis in range(3):
        u_axis, v_axis = [a for a in range(3) if a != axis]
        for sign in (-1.0, 1.0):
            us, vs = coords[u_axis], coords[v_axis]
            grid = np.zeros((len(us), len(vs), 3))
            grid[..., axis] = sign * half[axis]
            grid[..., u_axis] = us[:, None]
            grid[..., v_axis] = vs[None, :]
            q = grid.reshape(-1, 3)
            core = np.clip(q, -inner, inner)
            offset = q - core
            direction = offset / np.linalg.norm(offset, axis=1, keepdims=True)
            base = sum(len(p) for p in points)
            points.append(core + r * direction)
            normals.append(direction)
            nu, nv = len(us), len(vs)
            idx = np.arange(nu * nv).reshape(nu, nv) + base
            quads = np.stack([idx[:-1, :-1], idx[1:, :-1], idx[1:, 1:], idx[:-1, 1:]], axis=-1).reshape(-1, 4)
            tris = np.concatenate([quads[:, [0, 1, 2]], quads[:, [0, 2, 3]]])
            # orient this face so its triangles point away from the box centre
            pts = np.concatenate(points)
            a, b, c = pts[tris[:, 0]], pts[tris[:, 1]], pts[tris[:, 2]]
            if np.einsum("ij,ij->i", np.cross(b - a, c - a), np.concatenate(normals)[tris[:, 0]]).sum() < 0:
                tris = tris[:, ::-1]
            triangles.append(tris)
    points, normals, triangles = np.concatenate(points), np.concatenate(normals), np.concatenate(triangles)
    p = points[triangles]
    area = np.linalg.norm(np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0]), axis=1)
    return _finalize(points, triangles[area > 1e-12], normals)


def rounded_rect_profile(
    r_inner: float, r_outer: float, width: float, corner: float, corner_segments: int = 4
) -> list[tuple[float, float]]:
    """Closed (radius, y) profile of a rounded rectangle, for :func:`revolve` (tyres, rims)."""
    c = min(corner, 0.5 * (r_outer - r_inner), 0.5 * width)
    w = 0.5 * width
    pts: list[tuple[float, float]] = []
    corners = (  # (centre radius, centre y, start angle) walking around the section
        (r_outer - c, w - c, 0.0),
        (r_inner + c, w - c, 0.5 * np.pi),
        (r_inner + c, -w + c, np.pi),
        (r_outer - c, -w + c, 1.5 * np.pi),
    )
    for cr, cy, start in corners:
        for t in np.linspace(start, start + 0.5 * np.pi, corner_segments + 1):
            pts.append((cr + c * np.cos(t), cy + c * np.sin(t)))
    return pts


def disc_profile(radius: float, width: float, bevel: float = 0.0) -> list[tuple[float, float]]:
    """Closed (radius, y) profile of a solid disc/cylinder along y with optional bevelled rim, for :func:`revolve`."""
    w = 0.5 * width
    b = min(bevel, 0.5 * w, 0.5 * radius)
    pts = [(0.0, -w), (radius - b, -w), (radius - b, -w)]  # repeated point -> hard edge on the face
    if b > 0:
        pts += [(radius, -w + b), (radius, -w + b), (radius, w - b), (radius, w - b)]
    else:
        pts += [(radius, -w), (radius, w)]
    pts += [(radius - b, w), (radius - b, w), (0.0, w)]
    return pts
