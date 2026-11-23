"""Extract tube centrelines from the scan: voxelize -> close/fill -> 3D thinning -> branch graph.

Writes data/skeleton.npz with skeleton voxel centres (mm), per-voxel neighbour counts,
branch ids, and the local tube radius from the distance transform.
"""
import numpy as np
from scipy import ndimage
from skimage.morphology import skeletonize, ball

from scan_tools import load_scan_mm, ROOT

VOX = 4.0  # mm

tri, _ = load_scan_mm()
# dense surface samples: vertices + centroids + edge midpoints
pts = np.concatenate([tri.reshape(-1, 3), tri.mean(1), (tri[:, 0] + tri[:, 1]) / 2,
                      (tri[:, 1] + tri[:, 2]) / 2, (tri[:, 2] + tri[:, 0]) / 2])
lo = pts.min(0) - 5 * VOX
idx = np.floor((pts - lo) / VOX).astype(int)
shape = idx.max(0) + 5
occ = np.zeros(shape, bool)
occ[tuple(idx.T)] = True
print("grid", shape, "surface voxels", occ.sum())

# Seal the shell and fill tube interiors. Closing radius ~ half the largest tube's inner gap.
solid = ndimage.binary_closing(occ, structure=ball(3), iterations=1)
solid = ndimage.binary_fill_holes(solid)
# remove specks
lab, n = ndimage.label(solid)
sizes = ndimage.sum(solid, lab, range(1, n + 1))
solid = np.isin(lab, 1 + np.flatnonzero(sizes > 200))
print("solid voxels", solid.sum(), "components kept", (sizes > 200).sum())

dist = ndimage.distance_transform_edt(solid) * VOX  # mm to the nearest outside voxel
skel = skeletonize(solid)  # Lee 1994 thinning for 3D input
sk = np.argwhere(skel)
print("skeleton voxels", len(sk))

# neighbour count in the 26-neighbourhood: 1 = endpoint, 2 = along a branch, >=3 = junction
nb = ndimage.convolve(skel.astype(np.uint8), np.ones((3, 3, 3), np.uint8), mode="constant")[skel] - 1
# branches = skeleton with junctions removed, then connected components
body = skel.copy()
body[tuple(sk[nb >= 3].T)] = False
blab, nbr = ndimage.label(body, structure=np.ones((3, 3, 3)))
branch = blab[skel]
print("branches", nbr, "junction voxels", (nb >= 3).sum(), "endpoints", (nb == 1).sum())

np.savez(ROOT / "data" / "skeleton.npz", p=lo + (sk + 0.5) * VOX, nb=nb, branch=branch,
         radius=dist[skel], vox=VOX)
