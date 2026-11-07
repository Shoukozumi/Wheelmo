import math

import numpy as np
import pytest

from wheelmo_sim.meshes import disc_profile, revolve, rounded_box, rounded_rect_profile


def _check_closed_outward(mesh):
    assert mesh.signed_volume() > 0.0, "winding must face outward"
    assert np.allclose(np.linalg.norm(mesh.normals, axis=1), 1.0, atol=1e-5)
    assert mesh.triangles.min() >= 0 and mesh.triangles.max() < len(mesh.points)


def test_rounded_box_volume_and_bounds():
    size, r = (0.46, 0.46, 0.08), 0.03
    mesh = rounded_box(size, r)
    _check_closed_outward(mesh)
    assert np.allclose(mesh.points.max(axis=0) - mesh.points.min(axis=0), size, atol=1e-6)
    # box minus the material removed by rounding the 12 edges and 8 corners
    a, b, c = size
    edge_loss = (1 - math.pi / 4) * (2 * r) ** 2 * (a + b + c - 6 * r)
    corner_loss = 8 * (r**3 - (4 / 3) * math.pi * r**3 / 8)
    assert mesh.signed_volume() == pytest.approx(a * b * c - edge_loss - corner_loss, rel=0.02)
    # every normal points away from the centre on a convex shape
    assert (np.einsum("ij,ij->i", mesh.normals, mesh.points) > 0).all()


def test_revolved_disc_and_tyre():
    disc = revolve(disc_profile(0.04, 0.06), 64)
    _check_closed_outward(disc)
    assert disc.signed_volume() == pytest.approx(math.pi * 0.04**2 * 0.06, rel=0.01)

    tyre = revolve(rounded_rect_profile(0.128, 0.17, 0.07, 0.015), 64)
    _check_closed_outward(tyre)
    radius = np.linalg.norm(tyre.points[:, [0, 2]], axis=1)
    assert radius.max() == pytest.approx(0.17, abs=1e-6)
    assert radius.min() == pytest.approx(0.128, abs=1e-6)
    # flat tread: points at the outer radius span the width between the rounded corners
    tread = tyre.points[np.isclose(radius, 0.17, atol=1e-6)]
    assert tread[:, 1].max() == pytest.approx(0.035 - 0.015, abs=1e-6)
