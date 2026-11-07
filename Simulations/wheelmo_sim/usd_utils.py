"""Low-level USD authoring helpers shared by the wheelchair and corridor assets.

``pxr`` is imported lazily so modules using these helpers import cleanly before the app launches.

Visual vs. collision convention (it matters for every renderer):

* Visual geometry carries no physics schema and is what viewers draw.
* Collision proxies (``proxy=True``) get ``CollisionAPI`` and USD ``purpose = guide``. Newton's importer hides
  them in viewers (it also hides any collider on a body that has visual geometry). They are still hit by the
  LiDAR and still drawn by camera sensors sharing the ray-cast BVH, so keep every proxy *inside* its visual.
* A shape that is both (``collider=True``, no proxy) is fine on bodies without separate visuals, e.g. the
  corridor's walls and clutter.
"""

from __future__ import annotations

import math

import numpy as np

from .meshes import TriMesh

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # (x, y, z, w)
IDENTITY: Quat = (0.0, 0.0, 0.0, 1.0)


def pxr():
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, Vt  # noqa: PLC0415

    return Gf, Sdf, UsdGeom, UsdPhysics, Vt


def axis_angle_quat(axis: Vec3, angle: float) -> Quat:
    s = math.sin(0.5 * angle)
    n = math.sqrt(sum(a * a for a in axis))
    return (axis[0] / n * s, axis[1] / n * s, axis[2] / n * s, math.cos(0.5 * angle))


def quat_z_to(direction: Vec3) -> Quat:
    """Shortest rotation taking +z onto ``direction``."""
    dx, dy, dz = direction
    n = math.sqrt(dx * dx + dy * dy + dz * dz)
    dx, dy, dz = dx / n, dy / n, dz / n
    if dz < -0.999999:
        return (1.0, 0.0, 0.0, 0.0)  # 180 deg about x
    x, y, z, w = -dy, dx, 0.0, 1.0 + dz  # cross(z, d), 1 + dot(z, d)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


Y_TO_Z: Quat = axis_angle_quat((1.0, 0.0, 0.0), 0.5 * math.pi)
"""Rotates a mesh built around the y axis (e.g. by :func:`~wheelmo_sim.meshes.revolve`) to stand along z."""


def set_pose(prim, pos: Vec3, quat: Quat = IDENTITY, scale: Vec3 | None = None) -> None:
    Gf, _, UsdGeom, _, _ = pxr()
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*pos))
    x, y, z, w = quat
    xform.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
    if scale is not None:
        xform.AddScaleOp().Set(Gf.Vec3f(*scale))


def _style(prim, color: Vec3, collider: bool, proxy: bool) -> None:
    Gf, _, UsdGeom, UsdPhysics, _ = pxr()
    UsdGeom.Gprim(prim).CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collider or proxy:
        UsdPhysics.CollisionAPI.Apply(prim)
    if proxy:
        UsdGeom.Imageable(prim).CreatePurposeAttr(UsdGeom.Tokens.guide)


def group(stage, path: str):
    _, _, UsdGeom, _, _ = pxr()
    return UsdGeom.Xform.Define(stage, path).GetPrim()


def box(stage, path: str, center: Vec3, size: Vec3, color: Vec3, *, collider=False, proxy=False, quat: Quat = IDENTITY):
    _, _, UsdGeom, _, _ = pxr()
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    set_pose(cube.GetPrim(), center, quat, scale=size)
    _style(cube.GetPrim(), color, collider, proxy)
    return cube.GetPrim()


def cylinder(stage, path: str, center: Vec3, radius: float, height: float, axis: str, color: Vec3, *, collider=False, proxy=False):
    _, _, UsdGeom, _, _ = pxr()
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr(axis)
    set_pose(cyl.GetPrim(), center)
    _style(cyl.GetPrim(), color, collider, proxy)
    return cyl.GetPrim()


def sphere(stage, path: str, center: Vec3, radius: float, color: Vec3):
    _, _, UsdGeom, _, _ = pxr()
    sph = UsdGeom.Sphere.Define(stage, path)
    sph.CreateRadiusAttr(radius)
    set_pose(sph.GetPrim(), center)
    _style(sph.GetPrim(), color, False, False)
    return sph.GetPrim()


def tube(stage, path: str, p0: Vec3, p1: Vec3, radius: float, color: Vec3):
    """Visual capsule from ``p0`` to ``p1`` (frame tubes, handles)."""
    _, _, UsdGeom, _, _ = pxr()
    d = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    length = math.sqrt(sum(c * c for c in d))
    cap = UsdGeom.Capsule.Define(stage, path)
    cap.CreateRadiusAttr(radius)
    cap.CreateHeightAttr(length)
    cap.CreateAxisAttr("Z")
    set_pose(cap.GetPrim(), tuple(0.5 * (a + b) for a, b in zip(p0, p1, strict=True)), quat_z_to(d))
    _style(cap.GetPrim(), color, False, False)
    return cap.GetPrim()


def mesh(stage, path: str, tri: TriMesh, color: Vec3, pos: Vec3 = (0.0, 0.0, 0.0), quat: Quat = IDENTITY):
    """Visual triangle mesh with smooth vertex normals."""
    _, _, UsdGeom, _, Vt = pxr()
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(tri.points))
    m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(tri.triangles), 3, dtype=np.int32)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(tri.triangles.reshape(-1)))
    m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(tri.normals))
    m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(np.stack([tri.points.min(0), tri.points.max(0)])))
    set_pose(m.GetPrim(), pos, quat)
    _style(m.GetPrim(), color, False, False)
    return m.GetPrim()


def rigid_body(stage, path: str, pos: Vec3, mass: float, com: Vec3, inertia: Vec3):
    Gf, _, UsdGeom, UsdPhysics, _ = pxr()
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    set_pose(prim, pos)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(float(mass))
    mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*com))
    mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
    return prim


def revolute(stage, path: str, body0: str, body1: str, axis: str, pos0: Vec3, pos1: Vec3 = (0.0, 0.0, 0.0)):
    Gf, Sdf, _, UsdPhysics, _ = pxr()
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    joint.CreateAxisAttr(axis)
    joint.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(*pos1))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    # Zero-gain drive; the Isaac Lab actuator config decides velocity/passive behaviour.
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0)
    return joint.GetPrim()
