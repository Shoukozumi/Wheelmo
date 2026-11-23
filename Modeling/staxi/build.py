"""Build everything: STEP, per-link meshes, URDF, and a scan-pose overlay mesh.

Outputs (in out/):
    staxi_wheelchair.step         full assembly, brake squeezed, mm
    staxi_wheelchair.urdf         articulated model for PyBullet / Isaac Lab (metres)
    meshes/*.stl                  visual meshes per link, link frame, metres
    assembly_mm.stl               assembly as one mesh (mm), for viewing
    overlay_scan_pose_mm.stl      assembly posed like the scan (brake released, casters swivelled)
"""
import math
from pathlib import Path
from xml.sax.saxutils import quoteattr

import numpy as np
import trimesh
from build123d import Compound, export_step, export_stl
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

import model
import params as P

OUT = Path(__file__).resolve().parent / "out"
MESH_DIR = OUT / "meshes"
SCAN_POSE = {"caster_fork_l": math.radians(-50), "caster_fork_r": math.radians(-69)}  # [scan] fitted wheel planes -49.9 / -69.1 deg
# Rear wheels: turn the left cog to the scan's tooth phase (gap centres at 7.7 + k*18 deg, where its pawl
# rests on a tooth); the right pawl is in a gap in the scan, which is the joint-zero phase.
SCAN_POSE["rear_wheel_l"] = math.radians((model.pawl_angle() - 7.70 + 9) % 18 - 9)
SCAN_POSE["rear_wheel_r"] = 0.0
MATERIALS = {"metal": (0.75, 0.76, 0.78, 1), "cushion": (0.12, 0.14, 0.18, 1),
             "tire": (0.15, 0.15, 0.15, 1), "pad": (0.1, 0.1, 0.1, 1),
             "stripe_white": (0.93, 0.93, 0.93, 1), "stripe_black": (0.05, 0.05, 0.05, 1),
             "plastic": (0.22, 0.22, 0.24, 1)}


def mass_props(shape, mass, kind="solid"):
    """(com_mm, inertia_kg_mm2 about com) for a shape scaled to `mass` kg. kind="shell" spreads the mass
    over the surface instead of the volume (thin-walled tubes: a tube's mass goes with its perimeter)."""
    g = GProp_GProps()
    (BRepGProp.SurfaceProperties_s if kind == "shell" else BRepGProp.VolumeProperties_s)(shape.wrapped, g)
    vol = g.Mass()
    c = g.CentreOfMass()
    M = g.MatrixOfInertia()
    I = np.array([[M.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]) * (mass / vol)
    return np.array([c.X(), c.Y(), c.Z()]), I


def combine(parts):
    """Combine [(mass, com, I_about_com)] with the parallel-axis theorem."""
    m = sum(p[0] for p in parts)
    com = sum(p[0] * p[1] for p in parts) / m
    I = np.zeros((3, 3))
    for mi, ci, Ii in parts:
        d = ci - com
        I += Ii + mi * (d @ d * np.eye(3) - np.outer(d, d))
    return m, com, I


def rpy_from_z_to(d):
    """URDF roll-pitch-yaw that rotates the local Z axis onto unit vector d."""
    d = d / np.linalg.norm(d)
    z = np.array([0.0, 0.0, 1.0])
    v, c = np.cross(z, d), float(z @ d)
    if np.linalg.norm(v) < 1e-9:
        R = np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    else:
        k = v / np.linalg.norm(v)
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        a = math.acos(max(-1.0, min(1.0, c)))
        R = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def f3(v, scale=1.0):
    return " ".join(f"{x * scale:.6g}" for x in v)


def export_mesh_m(shape, path):
    """Tessellate a build123d shape (mm) and save it as an STL in metres."""
    tmp = path.with_suffix(".mm.stl")
    export_stl(shape, str(tmp), tolerance=0.25, angular_tolerance=0.25)
    m = trimesh.load(tmp, force="mesh")
    m.apply_scale(0.001)
    m.export(path)
    tmp.unlink()


def collision_xml(c):
    if c["type"] == "cylinder":
        a, b = np.array(c["a"]), np.array(c["b"])
        L = np.linalg.norm(b - a)
        return (f'<collision><origin xyz="{f3((a + b) / 2, 1e-3)}" rpy="{f3(rpy_from_z_to(b - a))}"/>'
                f'<geometry><cylinder radius="{c["r"] * 1e-3:.6g}" length="{L * 1e-3:.6g}"/></geometry></collision>')
    return (f'<collision><origin xyz="{f3(c["center"], 1e-3)}" rpy="{f3(c["rpy"])}"/>'
            f'<geometry><box size="{f3(c["size"], 1e-3)}"/></geometry></collision>')


def visual_xml(mesh_rel, material):
    return (f'<visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename={quoteattr(mesh_rel)}/></geometry>'
            f'<material name="{material}"/></visual>')


def write_urdf(links, path):
    lines = ['<?xml version="1.0"?>', '<robot name="staxi_wheelchair">']
    for name, rgba in MATERIALS.items():
        lines.append(f'  <material name="{name}"><color rgba="{f3(rgba)}"/></material>')
    for name, L in links.items():
        parts = L.get("mass_parts") or [(L["shape"], L["mass"])]
        m, com, I = combine([(p[1], *mass_props(*p)) for p in parts])
        I = I * 1e-6  # kg mm^2 -> kg m^2
        lines.append(f'  <link name="{name}">')
        lines.append(f'    <inertial><origin xyz="{f3(com, 1e-3)}" rpy="0 0 0"/><mass value="{m:.6g}"/>'
                     f'<inertia ixx="{I[0,0]:.6g}" ixy="{I[0,1]:.6g}" ixz="{I[0,2]:.6g}" '
                     f'iyy="{I[1,1]:.6g}" iyz="{I[1,2]:.6g}" izz="{I[2,2]:.6g}"/></inertial>')
        for mesh_rel, mat in L["visuals"]:
            lines.append("    " + visual_xml(mesh_rel, mat))
        for c in L["collision"]:
            lines.append("    " + collision_xml(c))
        lines.append("  </link>")
    for name, L in links.items():
        if L["parent"] is None:
            continue
        j = L["joint"]
        lines.append(f'  <joint name="{name}_joint" type="{j["type"]}">')
        lines.append(f'    <parent link="{L["parent"]}"/><child link="{name}"/>')
        lines.append(f'    <origin xyz="{f3(j["origin"], 1e-3)}" rpy="0 0 0"/><axis xyz="{f3(j["axis"])}"/>')
        if j["type"] == "revolute":
            lines.append(f'    <limit lower="{j["lower"]:.6g}" upper="{j["upper"]:.6g}" '
                         f'effort="{j["effort"]:.6g}" velocity="{j["velocity"]:.6g}"/>')
        lines.append(f'    <dynamics damping="{j["damping"]:.6g}" friction="{j["friction"]:.6g}"/>')
        lines.append("  </joint>")
    lines.append("</robot>")
    path.write_text("\n".join(lines) + "\n")


def main():
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    links = model.build(brake_squeezed=True)

    # visual meshes per link (base link split by material)
    for name, L in links.items():
        L["visuals"] = []
        if name == "base_link":
            metal, cushion = (p[0] for p in L["mass_parts"])
            groups = [("base_link_metal", metal, "metal"), ("base_link_cushion", cushion, "cushion")]
        elif "paint" in L:  # one mesh per colour, e.g. a wheel's black and white stripes
            by_mat = {}
            for shape, mat in L["paint"]:
                by_mat.setdefault(mat, []).append(shape)
            groups = [(f"{name}_{mat}", Compound(shapes), mat) for mat, shapes in by_mat.items()]
        else:
            mat = "tire" if "wheel" in name else ("pad" if "armrest" in name else "metal")
            groups = [(name, L["shape"], mat)]
        for fname, shape, mat in groups:
            export_mesh_m(shape, MESH_DIR / f"{fname}.stl")
            L["visuals"].append((f"meshes/{fname}.stl", mat))

    write_urdf(links, OUT / "staxi_wheelchair.urdf")
    asm = model.assembly(links)
    export_step(asm, str(OUT / "staxi_wheelchair.step"))
    export_stl(asm, str(OUT / "assembly_mm.stl"), tolerance=0.25, angular_tolerance=0.25)

    scan_links = model.build(brake_squeezed=False)
    export_stl(model.assembly(scan_links, SCAN_POSE), str(OUT / "overlay_scan_pose_mm.stl"),
               tolerance=0.5, angular_tolerance=0.3)
    print("wrote", *sorted(p.name for p in OUT.iterdir()))


if __name__ == "__main__":
    main()
