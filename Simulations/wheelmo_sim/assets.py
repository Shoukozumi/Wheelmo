"""Isaac Lab asset configs: the procedural wheelchair articulation and the corridor.

Both are authored directly as USD Physics prims (no URDF/CAD import), following the same pattern as
Isaac Lab's procedural keyboard asset. The wheelchair's geometry lives in :mod:`wheelmo_sim.wheelchair_model`.
When a CAD model or URDF of the real chair exists, swap :func:`wheelchair_cfg`'s ``spawn`` for
``sim_utils.UsdFileCfg`` / ``sim_utils.UrdfFileCfg`` and keep the joint names (``left_drive_joint`` etc.) so the
controller and sensor mounts keep working.

Articulation layout (all links are siblings under the robot prim)::

    Robot (ArticulationRootAPI, self-collision off)
    |- base_link                                   frame, seat, backrest, legrests, sensor mast
    |- left_wheel / right_wheel                    <- left/right_drive_joint (revolute y, velocity-driven)
    |- left_caster_fork / right_caster_fork        <- left/right_caster_swivel_joint (revolute z, passive)
    |- left_caster_wheel / right_caster_wheel      <- left/right_caster_wheel_joint (revolute y, passive)
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import field
from typing import Any

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
from isaaclab.sim.utils import clone, get_current_stage
from isaaclab.utils import configclass

from . import usd_utils as U
from .corridor_layout import Box, CorridorSpec, Cylinder, generate_corridor_layout
from .wheelchair_model import author_wheelchair
from .wheelchair_spec import WheelchairSpec

# ------------------------------------------------------------------------------------------------------
# Wheelchair
# ------------------------------------------------------------------------------------------------------


@clone
def spawn_wheelchair(prim_path: str, cfg: WheelchairSpawnerCfg, translation=None, orientation=None, **kwargs) -> Any:
    """Author the wheelchair articulation at ``prim_path`` (cloned to every environment by ``@clone``)."""
    _, Sdf, UsdGeom, UsdPhysics, _ = U.pxr()
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path).GetPrim()
    U.set_pose(root, translation or (0.0, 0.0, 0.0), orientation or U.IDENTITY)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    for schema, attr in (("PhysxArticulationAPI", "physxArticulation:enabledSelfCollisions"),
                         ("NewtonArticulationRootAPI", "newton:selfCollisionEnabled")):
        root.AddAppliedSchema(schema)
        root.CreateAttribute(attr, Sdf.ValueTypeNames.Bool).Set(False)
    author_wheelchair(stage, prim_path, cfg.spec)
    return root


@configclass
class WheelchairSpawnerCfg(SpawnerCfg):
    """Spawns the procedural wheelchair described by :attr:`spec`."""

    func: Callable = spawn_wheelchair
    spec: WheelchairSpec = field(default_factory=WheelchairSpec)


def wheelchair_cfg(spec: WheelchairSpec | None = None, prim_path: str = "{ENV_REGEX_NS}/Robot") -> ArticulationCfg:
    """Articulation config for the wheelchair, spawned at the corridor start."""
    spec = spec or WheelchairSpec()
    return ArticulationCfg(
        prim_path=prim_path,
        spawn=WheelchairSpawnerCfg(spec=spec),
        init_state=ArticulationCfg.InitialStateCfg(pos=(1.0, 0.0, spec.drive_wheel_radius + 0.002)),
        actuators={
            # Damping-only implicit actuators run in velocity mode: torque = damping * (w_target - w), clipped
            # to the torque limit. The gain must be high enough to scrub the casters around when turning
            # in place, like a power chair's motor controller.
            "drive": ImplicitActuatorCfg(
                joint_names_expr=[".*_drive_joint"],
                stiffness=0.0,
                damping=200.0,
                armature=0.05,
                joint_effort_limit=spec.drive_torque_limit,
                actuator_velocity_limit=spec.drive_speed_limit,
            ),
            # Passive caster joints: zero target velocity + light damping = bearing friction.
            "caster_swivel": ImplicitActuatorCfg(
                joint_names_expr=[".*_caster_swivel_joint"], stiffness=0.0, damping=0.05, armature=0.01
            ),
            "caster_roll": ImplicitActuatorCfg(
                joint_names_expr=[".*_caster_wheel_joint"], stiffness=0.0, damping=0.005, armature=0.005
            ),
        },
    )


# ------------------------------------------------------------------------------------------------------
# Corridor
# ------------------------------------------------------------------------------------------------------


@clone
def spawn_corridor(prim_path: str, cfg: CorridorSpawnerCfg, translation=None, orientation=None, **kwargs) -> Any:
    """Author the corridor floor, walls, doorways and clutter as static colliders under ``prim_path``.

    These shapes are both visual and collision geometry: static bodies with no separate visuals are drawn
    as-is by every renderer.
    """
    stage = get_current_stage()
    root = U.group(stage, prim_path)
    U.set_pose(root, translation or (0.0, 0.0, 0.0), orientation or U.IDENTITY)
    layout = generate_corridor_layout(cfg.spec)
    for shape in (layout.floor, *layout.shapes):
        path = f"{prim_path}/{shape.name}"
        if isinstance(shape, Box):
            U.box(stage, path, shape.center, shape.size, shape.color, collider=True)
        elif isinstance(shape, Cylinder):
            U.cylinder(stage, path, shape.center, shape.radius, shape.height, "Z", shape.color, collider=True)
    return root


@configclass
class CorridorSpawnerCfg(SpawnerCfg):
    """Spawns the static corridor described by :attr:`spec` (same layout in every environment)."""

    func: Callable = spawn_corridor
    spec: CorridorSpec = field(default_factory=CorridorSpec)
