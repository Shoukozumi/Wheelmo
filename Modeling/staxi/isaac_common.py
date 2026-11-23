"""Shared Isaac Lab setup for the Staxi scenes (isaac_hallway.py, isaac_stack.py): the chair's articulation
config and the rubber tyres. Import it after the launcher's argument parsing, like the other Isaac Lab
modules."""

import re
from pathlib import Path

from pxr import Usd, UsdPhysics
from isaaclab_newton.sim.schemas import NewtonArticulationCfg
from isaaclab_physx.sim.schemas import PhysxArticulationCfg

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

HERE = Path(__file__).resolve().parent
URDF = HERE / "out" / "staxi_wheelchair.urdf"
USD = HERE / "out" / "usd" / "staxi_wheelchair" / "staxi_wheelchair.usda"
CONVERT = f'bash "{HERE / "make_usd.sh"}"'
HANDLE = (-0.267, 0.0, 0.998)  # push-handle crossbar in the chair frame [m]

# the tyres are rubber: high grip on the floor
RUBBER = sim_utils.RigidBodyMaterialCfg(static_friction=1.2, dynamic_friction=1.0, restitution=0.0)
WHEEL_LINK = re.compile(r"(rear|caster|anti_tip)_wheel_[lr]$")


def check_usd():
    """Stop with a hint if the USD is missing or older than the URDF."""
    if not USD.exists() or USD.stat().st_mtime < URDF.stat().st_mtime:
        raise SystemExit(f"The wheelchair USD is missing or older than the URDF. Run:\n  {CONVERT}")


def spawn_chair(prim_path, cfg, *args, **kwargs):
    """Spawn the chair's USD, then bind the rubber material to every wheel's collider (the URDF has no
    friction, and the USD spawner can't target child prims)."""
    prim = sim_utils.spawn_from_usd(prim_path, cfg, *args, **kwargs)
    material = f"{prim.GetPath()}/PhysicsMaterials/rubber"
    RUBBER.func(material, RUBBER)
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.CollisionAPI) and WHEEL_LINK.search(p.GetParent().GetName()):
            sim_utils.bind_physics_material(str(p.GetPath()), material)
    return prim


STAXI_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Wheelchair",
    # USD converted from the URDF with Isaac Lab's convert_urdf.py (free base, joint drives off)
    spawn=sim_utils.UsdFileCfg(
        func=spawn_chair,
        usd_path=str(USD),
        articulation_props=[
            PhysxArticulationCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=1,
                sleep_threshold=0.0,  # never sleep: a sleeping chair ignores the push
            ),
            NewtonArticulationCfg(self_collision_enabled=False),
        ],
    ),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.005), joint_pos={".*": 0.0}),
    # passive joints: no stiffness, only damping; the wheel bearings have very little friction
    actuators={
        "wheels": ImplicitActuatorCfg(joint_names_expr=["rear_wheel_.*", "caster_wheel_.*"], stiffness=0.0, damping=1e-4),
        "casters": ImplicitActuatorCfg(joint_names_expr=["caster_fork_.*"], stiffness=0.0, damping=0.002),
        # tiny wheels: keep damping tiny too, or they stop between bounces and skid
        "anti_tip": ImplicitActuatorCfg(joint_names_expr=["anti_tip_wheel_.*"], stiffness=0.0, damping=1e-5),
        "armrests": ImplicitActuatorCfg(joint_names_expr=["armrest_.*"], stiffness=0.0, damping=0.5),
    },
)
