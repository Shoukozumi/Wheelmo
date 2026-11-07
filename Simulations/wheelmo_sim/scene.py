"""Scene: corridor + wheelchair with one 360 deg LiDAR and two 1080p RGB cameras.

Everything runs on Isaac Lab's kit-less stack: Newton (MuJoCo-Warp) physics, Newton's BVH ray caster for the
LiDAR and the Newton Warp ray-traced renderer for the cameras. Isaac Sim is not required, and every asset is
generated procedurally, so nothing is downloaded at runtime.
"""

from __future__ import annotations

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.sensors.ray_caster.patterns import LidarPatternCfg
from isaaclab.utils import configclass
from isaaclab_newton.renderers import NewtonWarpRendererCfg
from isaaclab_newton.sensors import NewtonRaycastSensorCfg

from .assets import CorridorSpawnerCfg, wheelchair_cfg
from .corridor_layout import CorridorSpec
from .wheelchair_spec import WheelchairSpec, yaw_pitch_quat_xyzw

CAMERA_WIDTH, CAMERA_HEIGHT = 1920, 1080


def lidar_cfg(spec: WheelchairSpec) -> NewtonRaycastSensorCfg:
    """360 deg spinning LiDAR on top of the mast (Ouster OS0-32-like: 32 beams, 90 deg vertical FOV, 10 Hz).

    For a single-plane 2D LiDAR use ``channels=1, vertical_fov_range=(0.0, 0.0)``.
    """
    return NewtonRaycastSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link",
        offset=NewtonRaycastSensorCfg.OffsetCfg(pos=spec.lidar_pos),
        pattern_cfg=LidarPatternCfg(
            channels=32,
            vertical_fov_range=(-45.0, 45.0),
            horizontal_fov_range=(-180.0, 180.0),
            horizontal_res=0.35,
        ),
        ray_alignment="base",
        max_distance=30.0,
        update_period=0.1,
    )


def rgb_camera_cfg(name: str, pos: tuple[float, float, float], yaw_deg: float, pitch_deg: float) -> CameraCfg:
    """1920x1080 RGB pinhole camera with a ~90 deg horizontal FOV, rendered by Newton's Warp ray tracer at 30 Hz."""
    return CameraCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/base_link/{name}",
        # "world" convention: the camera looks along +x of the offset frame, with +z up.
        offset=CameraCfg.OffsetCfg(pos=pos, rot=yaw_pitch_quat_xyzw(yaw_deg, pitch_deg), convention="world"),
        update_period=1.0 / 30.0,
        # The camera rides on a moving body; without this it keeps rendering from its spawn pose.
        update_latest_camera_pose=True,
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=10.5, horizontal_aperture=20.955, clipping_range=(0.05, 50.0)),
        renderer_cfg=NewtonWarpRendererCfg(enable_shadows=True),
    )


@configclass
class WheelchairCorridorSceneCfg(InteractiveSceneCfg):
    """One corridor (with its own floor) + wheelchair per environment. Build with :func:`make_scene_cfg`."""

    corridor: AssetBaseCfg = MISSING
    robot: ArticulationCfg = MISSING
    lidar: NewtonRaycastSensorCfg = MISSING
    front_camera: CameraCfg = MISSING
    rear_camera: CameraCfg = MISSING


def make_scene_cfg(
    num_envs: int = 1,
    wheelchair: WheelchairSpec | None = None,
    corridor: CorridorSpec | None = None,
) -> WheelchairCorridorSceneCfg:
    wheelchair, corridor = wheelchair or WheelchairSpec(), corridor or CorridorSpec()
    return WheelchairCorridorSceneCfg(
        num_envs=num_envs,
        # Corridors run along +x; space environments so neighbouring corridors don't overlap visually.
        env_spacing=corridor.length + 4.0,
        corridor=AssetBaseCfg(prim_path="{ENV_REGEX_NS}/Corridor", spawn=CorridorSpawnerCfg(spec=corridor)),
        robot=wheelchair_cfg(wheelchair),
        lidar=lidar_cfg(wheelchair),
        front_camera=rgb_camera_cfg("front_camera", wheelchair.front_camera_pos, 0.0, wheelchair.front_camera_pitch_deg),
        rear_camera=rgb_camera_cfg("rear_camera", wheelchair.rear_camera_pos, 180.0, wheelchair.rear_camera_pitch_deg),
    )
