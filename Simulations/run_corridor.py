"""Drive the simulated wheelchair down a tightly packed corridor using its LiDAR.

Examples (run from this directory)::

    uv run python run_corridor.py --max_steps 4000                  # headless smoke test
    uv run python run_corridor.py --viz newton_gl                   # with a viewer window
    uv run python run_corridor.py --save_every 25 --max_steps 4000  # also dump camera + LiDAR images to outputs/
    uv run python run_corridor.py --num_envs 4 --seed 3             # 4 chairs in parallel, another layout
"""

from __future__ import annotations

import argparse
import math
import os
import platform
import time
from pathlib import Path

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description="Wheelchair corridor navigation in Isaac Lab (Newton backend).")
parser.add_argument("--num_envs", type=int, default=1, help="Parallel environments (each renders 2x 1080p cameras).")
parser.add_argument("--seed", type=int, default=0, help="Corridor layout seed.")
parser.add_argument("--max_steps", type=int, default=-1, help="Physics steps to run; -1 runs until the window closes.")
parser.add_argument("--episode_seconds", type=float, default=90.0, help="Episode timeout in simulated seconds.")
parser.add_argument("--controller", choices=("planner", "constant"), default="planner", help="LiDAR local planner or fixed command.")
parser.add_argument("--cmd", type=float, nargs=2, default=(0.3, 0.0), metavar=("V", "W"), help="Fixed (m/s, rad/s) command.")
parser.add_argument("--save_every", type=int, default=0, help="Save sensor images every N control steps (0 = off).")
parser.add_argument("--output_dir", type=Path, default=Path("outputs"), help="Where --save_every writes images.")
parser.add_argument("--render_hz", type=float, default=30.0, help="Viewer refresh rate (only with --viz).")
add_launcher_args(parser)
args_cli = parser.parse_args()

# OpenGL viewer under WSL2: WSLg's Mesa D3D12 driver defaults to the first adapter, which on hybrid-graphics
# laptops is the integrated GPU. Prefer the NVIDIA GPU unless the user already picked an adapter.
if "microsoft" in platform.release().lower():
    os.environ.setdefault("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA")
if "newton_gl" in str(args_cli.visualizer) and os.environ.get("LIBGL_ALWAYS_SOFTWARE", "0").lower() not in ("0", "false", "no", ""):
    print("[WARN] LIBGL_ALWAYS_SOFTWARE is set, so the viewer window is rasterised on the CPU (llvmpipe), not the GPU. "
          "Run with LIBGL_ALWAYS_SOFTWARE=0 to use the GPU for the viewer.")

import torch  # noqa: E402
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.scene import InteractiveScene  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_apply_inverse  # noqa: E402

from wheelmo_sim.controller import LocalPlanner, LocalPlannerCfg, ObstacleMemory  # noqa: E402
from wheelmo_sim.corridor_layout import CorridorSpec, generate_corridor_layout  # noqa: E402
from wheelmo_sim.scene import WheelchairCorridorSceneCfg, make_scene_cfg  # noqa: E402
from wheelmo_sim.sensor_io import save_bev, save_rgb  # noqa: E402
from wheelmo_sim.wheelchair_spec import WheelchairSpec  # noqa: E402

PHYSICS_DT = 1.0 / 200.0
CONTROL_DECIMATION = 4  # control at 50 Hz
# Physics steps per viewer redraw. ``sim.step()`` redraws on every call unless told otherwise, and each redraw
# (copy to host + window present) costs far more than a physics step, which leaves the GPU idle waiting on it.
# Camera sensors are unaffected: they render on their own ``update_period``.
RENDER_DECIMATION = max(1, round(1.0 / (PHYSICS_DT * args_cli.render_hz)))


@configclass
class LaunchCfg:
    """Everything ``launch_simulation`` should scan for backend/renderer requirements."""

    sim: sim_utils.SimulationCfg = None
    scene: WheelchairCorridorSceneCfg = None


def main() -> None:
    chair = WheelchairSpec()
    corridor = CorridorSpec(seed=args_cli.seed)
    layout = generate_corridor_layout(corridor)
    scene_cfg = make_scene_cfg(args_cli.num_envs, chair, corridor)
    sim_cfg = sim_utils.SimulationCfg(
        dt=PHYSICS_DT,
        render_interval=RENDER_DECIMATION,
        device=args_cli.device,
        physics=NewtonCfg(solver_cfg=MJWarpSolverCfg(integrator="implicitfast")),
    )
    with launch_simulation(LaunchCfg(sim=sim_cfg, scene=scene_cfg), args_cli):
        sim = sim_utils.SimulationContext(sim_cfg)
        scene = InteractiveScene(scene_cfg)
        # set after the scene exists so env 0's origin is known; applied when the viewer starts in sim.reset()
        sim.set_camera_view(*birdseye_view(corridor, scene.env_origins[0].tolist()))
        sim.reset()
        print(f"[INFO] Corridor seed {corridor.seed}: {len(layout.obstacles)} obstacles, "
              f"{corridor.width:.2f} m wide, doorways at x={list(corridor.doorway_x)}")
        run(sim, scene, chair, layout)


VIEWER_VFOV_DEG = 65.0  # Newton GL viewer default: 12 mm focal length on USD's 15.29 mm vertical aperture
VIEWER_ASPECT = 16.0 / 9.0  # default 1920x1080 window


def birdseye_view(corridor: CorridorSpec, origin: list[float], margin: float = 0.5):
    """Viewer (eye, target) looking down on the whole corridor of the environment at ``origin``.

    The corridor runs along +x, so it should fill the window's wide (horizontal) axis. The viewer orients itself
    by yaw/pitch and clamps pitch at 89 deg, so the camera tilts 1 deg off vertical, with the eye slightly on the
    -y side of the target. That makes +y screen-up and +x screen-right: the corridor runs left to right, start on
    the left. The height is the lowest at which the tops of the end walls (the closest points to the camera)
    still fit the window width.
    """
    half_length = 0.5 * corridor.length + corridor.wall_thickness + margin
    tan_half_hfov = math.tan(math.radians(0.5 * VIEWER_VFOV_DEG)) * VIEWER_ASPECT
    height = corridor.wall_height + half_length / tan_half_hfov
    target = (origin[0] + 0.5 * corridor.length, origin[1], origin[2])
    eye = (target[0], target[1] - height * math.tan(math.radians(1.0)), target[2] + height)
    return eye, target


def planar_to_base(offset_xy: torch.Tensor, root_quat: torch.Tensor) -> torch.Tensor:
    """Rotate world-frame planar offsets (N, ..., 2) into each robot's base_link frame."""
    quat = root_quat.view(root_quat.shape[0], *([1] * (offset_xy.dim() - 2)), 4)
    return quat_apply_inverse(quat, torch.cat([offset_xy, torch.zeros_like(offset_xy[..., :1])], -1))[..., :2]


def run(sim: sim_utils.SimulationContext, scene: InteractiveScene, chair: WheelchairSpec, layout) -> None:
    robot, lidar = scene["robot"], scene["lidar"]
    cameras = {"front": scene["front_camera"], "rear": scene["rear_camera"]}
    drive_ids, _ = robot.find_joints(["left_drive_joint", "right_drive_joint"], preserve_order=True)
    num_envs, device = scene.num_envs, sim.device
    all_envs = torch.arange(num_envs, device=device)
    planner_cfg = LocalPlannerCfg()
    planner = LocalPlanner(planner_cfg, chair, num_envs, PHYSICS_DT * CONTROL_DECIMATION, device)
    corridor_axis = torch.tensor([[1.0, 0.0]], device=device).expand(num_envs, 2)  # travel direction (env frame)
    half_w = 0.5 * layout.spec.width + 0.3
    memory = ObstacleMemory(num_envs, (-0.5, layout.spec.length + 0.5), (-half_w, half_w), 0.05, device)
    goal_x = layout.goal_xy[0]
    episode_steps = int(args_cli.episode_seconds / (PHYSICS_DT * CONTROL_DECIMATION))
    if args_cli.save_every:
        args_cli.output_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Joints: {robot.joint_names}")
    print(f"[INFO] Bodies: {robot.body_names}")
    print(f"[INFO] Body masses [kg]: {[round(m, 2) for m in robot.data.body_mass.torch[0].tolist()]}")
    print(f"[INFO] LiDAR rays per scan: {lidar.data.ray_hits_w.torch.shape[1]}")

    def reset(env_ids: torch.Tensor) -> None:
        pose = robot.data.default_root_pose.torch[env_ids].clone()
        pose[:, :3] += scene.env_origins[env_ids]
        robot.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        robot.write_root_velocity_to_sim_index(root_velocity=robot.data.default_root_vel.torch[env_ids].clone(), env_ids=env_ids)
        robot.write_joint_position_to_sim_index(position=robot.data.default_joint_pos.torch[env_ids].clone(), env_ids=env_ids)
        robot.write_joint_velocity_to_sim_index(velocity=robot.data.default_joint_vel.torch[env_ids].clone(), env_ids=env_ids)
        scene.reset(env_ids)
        memory.reset(env_ids)
        planner.reset(env_ids)

    reset(all_envs)
    episode_step = torch.zeros(num_envs, dtype=torch.long, device=device)
    wheel_targets = torch.zeros(num_envs, 2, device=device)
    linear = angular = ttc = torch.zeros(num_envs, device=device)
    step = control_step = 0
    wall_start = time.perf_counter()

    while sim.is_running() and (args_cli.max_steps < 0 or step < args_cli.max_steps):
        if step % CONTROL_DECIMATION == 0:
            root_pos, root_quat = robot.data.root_pos_w.torch, robot.data.root_quat_w.torch
            pos_env = root_pos - scene.env_origins

            # ---- episode bookkeeping ------------------------------------------------------------------------
            up_z = quat_apply(root_quat, torch.tensor([0.0, 0.0, 1.0], device=device).expand(num_envs, 3))[:, 2]
            reached, tipped, timeout = pos_env[:, 0] > goal_x, up_z < 0.7, episode_step >= episode_steps
            done = reached | tipped | timeout
            if done.any():
                for i in done.nonzero().flatten().tolist():
                    reason = "reached goal" if reached[i] else ("tipped over" if tipped[i] else "timed out")
                    t = episode_step[i].item() * PHYSICS_DT * CONTROL_DECIMATION
                    print(f"[EPISODE] env {i}: {reason} after {t:.1f} s at x = {pos_env[i, 0]:.2f} m")
                done_ids = done.nonzero().flatten()
                reset(done_ids)
                episode_step[done_ids] = 0
                root_pos, root_quat = robot.data.root_pos_w.torch, robot.data.root_quat_w.torch
                pos_env = root_pos - scene.env_origins

            # ---- sensors -> command -----------------------------------------------------------------------
            hits_w = lidar.data.ray_hits_w.torch  # (N, rays, 3), inf where a ray hit nothing
            points_b = quat_apply_inverse(root_quat.unsqueeze(1), hits_w - root_pos.unsqueeze(1))
            if args_cli.controller == "planner":
                # remember returns between floor and head height (env frame: floor at z = 0)
                hits_env = hits_w - scene.env_origins.unsqueeze(1)
                seen = torch.isfinite(hits_env).all(-1) & (hits_env[..., 2] > 0.04) & (hits_env[..., 2] < 2.0)
                memory.insert(hits_env[..., :2], seen)
                cells, cell_valid = memory.nearest(pos_env[:, :2], planner_cfg.max_points)
                obstacles_b = planar_to_base(cells - pos_env[:, None, :2], root_quat)
                direction_b = torch.nn.functional.normalize(planar_to_base(corridor_axis, root_quat), dim=-1)
                linear, angular, ttc = planner(obstacles_b, cell_valid, direction_b)
            else:
                linear = torch.full((num_envs,), args_cli.cmd[0], device=device)
                angular = torch.full((num_envs,), args_cli.cmd[1], device=device)
            left, right = chair.wheel_speeds(linear, angular)
            wheel_targets = torch.stack([left, right], dim=-1).clamp(-chair.drive_speed_limit, chair.drive_speed_limit)
            robot.actuators.target_command.set_velocity_index(value=wheel_targets, joint_ids=drive_ids)

            # ---- logging / saving -------------------------------------------------------------------------
            if control_step % 100 == 0:
                sim_t = step * PHYSICS_DT
                rtf = sim_t / max(time.perf_counter() - wall_start, 1e-6)  # real-time factor
                speed = robot.data.root_lin_vel_b.torch[0, 0].item()
                yaw = math.degrees(2 * math.atan2(root_quat[0, 2].item(), root_quat[0, 3].item()))  # chair stays level
                others = f" | x all envs: {[round(x, 1) for x in pos_env[:8, 0].tolist()]}" if num_envs > 1 else ""
                print(f"[t={sim_t:6.1f}s rtf={rtf:4.2f}] env0 x={pos_env[0, 0]:5.2f} y={pos_env[0, 1]:+.2f} yaw={yaw:+4.0f} "
                      f"v_cmd={linear[0]:.2f} v={speed:.2f} w_cmd={angular[0]:+.2f} ttc={ttc[0]:.1f}s{others}")
            if args_cli.save_every and control_step % args_cli.save_every == 0:
                tag = f"{control_step:06d}"
                for name, cam in cameras.items():
                    save_rgb(cam.data.output["rgb"], args_cli.output_dir / f"{name}_{tag}.png")
                save_bev(points_b[0], chair, args_cli.output_dir / f"lidar_{tag}.png")
            episode_step += 1
            control_step += 1

        scene.write_data_to_sim()
        sim.step(render=step % RENDER_DECIMATION == 0)
        scene.update(PHYSICS_DT)
        step += 1


if __name__ == "__main__":
    main()
