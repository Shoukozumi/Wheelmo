"""Roll the Staxi wheelchair down a hallway in Isaac Lab 3.0.

An "attendant" pushes forward on the push handle until the chair reaches walking pace, keeps it there
for the first half of the hallway, lets go so it coasts, then brakes near the end. The chair resets to
the start and it repeats. All of the chair's joints are passive (no motors).

With --interactive there is no attendant: the chair stands still until you drag it yourself
(Newton GL viewer: right-click and drag on any part).

usage (from any folder; run_hallway.sh runs it in the Isaac Lab checkout's environment):
    bash run_hallway.sh --viz newton_gl --interactive  # desktop window, drag it with right-click
    bash run_hallway.sh --viz viser                    # browser at http://localhost:8080 (no dragging)
    bash run_hallway.sh                                # headless, prints progress only
The Isaac Sim viewport (--viz kit) needs an NVIDIA Vulkan driver, which WSL2 doesn't have.
Add --physics isaacsim_physx to use PhysX instead of Newton (see the note at --physics).
"""

import argparse
import os
import sys
import time
from pathlib import Path

from isaaclab.app import add_launcher_args, launch_simulation

sys.path.insert(0, str(Path(__file__).resolve().parent))  # isaac_common.py

parser = argparse.ArgumentParser(description="Push the Staxi wheelchair down a hallway.")
parser.add_argument("--speed", type=float, default=1.2, help="walking pace the attendant pushes to [m/s]")
parser.add_argument("--follow", action="store_true",
                    help="keep the camera on the chair (it then overrides dragging the view with the mouse)")
parser.add_argument("--loops", type=int, default=0, help="stop after this many runs down the hallway (0 = forever)")
parser.add_argument("--fast", action="store_true", help="don't slow down to real time when a viewer is open")
parser.add_argument("--interactive", action="store_true",
                    help="no attendant: the chair stays put until you drag it (right-click in --viz newton_gl)")
# read by Isaac Lab's launcher. Default Newton: in this IsaacLab checkout (2026-10-14) the PhysX articulation
# crashes on startup (articulation_data.joint_acc passes `self.joint_vel` without `.warp`), tutorials included.
parser.add_argument("--physics", default="newton_mjwarp", choices=["newton_mjwarp", "isaacsim_physx", "physx", "ovphysx"],
                    help="physics engine (default: newton_mjwarp)")
add_launcher_args(parser)
args_cli = parser.parse_args()

# WSL: OpenGL defaults to the CPU renderer (llvmpipe). Mesa's D3D12 driver runs it on the GPU instead,
# which the Newton GL window needs to be smooth. Must be set before the window's GL context exists.
if "microsoft" in Path("/proc/version").read_text().lower():
    os.environ.setdefault("GALLIUM_DRIVER", "d3d12")
    os.environ.setdefault("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA")

import torch  # noqa: E402
from isaaclab_newton.physics import NewtonCfg  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import AssetBaseCfg  # noqa: E402
from isaaclab.scene import InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab.utils import instantiate  # noqa: E402

from isaac_common import HANDLE, STAXI_CFG, check_usd  # noqa: E402

# hallway: floor from x = -2 m to +18 m, 1.8 m between the walls (hospital corridor)
HALL_X0, HALL_X1, HALL_W, WALL_H = -2.0, 18.0, 1.8, 2.4
PUSH_UNTIL_X = 8.0   # attendant lets go here
BRAKE_FROM_X = 14.0  # ... and stops the chair from here on
MAX_PUSH = 40.0      # attendant's maximum push/pull [N]
PHYSICS_HZ, FRAME_HZ = 240, 60  # physics steps and attendant decisions per second (viewer: up to FRAME_HZ)
MAX_CATCHUP = 8                 # most physics frames run between two viewer frames when drawing is slow
OVERVIEW = ([-4.0, 0.0, 2.6], [4.0, 0.0, 0.5])  # camera eye / target: behind the start, down the hallway


def box(size, center, color, collide=True):
    """A fixed box: static collider (floor, walls) or, with collide=False, decoration only."""
    return sim_utils.CuboidCfg(
        size=size,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
        collision_props=sim_utils.CollisionPropertiesCfg() if collide else None,
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=0.9) if collide else None,
    ), center


def hallway_parts():
    """{name: (spawn cfg, centre)} for the floor, walls and doors."""
    length, cx = HALL_X1 - HALL_X0, (HALL_X0 + HALL_X1) / 2
    half = HALL_W / 2
    # a true ground plane, not a box: Newton's wheel-cylinder vs box contact jitters sideways and steers the chair
    floor = sim_utils.GroundPlaneCfg(physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=0.9))
    parts = {"floor": (floor, (0.0, 0.0, 0.0))}
    for side, y in (("left", half + 0.05), ("right", -half - 0.05)):
        parts[f"wall_{side}"] = box((length, 0.1, WALL_H), (cx, y, WALL_H / 2), (0.92, 0.92, 0.88))
        # doors every 4 m, so the motion is easy to see
        for i, x in enumerate(range(int(HALL_X0) + 3, int(HALL_X1), 4)):
            yd = y - 0.06 if y > 0 else y + 0.06
            parts[f"door_{side}_{i}"] = box((0.9, 0.02, 2.1), (x, yd, 1.05), (0.45, 0.30, 0.18), collide=False)
    parts["wall_end"] = box((0.1, HALL_W + 0.2, WALL_H), (HALL_X1 + 0.05, 0.0, WALL_H / 2), (0.92, 0.92, 0.88))
    return parts


def scene_cfg():
    cfg = InteractiveSceneCfg(num_envs=1, env_spacing=0.0)
    for name, (spawn, center) in hallway_parts().items():
        setattr(cfg, name, AssetBaseCfg(prim_path=f"/World/Hall/{name}", spawn=spawn,
                                        init_state=AssetBaseCfg.InitialStateCfg(pos=center)))
    cfg.dome_light = AssetBaseCfg(prim_path="/World/LightDome",
                                  spawn=sim_utils.DomeLightCfg(intensity=1500.0, color=(0.9, 0.9, 0.9)))
    cfg.sun = AssetBaseCfg(prim_path="/World/LightSun", spawn=sim_utils.DistantLightCfg(intensity=2500.0, angle=1.0))
    cfg.chair = STAXI_CFG
    return cfg


def main():
    check_usd()
    # the physics config here is a placeholder: the launcher swaps in the --physics choice
    substeps = PHYSICS_HZ // FRAME_HZ
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt=1 / PHYSICS_HZ, render_interval=substeps,
                                      physics=NewtonCfg())
    with launch_simulation(sim_cfg, args_cli):
        sim = SimulationContext(sim_cfg)
        scene = instantiate(scene_cfg())
        chair = scene["chair"]
        sim.set_camera_view(*OVERVIEW)  # once; after that the camera is the viewer's to drag
        sim.reset()
        dt = sim.get_physics_dt()

        base = chair.find_bodies("base_link")[0][0]
        # the push acts on the handle; the wrench API wants it relative to the base's centre of mass
        handle = torch.tensor(HANDLE, device=sim.device) - chair.data.body_com_pos_b.torch[0, base]
        print(f"[INFO] joints: {chair.joint_names}")
        print(f"[INFO] total mass {chair.data.body_mass.torch.sum().item():.1f} kg")

        def reset():
            chair.write_root_pose_to_sim_index(root_pose=chair.data.default_root_pose.torch.clone())
            chair.write_root_velocity_to_sim_index(root_velocity=chair.data.default_root_vel.torch.clone())
            chair.write_joint_position_to_sim_index(position=chair.data.default_joint_pos.torch.clone())
            chair.write_joint_velocity_to_sim_index(velocity=chair.data.default_joint_vel.torch.clone())
            scene.reset()

        def push(force):
            chair.permanent_wrench_composer.set_forces_and_torques_index(
                forces=torch.tensor([[[force, 0.0, 0.0]]], device=sim.device),
                positions=handle.view(1, 1, 3),
                body_ids=[base],
                is_global=False,  # force along the chair's own forward axis, at the handle
            )

        reset()
        st = dict(frame=0, loops=0, phase="push", stopped_for=0.0, x=0.0)
        realtime = bool(args_cli.visualizer) and not args_cli.fast  # --viz none parses to []
        frame_dt = substeps * dt

        def physics_frame():
            """The attendant's decision, then 1/FRAME_HZ s of physics. False once the requested runs are done."""
            x = st["x"] = chair.data.root_pos_w.torch[0, 0].item()
            y = chair.data.root_pos_w.torch[0, 1].item()
            v = chair.data.root_lin_vel_b.torch[0, 0].item()  # forward speed

            if args_cli.interactive:
                st["phase"] = "free"
            if st["phase"] == "push" and x >= PUSH_UNTIL_X:
                st["phase"] = "coast"
            if st["phase"] == "coast" and x >= BRAKE_FROM_X:
                st["phase"] = "brake"
            if st["phase"] == "push":
                force = min(MAX_PUSH, max(0.0, 250.0 * (args_cli.speed - v)))
            elif st["phase"] == "brake":
                force = -min(MAX_PUSH, max(0.0, 250.0 * v))
            else:
                force = 0.0
            push(force)

            stopped = abs(v) < 0.02 and st["phase"] in ("coast", "brake")
            st["stopped_for"] = st["stopped_for"] + frame_dt if stopped else 0.0
            if st["stopped_for"] > 1.5:
                st["loops"] += 1
                print(f"[INFO] run {st['loops']} done: stopped at x = {x:.2f} m, {y * 100:+.0f} cm off the centreline")
                if args_cli.loops and st["loops"] >= args_cli.loops:
                    return False
                reset()
                st.update(phase="push", stopped_for=0.0)

            for _ in range(substeps):
                scene.write_data_to_sim()
                sim.step(render=False)  # sim.step() would also send the viewer a frame every physics step
                scene.update(dt)
            st["frame"] += 1
            if st["frame"] % FRAME_HZ == 0:
                print(f"[t={st['frame'] * frame_dt:5.1f}s] {st['phase']:5s}  x={x:5.2f} m  y={y * 100:+5.1f} cm  "
                      f"speed={v:4.2f} m/s  handle force={force:+5.1f} N")
            return True

        wall0 = time.perf_counter()
        running = True
        while running and sim.is_running():
            # Physics keeps up with the wall clock and the viewer draws one frame per pass, however long
            # that takes: Viser needs < 1 ms, the Newton GL window under WSL ~40 ms (it then shows fewer
            # frames, but the chair still moves at real speed)
            due = int((time.perf_counter() - wall0) / frame_dt) - st["frame"] if realtime else 1
            for _ in range(min(max(due, 1), MAX_CATCHUP)):
                running = physics_frame()
                if not running:
                    break
            if args_cli.follow:
                sim.set_camera_view([st["x"] - 3.0, -0.5, 1.8], [st["x"] + 1.0, 0.0, 0.5])
            sim.render()
            # the Newton GL window's "Reset Episode" button puts the chair back at the start
            if any(getattr(viz, "consume_reset_request", lambda: False)() for viz in sim.visualizers):
                reset()
                st.update(phase="push", stopped_for=0.0)
            if realtime:
                ahead = st["frame"] * frame_dt - (time.perf_counter() - wall0)
                if ahead > 0:
                    time.sleep(ahead)
                elif ahead < -0.5:  # can't keep up even skipping frames: run slower rather than lurch
                    wall0 -= ahead


if __name__ == "__main__":
    main()
