"""Stack Staxi wheelchairs in Isaac Lab 3.0, to see whether (and how far) they nest.

N chairs stand in a row, all facing the same way (+x). The front chair is parked against a wall. Then, one at a
time from the front, an "attendant" pushes the next chair forward on its push handle at a slow walk, with at most
--force newtons, until it won't go any further (it has nested into, or butted against, the chair ahead). The
attendant lets go, the stack settles, and the next chair comes in. At the end it prints, for every chair, how far
its origin is behind the one ahead (the stacking pitch: less than a chair's length means they nest) and whether it
had to ride up or tip to get there, plus the length of the whole stack.

All joints are passive and the brake bar is zip-tied (wheels free), as in isaac_hallway.py.

usage (from any folder; run_stack.sh runs it in the Isaac Lab checkout's environment):
    bash run_stack.sh --viz newton_gl         # desktop window (right-click drag any chair to poke it)
    bash run_stack.sh --viz viser             # browser at http://localhost:8080
    bash run_stack.sh --chairs 2 --fast       # headless, two chairs, as fast as possible
"""

import argparse
import math
import os
import struct
import sys
import time
from pathlib import Path

from isaaclab.app import add_launcher_args, launch_simulation

sys.path.insert(0, str(Path(__file__).resolve().parent))  # isaac_common.py

parser = argparse.ArgumentParser(description="Stack Staxi wheelchairs.")
parser.add_argument("--chairs", type=int, default=10, help="number of chairs (2 or more)")
parser.add_argument("--speed", type=float, default=0.4, help="pace the attendant pushes each chair in at [m/s]")
parser.add_argument("--force", type=float, default=80.0, help="the attendant's maximum push [N]")
parser.add_argument("--gap", type=float, default=0.4, help="clear gap between neighbouring chairs at the start [m]")
parser.add_argument("--fast", action="store_true", help="don't slow down to real time when a viewer is open")
parser.add_argument("--hold", action="store_true", help="keep the window open after the last chair (Ctrl+C to quit)")
parser.add_argument("--physics", default="newton_mjwarp", choices=["newton_mjwarp", "isaacsim_physx", "physx", "ovphysx"],
                    help="physics engine (default: newton_mjwarp; see isaac_hallway.py)")
add_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.chairs < 2:
    parser.error("--chairs must be 2 or more")

if "microsoft" in Path("/proc/version").read_text().lower():  # WSL: run OpenGL on the GPU (see isaac_hallway.py)
    os.environ.setdefault("GALLIUM_DRIVER", "d3d12")
    os.environ.setdefault("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA")

import torch  # noqa: E402
from isaaclab_newton.physics import NewtonCfg  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import ArticulationCfg, AssetBaseCfg  # noqa: E402
from isaaclab.scene import InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab.utils import instantiate  # noqa: E402

from isaac_common import HANDLE, HERE, STAXI_CFG, check_usd  # noqa: E402

PHYSICS_HZ, FRAME_HZ = 240, 60
MAX_CATCHUP = 8
STOPPED_V, STOPPED_FOR = 0.02, 1.0  # a chair counts as stuck after this long below this speed [m/s, s]
SETTLE = 1.0                         # seconds with no push before the next chair comes in
TIMEOUT = 30.0                       # seconds per chair, at most
CONTACTS_PER_CHAIR = 120             # contact budget per chair (floor + nesting contacts with its neighbours)


def chair_extent_x():
    """Front and back of the chair along x [m], from the CAD (out/assembly_mm.stl, brake squeezed)."""
    data = (HERE / "out" / "assembly_mm.stl").read_bytes()
    n = struct.unpack_from("<I", data, 80)[0]
    import numpy as np
    rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)
    x = rec["v"][:, :, 0]
    return float(x.max()) / 1000, float(x.min()) / 1000


def main():
    check_usd()
    n = args_cli.chairs
    front, back = chair_extent_x()
    length = front - back
    pitch0 = length + args_cli.gap
    print(f"[INFO] chair length {length * 1000:.0f} mm (front {front * 1000:+.0f}, back {back * 1000:+.0f} mm from "
          f"the rear axle); {n} chairs at {pitch0:.2f} m apart, front one against a wall")

    substeps = PHYSICS_HZ // FRAME_HZ
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt=1 / PHYSICS_HZ, render_interval=substeps,
                                      physics=NewtonCfg())
    cfg = InteractiveSceneCfg(num_envs=1, env_spacing=0.0)
    # all chairs in ONE environment: chairs in different environments don't collide with each other
    xs0 = [-k * pitch0 for k in range(n)]
    for k, x in enumerate(xs0):
        setattr(cfg, f"chair_{k}", STAXI_CFG.replace(
            prim_path=f"{{ENV_REGEX_NS}}/Chair_{k}",
            init_state=ArticulationCfg.InitialStateCfg(pos=(x, 0.0, 0.005), joint_pos={".*": 0.0})))
    cfg.floor = AssetBaseCfg(prim_path="/World/Floor", spawn=sim_utils.GroundPlaneCfg(
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=0.9)))
    wall_x = front + 0.005 + 0.05
    cfg.wall = AssetBaseCfg(prim_path="/World/StopWall", spawn=sim_utils.CuboidCfg(
        size=(0.1, 1.6, 1.2), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.92, 0.92, 0.88)),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=0.9)),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(wall_x, 0.0, 0.6)))
    cfg.dome_light = AssetBaseCfg(prim_path="/World/LightDome",
                                  spawn=sim_utils.DomeLightCfg(intensity=1500.0, color=(0.9, 0.9, 0.9)))
    cfg.sun = AssetBaseCfg(prim_path="/World/LightSun", spawn=sim_utils.DistantLightCfg(intensity=2500.0, angle=1.0))

    with launch_simulation(sim_cfg, args_cli):
        # the launcher has swapped in the --physics preset; all chairs share one world's contact budget, and
        # nested chairs touch in many places, so raise it (an overflow silently drops contacts)
        solver = getattr(sim_cfg.physics, "solver_cfg", None)
        if solver is not None and hasattr(solver, "nconmax"):
            solver.nconmax = CONTACTS_PER_CHAIR * n
            solver.njmax = 8 * solver.nconmax
            print(f"[INFO] contact budget {solver.nconmax}, constraint budget {solver.njmax}")
        sim = SimulationContext(sim_cfg)
        scene = instantiate(cfg)
        chairs = [scene[f"chair_{k}"] for k in range(n)]
        x_mid = (xs0[0] + xs0[-1]) / 2
        sim.set_camera_view([x_mid + 0.3, -1.6 - 0.32 * n, 1.6], [x_mid, 0.0, 0.5])  # from the side
        sim.reset()
        dt = sim.get_physics_dt()
        base = chairs[0].find_bodies("base_link")[0][0]
        handle = torch.tensor(HANDLE, device=sim.device) - chairs[0].data.body_com_pos_b.torch[0, base]
        zero = torch.zeros(1, 1, 3, device=sim.device)

        def push(k, force):
            chairs[k].permanent_wrench_composer.set_forces_and_torques_index(
                forces=torch.tensor([[[force, 0.0, 0.0]]], device=sim.device), positions=handle.view(1, 1, 3),
                body_ids=[base], is_global=False)

        def state(k):
            c = chairs[k]
            x, z = c.data.root_pos_w.torch[0, 0].item(), c.data.root_pos_w.torch[0, 2].item()
            g = c.data.projected_gravity_b.torch[0]
            nose_up = math.degrees(math.asin(max(-1.0, min(1.0, -g[0].item()))))
            return x, z, nose_up, c.data.root_lin_vel_b.torch[0, 0].item()

        st = dict(frame=0, k=1, phase="approach", t=0.0, slow=0.0, force=0.0, done=False)
        results = {0: None}
        realtime = bool(args_cli.visualizer) and not args_cli.fast
        frame_dt = substeps * dt

        def physics_frame():
            k = st["k"]
            if not st["done"]:
                x, z, nose, v = state(k)
                st["t"] += frame_dt
                if st["phase"] == "approach":
                    st["force"] = min(args_cli.force, max(0.0, 250.0 * (args_cli.speed - v)))
                    st["slow"] = st["slow"] + frame_dt if (abs(v) < STOPPED_V and st["t"] > 1.0) else 0.0
                    if st["slow"] > STOPPED_FOR or st["t"] > TIMEOUT:
                        st.update(phase="settle", t=0.0, pushed_with=st["force"], timed_out=st["t"] > TIMEOUT)
                        st["force"] = 0.0
                elif st["phase"] == "settle" and st["t"] > SETTLE:
                    xa = state(k - 1)[0]
                    results[k] = dict(pitch=xa - x, z=z, nose=nose, force=st["pushed_with"], timeout=st["timed_out"])
                    print(f"[chair {k}] stopped {xa - x:.3f} m behind chair {k - 1} (a chair is {length:.3f} m long: "
                          f"{'nested ' + format((length - (xa - x)) * 1000, '.0f') + ' mm' if xa - x < length - 0.005 else 'not nested'})"
                          f"; pushed with {st['pushed_with']:.0f} N; base z {z * 1000:+.0f} mm, tilt {nose:+.1f} deg"
                          + ("; TIMED OUT still moving" if st["timed_out"] else ""))
                    if k + 1 < n:
                        st.update(k=k + 1, phase="approach", t=0.0, slow=0.0)
                    else:
                        st["done"] = True
                        summary()
                        if not args_cli.hold:
                            return False
                for j in range(n):
                    push(j, st["force"] if (j == st["k"] and not st["done"]) else 0.0)
            for _ in range(substeps):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(dt)
            st["frame"] += 1
            if st["frame"] % FRAME_HZ == 0 and not st["done"]:
                x, z, nose, v = state(st["k"])
                print(f"[t={st['frame'] * frame_dt:5.1f}s] chair {st['k']} {st['phase']:8s} x={x:6.3f} m  "
                      f"speed={v:5.2f} m/s  push={st['force']:5.1f} N  tilt={nose:+.1f} deg")
            return True

        def summary():
            xs = [state(j)[0] for j in range(n)]
            pitches = [results[j]["pitch"] for j in range(1, n)]
            stack = (xs[0] + front) - (xs[-1] + back)
            print("\n[RESULT] stacking pitch (origin to origin) per chair:",
                  ", ".join(f"{p * 1000:.0f}" for p in pitches), "mm")
            print(f"[RESULT] mean pitch {sum(pitches) / len(pitches) * 1000:.0f} mm vs chair length {length * 1000:.0f} mm; "
                  f"{n} chairs take {stack:.2f} m (end to end: {n * length:.2f} m)")
            moved = xs0[0] - xs[0]
            if abs(moved) > 0.01:
                print(f"[RESULT] the front chair moved {moved * 1000:+.0f} mm")

        wall0 = time.perf_counter()
        running = True
        while running and sim.is_running():
            due = int((time.perf_counter() - wall0) / frame_dt) - st["frame"] if realtime else 1
            for _ in range(min(max(due, 1), MAX_CATCHUP)):
                running = physics_frame()
                if not running:
                    break
            sim.render()
            if realtime:
                ahead = st["frame"] * frame_dt - (time.perf_counter() - wall0)
                if ahead > 0:
                    time.sleep(ahead)
                elif ahead < -0.5:
                    wall0 -= ahead


if __name__ == "__main__":
    main()
