# Wheelmo simulation (Isaac Lab 3.0)

A first, deliberately small simulation of the autonomous wheelchair: a rear-wheel-drive power chair with
**one 360° LiDAR** and **two 1920×1080 RGB cameras** driving down a **tightly packed corridor**, plus a baseline
LiDAR planner that closes the sensor → command loop so there is something to replace with the real stack.

It runs on Isaac Lab's kit-less stack (Newton/MuJoCo-Warp physics, Warp ray-cast LiDAR, Newton Warp camera
renderer). **Isaac Sim is not installed or needed**, and every asset is generated procedurally, so nothing is
downloaded at runtime.

## Setup

Requirements: Linux x86_64 (WSL2 works) with GLIBC ≥ 2.35, an NVIDIA GPU and driver ≥ 580.65 (Linux) / ≥ 580.88
(Windows host for WSL2), and [`uv`](https://docs.astral.sh/uv/). Python 3.12 is fetched by uv.

```bash
cd Simulations
uv sync              # ~5 GB: Isaac Lab 3.0.0, PyTorch 2.11 (CUDA 13.0), Newton, Warp
uv run pytest        # fast checks that don't need the simulator
```

## Run

```bash
uv run python run_corridor.py --max_steps 8000                 # headless, ~40 s of sim time
uv run python run_corridor.py --viz newton_gl                  # with a viewer window
uv run python run_corridor.py --save_every 25 --max_steps 4000 # dump front/rear RGB + LiDAR bird's-eye PNGs to outputs/
uv run python run_corridor.py --seed 3                         # another corridor layout
uv run python run_corridor.py --num_envs 4                     # 4 chairs in parallel (each renders 2x 1080p)
uv run python run_corridor.py --controller constant --cmd 0.3 0.2   # fixed (v [m/s], w [rad/s]), no planner
```

Each episode ends when the chair reaches the end of the corridor, tips over, or times out, and is then reset.
On an RTX 4060 Laptop GPU (8 GB) one environment runs at about 2× real time headless with both cameras rendering
at 30 Hz, and about 0.6× real time with the viewer open. Startup takes about 30 s, mostly solver initialisation.

### Viewer on WSL2

Physics, LiDAR and cameras always run on the NVIDIA GPU through CUDA. Only the `--viz newton_gl` window uses
OpenGL, and under WSLg that has some traps:

- `LIBGL_ALWAYS_SOFTWARE=1` (set in `~/.profile` on the dev laptop) forces OpenGL onto the CPU. Run with
  `LIBGL_ALWAYS_SOFTWARE=0 uv run python run_corridor.py --viz newton_gl` to use the GPU. The script prints a
  warning when it detects the variable.
- On hybrid-graphics laptops WSLg defaults to the integrated GPU. `run_corridor.py` sets
  `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` unless you set it yourself. Check with
  `LIBGL_ALWAYS_SOFTWARE=0 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA glxinfo -B`.
- Each viewer frame costs about 30 ms on WSL: state goes GPU → host → OpenGL because CUDA–OpenGL sharing isn't
  available, then WSLg ships the frame to Windows. The viewer redraws at `--render_hz` (default 30). Lower it if
  the sim needs to run faster.
- Windows Task Manager's default GPU graphs show 3D work only. To see CUDA load from WSL, switch one of the
  NVIDIA GPU's graphs to "Cuda" or "Compute", or run `nvidia-smi -l 1` inside WSL.

## What is simulated

| Part | Default | Defined in |
| --- | --- | --- |
| Wheelchair | Rear-wheel-drive power chair, 0.63 m wide × 1.1 m long, 160 kg incl. a 75 kg occupant (mass only). Velocity-controlled 14″ mag drive wheels (80 N·m), passive 8″ swivel casters, battery/motor base, cushioned seat and back, armrests with joystick, split footplates, anti-tip wheels | `wheelmo_sim/wheelchair_spec.py`, `wheelmo_sim/wheelchair_model.py` |
| LiDAR | 32 beams, ±45° vertical, 0.35° horizontal (32,928 rays), 30 m, 10 Hz, on a mast 1.5 m above the floor | `lidar_cfg` in `wheelmo_sim/scene.py` |
| Front camera | 1920×1080 RGB, ~90° HFOV, 30 Hz, front of right armrest at 0.8 m, pitched 15° down | `rgb_camera_cfg` in `wheelmo_sim/scene.py` |
| Rear camera | Same, on the mast at 1.1 m facing backwards, pitched 20° down | same |
| Corridor | 20 m × 1.6 m, 2.4 m walls, ~10 obstacles (carts, cabinets, benches, bins, plants) along both walls, a 0.9 m doorway at x = 10 m. A free lane ≥ 0.9 m is guaranteed (the chair is 0.63 m wide). | `wheelmo_sim/corridor_layout.py` |
| Baseline planner | Remembers LiDAR returns in a 5 cm occupancy grid and runs a dynamic-window planner over the chair's rectangular footprint, at 50 Hz | `wheelmo_sim/controller.py` |

All sensor mounts and dimensions derive from `WheelchairSpec`, so measuring the real chair and editing that one
dataclass updates the asset, the sensor poses and the planner footprint together.

### Visual vs. collision geometry

Every wheelchair body has a `visual` group (procedural meshes and tubes) and a `collision` group (a few
boxes/cylinders with USD `purpose = guide`). Follow this split when you change the model:

- Isaac Lab's Newton importer **hides any collider on a body that also has visual geometry**. A part authored
  as a visible collider will vanish from the viewer as soon as anything decorative is added to the same body.
  That is why the first model showed hubs without tyres and no seat.
- The LiDAR hits visual geometry too, and camera sensors also draw collision proxies (they share the ray-cast
  scene). So keep every proxy inside its visual, and keep all geometry clear of the LiDAR's rays: the puck sits
  below the optical centre by at least its radius.

## Where the autonomy stack plugs in

`run()` in `run_corridor.py` is the seam. Each control step it reads

- `scene["lidar"].data.ray_hits_w.torch`: (num_envs, 32928, 3) hit points in world frame (`inf` = no return),
- `scene["front_camera"].data.output["rgb"]` / `scene["rear_camera"]...`: (num_envs, 1080, 1920, 3) uint8,
- `robot.data.root_pos_w`, `root_quat_w` (x, y, z, w), `root_lin_vel_b`: ground-truth pose / odometry,

and writes left/right drive-wheel velocity targets [rad/s] via
`robot.actuators.target_command.set_velocity_index(...)`. `WheelchairSpec.wheel_speeds(v, w)` converts a body
twist into wheel speeds.

## Known gaps (it is preliminary)

- **LiDAR blind spot.** From a 1.5 m mast, low obstacles (a 0.45 m bench) drop out of view within ~1 m of the
  chair. The baseline planner only copes because it remembers what it saw earlier. Decide the real mount early,
  then set it in `WheelchairSpec`.
- **The occupant has no geometry**, so the sim shows no self-occlusion of the LiDAR or cameras by the user's
  head, legs or feet.
- **Ideal sensors**: no LiDAR noise, dropouts or motion distortion; no camera noise, exposure or lens distortion.
  Newton Warp is a fast direct-lighting renderer. For photoreal images use the OVRTX renderer
  (`isaaclab[ovrtx]`).
- **Static world**: obstacles never move and the obstacle memory never clears. People and doors need dynamic
  bodies plus a memory that clears free space.
- **Simplified dynamics**: idealised velocity-controlled motors, one friction coefficient everywhere, rigid tyres.
- **No ROS 2 bridge yet, and not yet an RL environment.** Wrapping the scene in an Isaac Lab `DirectRLEnv` and
  publishing sensors to ROS 2 are the natural next steps.

## Files

```
run_corridor.py              entry point: launch, control loop, episode resets, image dumps
wheelmo_sim/
  wheelchair_spec.py         chair dimensions, masses, sensor mounts (pure Python)
  corridor_layout.py         procedural corridor generator + feasibility helpers (pure Python)
  wheelchair_model.py        wheelchair geometry: visual meshes/tubes, hidden collision proxies, joints
  meshes.py                  procedural meshes: revolved tyres/rims/hubs, rounded boxes (pure numpy)
  usd_utils.py               low-level USD authoring helpers (shapes, meshes, bodies, joints)
  assets.py                  Isaac Lab asset configs: wheelchair articulation + actuators, corridor colliders
  scene.py                   Isaac Lab scene: corridor, robot, LiDAR, two cameras
  controller.py              baseline: LiDAR obstacle memory + dynamic-window local planner
  sensor_io.py               PNG writers for camera frames and LiDAR bird's-eye views
tests/                       layout feasibility, kinematics, meshes, planner behaviour (no simulator needed)
```
