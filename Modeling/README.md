# Modeling: Staxi wheelchair CAD and simulation model

A clean, parametric, simulatable model of the **standard Staxi medical wheelchair** (not the airport
chair), rebuilt from a phone scan. The Staxi is the platform for the Wheelmo autonomy kit, so this model is
what the kit, the sensor-placement study (`../sensor_placement_strategy.md`) and `../Simulations` build on.

The model exists as code (build123d), and every output is generated from it:
- articulated **URDF** + meshes, for PyBullet and Isaac Lab;
- **USD**, for Isaac Lab / Isaac Sim;
- **STEP**, for mechanical CAD;
- **STL**, for quick looks.

Status (2026-11-23):
- 93.9% of the scanned surface is within 12 mm of the CAD.
- The front casters were rebuilt from the scan: their median distance to it is 1.9 mm, about the scan's noise.
- The rear wheels, their wave-edged parking-brake cog, the brake rod / holder / spring / pawl, and the oval
  frame tubes around the wheels were rebuilt from the scan. The wheel shape is within 0.8–1.3 mm of the
  scan (median) on the tread and outer face.
- It passes PyBullet physics checks.
- It runs in Isaac Lab 3.0 (Newton physics) in WSL2.
- Dimensions not yet confirmed on the real chair are listed under **Known estimates**.

## Quick start

```bash
# one-time: Python environment for the CAD tools (Python 3.10 tested)
python3 -m venv ~/.venvs/wheelchair && ~/.venvs/wheelchair/bin/pip install -r requirements.txt

cd staxi
PY=~/.venvs/wheelchair/bin/python
$PY build.py                                       # CAD -> URDF, meshes, STEP, STLs in out/
$PY sim_check.py review/07_physics_check_pybullet.png     # PyBullet physics test
$PY deviation.py review/06_scan_vs_cad_deviation.png      # scan-vs-CAD heat map
$PY review_render.py review/05_cad_v2_renders.png         # six-view renders
$PY view.py                                        # send the model to the OCP CAD Viewer in VS Code
bash make_usd.sh                                   # URDF -> USD for Isaac Lab (after every build.py)
bash run_hallway.sh --viz viser                    # Isaac Lab hallway demo, browser at http://localhost:8080
```

`make_usd.sh` and `run_hallway.sh` run from any folder; they use the Isaac Lab environment (see **Isaac Lab**).

## Layout

| Path | What |
|---|---|
| `requirements.txt` | Python packages for the CAD tools |
| `.vscode/settings.json` | VS Code: Python interpreter = the venv above; OCP viewer in orbit mode |
| `docs/OCP_CAD_VIEWER_SHORTCUTS.md` | mouse and keyboard cheat sheet for the OCP CAD Viewer |
| `staxi/params.py` | **every dimension**, each tagged `[scan]` (measured on the scan), `[spec]` (Staxi published spec), `[user]` (from someone who has the chair) or `[est]` (estimate) |
| `staxi/model.py` | build123d parts, collision primitives, joints |
| `staxi/build.py` | writes everything in `out/` |
| `staxi/out/staxi_wheelchair.urdf` + `out/meshes/` | articulated model, metres |
| `staxi/out/staxi_wheelchair.step` | CAD assembly, mm (one compound for now; named parts are a to-do) |
| `staxi/out/assembly_mm.stl` | quick-look mesh (Windows "3D Viewer") |
| `staxi/out/overlay_scan_pose_mm.stl` | the CAD posed like the scan, used by `deviation.py` |
| `staxi/out/usd/` | USD for Isaac Lab, made by `make_usd.sh` (not in git) |
| `staxi/sim_check.py` | PyBullet test: settle, push, turn, tip back onto the anti-tip wheels, armrests |
| `staxi/isaac_hallway.py` | Isaac Lab demo: push the chair down a hallway, or drag it yourself |
| `staxi/make_usd.sh` | URDF -> USD (clears the old copy; Isaac's importer never overwrites) |
| `staxi/run_hallway.sh` | runs `isaac_hallway.py` in the Isaac Lab environment, from any folder |
| `staxi/view.py` | live 3D view in VS Code (OCP CAD Viewer) |
| `staxi/clean_scan.py` | raw scan OBJ -> floor removed, aligned STL |
| `staxi/scan_tools.py` | scan -> model frame (scale, axes), point-splat renderer |
| `staxi/blueprint.py` | orthographic scan views with a mm grid, optional CAD overlay |
| `staxi/skeleton.py`, `staxi/tube_paths.py` | 3D skeleton of the scan -> tube centrelines (used to place the frame tubes) |
| `staxi/deviation.py`, `staxi/review_render.py` | scan-vs-CAD heat map, CAD renders |
| `staxi/review_caster.py` | front-caster close-up: the CAD next to the scan coloured by distance to it |
| `staxi/review_footrest.py` | footrest close-up: the CAD next to the scan coloured by distance to it |
| `staxi/review_rear.py` | rear wheel + parking brake close-up: CAD, scan coloured by distance, CAD with the brake squeezed; optionally a CAD close-up of the latch (cog + pawl) |
| `staxi/scan/` | raw scan and cleaned STL (not in git, see `staxi/scan/README.md`) |
| `staxi/data/` | caches rebuilt from the scan (not in git) |
| `staxi/review/` | review images from each iteration (below) |

## The chair (facts from the people who have it)

- **Standard Staxi medical chair.** It doesn't fold; it **stacks**.
  - The lower bars at the back are the stacking/nesting bracket.
  - The frames under the seat are structural.
- **Brake:**
  - A **squeeze bar** behind the backrest, hinged around the lower middle of the backrest. At the top it
    sits about 5 cm from the push handle and needs medium force to squeeze.
  - The brake is **on by default**: on each side a push-rod runs down beside the rear post, held by a
    holder on the post a few cm above the wheel. A spring under the holder pushes the rod down, and the
    pawl on its end drops into the cog on the inner side of the rear wheel. Squeezing the bar lifts it out.
  - The cog has **no sharp teeth**: its edge is a smooth up-and-down wave. Each trough is a semicircular
    socket that the pawl's rounded tip fits; if the tip lands on a crest it slides into the next trough.
    The connector from the rod down to the wheel is a flat bar, vertical and then diagonal.
    See **Rear wheels and parking brake** below.
  - **In the model the bar is zip-tied squeezed**, as it will be in real use, so the rear wheels always
    roll freely.
- **Armrests** flip up about 110° on a rear hinge.
- **Front casters** swivel a full 360°, smoothly. They're stem-mounted swivel casters of the shopping-cart
  kind; see **Front casters** below for how they were measured.
- **Footrests** are fixed. Each is one pipe loop, a rectangle with rounded ends. The end nearest the frame
  is bent upward and welded to the frame's front tube. See **Footrests** below.
- The rods behind the backrest are a **metal back pocket** (for papers).
- **Anti-tip wheels:** small wheels at the tips of the two rear base rods. If the chair tips back, it lands
  on them and can still roll.

## How the model was made

1. **Scan.**
   - A phone scan exported by Artec (`model.obj` + `model.mtl` + `model_1.jpg` texture): 1.33 M vertices,
     with the floor and room clutter.
   - `clean_scan.py`:
     1. RANSAC floor fit, then remove the floor.
     2. Keep the chair's connected pieces.
     3. Find the left/right mirror-symmetry heading.
     4. Write `wheelchair_clean_aligned.stl`.
   - This step was reconstructed from the original session log; it reproduces the original STL byte for
     byte, in about 10 s.
2. **Scale.**
   - The OBJ's units are not mm: read as mm, the chair would be 3.4 m tall.
   - We fitted one scale factor to Staxi's published specs for the standard chair: 27" wide, 21.7" seat
     to floor, 8" rear wheels, 5" casters, 57 lb.
   - Result: **0.2935 mm per scan unit**. Width, seat height and wheel size agree within about 3%. Length
     matches the older 42" listing, not the 38" one.
3. **Model frame.** X forward, Y left, Z up. Origin on the floor, midway between the rear wheels, directly
   under the rear axle. Same frame in mm (CAD) and m (URDF/USD).
4. **Measure.**
   - Orthographic "blueprints" of the scan with a mm grid (`blueprint.py`).
   - Thin slices of the scan.
   - A 3D skeleton (4 mm voxels, `skeleton.py`) turned into tube centrelines (`tube_paths.py`).
   - From these, the tube paths, the cushion profile, the wheel positions and the joint axes.
5. **CAD** (`model.py`):
   - Frame tubes are swept along filleted polylines.
   - The cushion is one moulded L-profile.
   - Each moving part is its own link.
   - Collision shapes are primitives only: cylinders per tube segment and per wheel, boxes for the cushion.
6. **Exports** (`build.py`):
   - Masses from the Staxi spec, 57 lb (25.9 kg) total, split by estimated part masses.
   - Inertias from the geometry, with the foam cushion (solid) and the metal handled separately. The
     metal's mass is spread over its surface, not its volume, since the frame is thin-walled tube.
7. **Checks:** scan deviation, PyBullet physics, Isaac Lab, and review images for a human to look over.

### Iterations and fixes (review images in `staxi/review/`)

| Image | What it shows / what changed |
|---|---|
| `01_part_labels_first_guess.png` | First guess at which scan regions are which parts |
| `02_right_side_brake_linkage.png`, `03_push_handle_and_squeeze_bar.png`, `04_rear_wheel_cog.png` | Close-ups used to work out the brake bar, push handle and cog brake |
| `05_cad_v2_renders.png` | Current CAD, six views (wheels striped black/white) |
| `06_scan_vs_cad_deviation.png` | Scan coloured by distance to the CAD: green < 12 mm, yellow 12–25 mm, red > 25 mm |
| `07_physics_check_pybullet.png` | Snapshots from `sim_check.py` |
| `08_front_caster.png` | Front caster close-up: CAD (top), scan coloured by distance to it (bottom) |
| `09_rear_wheel_brake.png` | Left rear wheel + parking brake: CAD as scanned (top), scan coloured by distance (green ≤ 2 mm, light green 2–4, yellow 4–8, red > 8), CAD with the brake squeezed (bottom) |
| `10_parking_brake_latch.png` | The latch, CAD only: cog ring and pawl (brown), brake released (top) and squeezed (bottom) |
| `11_footrest.png` | Left footrest: CAD (top), scan coloured by distance (bottom; green ≤ 2 mm, light green 2–4, yellow 4–8, red > 8) |

**v1 → v2:**
- **Cushion:** the profile was replaced with one measured from scan slices; the match went from 76% to
  91% within 12 mm.
- **Inertia:** the base inertia now treats the metal and the foam separately.
- **Axle stub:** trimmed so it no longer pokes through the wheel.

**Later fixes, from review in the OCP viewer:**
- **Footrests** were floating. The loops were too narrow (52 → 62 mm) and too short. The "hook" connector
  followed a ghost surface in the scan. They then got one straight 3/4" beam per side, continuing the
  seat-rail line from the frame corner down onto the loop's outer end. That beam was wrong too; see
  **Footrests** below for the current model.
- **Anti-tip wheels** added: 38 mm wheels in small forks behind the rear rod ends, 36 mm off the floor.
  The chair touches down on them at about 9.5° of backward tilt.
- **Stripes:** black and white sectors on every wheel (8 sectors on the rear and caster wheels, 6 on the
  anti-tip wheels), so you can see them turn.
- **Rubber tyres and bearings:** high-grip tyres and nearly frictionless bearings (joint damping 1e-4,
  anti-tip wheels 1e-5).

### Front casters (rebuilt from the scan, 2026-11-23)

The first casters were placeholders (a box yoke, a 30 mm wheel, 30 mm trail). The real ones are
stem-mounted swivel casters like a shopping-cart caster:
- **Swivel head:** a bell-shaped head, widest (59 mm) where it meets the yoke and narrowing to 42 mm at the
  top. It turns in a vertical 39 mm housing under the frame's base tube.
- **Yoke:** pressed steel, about 3 mm plate. A hood, D-shaped from above, rounded at the front about the
  swivel axis, with its back sloping down to a lip behind the wheel. Two J-shaped legs, about 34 mm wide,
  run from under the head down past the wheel to rounded tips about 22 mm below the axle.
- **Wheel:** 128 mm diameter (5"). A tyre about 36 mm wide with rounded shoulders and a flat ~22 mm tread,
  on a dark wheel centre that is wider than the tyre (48 mm) and fills the yoke between the legs.
- **Axle:** flush with the outside of the legs; nothing sticks out past them in the scan.
- **Trail:** 37 mm (the axle behind the swivel axis).

How it was measured (both casters, which agree to within about 2 mm):
1. Fit each wheel's plane from its scan points, and a circle to its tread (residual 1.3 mm RMS). That
   gives the radius, the axle and the swivel angle each caster was scanned at: −50° and −69°, now
   `SCAN_POSE`.
2. Locate the swivel axis from the material just under the frame, then rotate each caster into its own
   frame so both overlay.
3. Read each feature from slices and slabs in that frame: the wheel's cross-section from its lower half,
   where no fork is in the way; the leg outline from the plate just outside the wheel centre; the head and
   hood from horizontal slices.
4. Check the result as a CAD overlay on the slices, then as distance maps (`review/08_front_caster.png`).

Result in the caster region (both casters): before → after, within 3 mm 32% → 72%, within 5 mm 48% → 90%,
within 12 mm 88% → 97%; median distance 5.2 → 1.9 mm, 90th percentile 12.7 → 5.1 mm.

### Rear wheels and parking brake (rebuilt from the scan, 2026-11-23)

Rebuilt to near 1:1 for designing a friction-wheel drive on the rear wheels. All values are `params.py`
(model frame: X forward, Y left, Z up, mm; origin on the floor under the rear axle, midway between the
wheels). The wheel frame: `a` = axial from the tyre mid-plane (+ = outward), `r` = radius from the axle.

**Rear wheel** (both wheels agree in shape to ~1 mm):

| Feature | Value |
|---|---|
| Axle | x 0.5, z 97.5, along Y (no toe or camber) |
| Tyre mid-plane | \|y\| 318 (scan: left 315, right 321; the model uses the mean) |
| Tread | crown radius 101.0 (Ø 202; 8" nominal is Ø 203.2). Crowned: r ≈ 101 − 0.025 a², so 100.1 at ±6, 97.6 at ±12, 93.5 at ±18 |
| Tyre | bulges past the tread: widest 55 mm (±27.5) at r 75, pinched to ±24 where it meets the wheel centre at r 62 |
| Outer face | slightly domed: a +23 at r 48–55, +25 at r 0–28. Nothing sticks out at the axle |
| Inner face | dished disc: a −39.6 at r 47–50, −42 at r 37, −43.8 at r 27; hub boss Ø 49 at a −49.5 (bears on a spacer); axle-bolt recess r < 11 at a −40 |
| Cog | a ring on the inner face whose outer edge is a smooth wave: 20 rounded crests and 20 troughs (18° pitch), no sharp teeth. Crests top out at r 60.5, troughs bottom out at r 51 (9.5 mm deep). Each trough is a U, 10 mm wide with parallel sides and a semicircular bottom (R 5) that the pawl's rounded tip seats in. The crests are rounded (R 3.5 corners, nearly fully round), so a tip that lands on one slides into a trough. The ring's face is a shallow cone (a −40.5 at r 50 → −38 at r 60); a trough's back wall is the wheel face behind the ring (a −34.2 at r 50 → −25.9 at r 62) |

**Parking brake** (per side; left-side values, pawl engaged as scanned):

| Part | Value |
|---|---|
| Push-rod | Ø 8, parallel to the rear post and 27 mm behind its axis (passes (−67.5, 251.7, 360)). Runs from the collar (z 244) up to z 648, then bends inward toward the brake bar |
| Holder | eye Ø 24 around the rod at z 296–306, on a short web welded to the back of the post |
| Spring | compression spring Ø 19 OD, ~2 mm wire, 9 coils, between the collar and the holder: 46 mm long engaged, 33 mm squeezed |
| Collar | Ø 21 × 6 on the rod, z 244–250: the spring seat |
| Pawl | flat bar 22 × 8 hung outboard of the rod: straight down (x −40..−18, \|y\| 261–269, z 180–244), then diagonally outward and down, narrowing to the tip. The tip is 9 mm wide across the trough and spans \|y\| 272–284; its end is a semicircle (R 4.5, seen along the axle) that seats in a trough |
| Travel | the pawl's line of travel passes through the axle (109° from forward), so the tip enters a trough straight on. Engaged, the rounded end sits at r 51.5, 0.5 mm off the trough bottom and 9 mm below the crests. Squeezed (the default, zip-tied), everything slides 13 mm up the rod axis: the tip clears the crests by 4 mm |

In the scan the right pawl sits in a trough and the left one rests on a crest (the spring pushes it in as
soon as the wheel turns). `SCAN_POSE` reproduces both: it turns the left wheel to its scanned phase and lifts
the left pawl 9 mm.

**Frame around the wheel:** the base tube and the rear post / push-handle loop are **oval tubes**, about
22 × 40 mm (base) and 24 × 40 mm (post: 40 mm fore-aft). Both were round 25.4 mm in the CAD before; that put
the post 6 mm too far back and the base tube 4–9 mm off. Next to the wheel the base tube's centre line is
z = 97.5 + 0.355 x at \|y\| ≈ 253, so the axle bolt passes straight through it. The axle stack, inside to
out: bolt head inside the tube, tube (\|y\| 242–264), spacer (264–268.5), hub boss. A small bent tab stands
on the base tube just in front of the pawl (x −21, \|y\| 243–263, up to z 148).

**Clearances from the tyre** (brake squeezed): post 23.5 mm, base tube 25.6 mm, pawl tip 6.5 mm, pawl
diagonal 6.8 mm, pawl blade 26.4 mm. The tyre occupies x ±101, \|y\| 290–346, z −3.5..198.5. Nothing is
outboard of the tyre.

How it was measured:
1. Fit each wheel's tread circles in thin slabs to get the axle and radius. The fitted ±2.5° toe and camber
   were artefacts; a pure-Y axle gives a consistent outer face.
2. Express the scan in each wheel's frame. Read the cross-section (a, r) from angular sectors no frame part
   covers.
3. Fold the cog zone onto one pitch (a periodogram over angle gives 20 on both wheels) and look at it along
   the axle: the ring's face shows round-bottomed troughs 10 mm wide down to r 51, between crests up to
   r 60.5.
4. Read the rod, holder, spring and collar from slices along the rod axis. For the pawl, remove every scan
   point the CAD already explains (wheel, tubes, rod, spring, holder, collar) and slice what's left: a
   vertical blade down to z ≈ 180, then a diagonal outward toward the top of the cog. The texture doesn't
   help separate it: steel and tyre are the same dark grey in the scan's colours.
5. Measure the base tube and post from sections every 20–40 mm along their length.

Result, scan → CAD (before → after):
- Wheel shape with the ±3 mm side offset removed, median distance: tread 3.4–3.7 → 1.2–1.3 mm; outer face
  1.6–2.5 → 0.8–1.2 mm; inner face and cog 4.4–7.8 → 1.6–2.4 mm.
- Base tube near the wheel: within 2 mm 17% → 52%; within 6 mm 48% → 96%.
- Rod / holder / spring / pawl / post region: within 2 mm 26% → 37%; median 4.3 → 3.0 mm.
- Whole chair within 12 mm: 91.4% → 93.1% (93.9% after the footrest rebuild).
- The cog ring alone (left wheel, the sectors the scanner saw clearly): median 0.87 mm, 87% within 2 mm.
  The scan's noise is too large to tell the wave from the earlier toothed version by score (0.89 mm); the
  shape follows the description from the people who have the chair and the folded scan view.

**Not resolved:**
- **The linkage at the top of the rod.** Where the rod meets the brake bar's hinge isn't clear in the
  scan. The rod's top bend and the 13 mm lift are estimates: the lift is the 9 mm trough depth plus 4 mm
  of clearance.
- **A plate on the inside of the left wheel.** The scan shows material the CAD doesn't have, mainly
  behind the top of the left wheel, on its inner side: a curved band at \|y\| 258–276, roughly r 60–105
  around the upper rear quarter (x −95..+30, z 90–185). It could be a guard over the cog and pawl, or the
  reconstruction filling the hidden gap between the pawl, frame and tyre. The right side shows only a faint
  outline. Check the real chair before designing anything into that space.

### Footrests (rebuilt from the scan, 2026-11-23)

Each footrest is **one closed loop of 5/8" (Ø 16) tube**:
- **Footplate:** a flat rectangle with rounded ends, tilted 28° front-edge-up. The long sides run along y,
  62.3 mm apart (centrelines at x 657.9, z 126.8 and x 712.9, z 156.0). The inner end is a semicircle,
  its centreline 11 mm from the chair's centre plane, so the two footplates nearly meet in the middle.
- **Bent-up outer end:** at |y| 148 both long sides bend up (radius 12) through **66°**, run 23 mm
  straight, and join in a semicircle the width of the loop. That makes a standing U whose top is 56 mm
  above the footplate, at |y| 181. It leans about 24° outward from square: it wraps around the outside of
  the front downtube and is welded to it.
- **Front downtube:** the frame's front tube, an oval 24 × 40 mm like the rear posts. It doesn't stop at
  the base-tube corner: it carries on down between the footplate's long sides and ends 95 mm below the
  corner (z ≈ 127), just under the footplate.

The right footrest sits about 6 mm further forward than the left in the scan; the model uses the mean.

How it was measured:
1. Fit circles to the long sides seen end-on in thin slabs along y. That gives the tube diameter (Ø 16), the
   width (62.3 mm) and the tilt (28°).
2. Express the scan in the footplate's own frame and slice the outer end along y. Render it from several
   angles to see the bend, the U and the downtube passing through it.
3. Remove the downtube (oval mask) and read the U face-on and from above. The top is 56 mm up at |y| 181,
   which fixes the bend angle at about 66°, not 90°. A U standing at 90° would run into the downtube.
4. Measure the downtube's section every 15 mm along it, and where it ends.

Result, scan → CAD (before → after), median distance: outer end, U and the downtube's foot 7.6 → 2.5 mm
(within 4 mm 31% → 71%); downtube z 235–330 6.1 → 2.1 mm; footplate long sides 2.4 → 2.35 mm. The
remaining red patches sit at the welded junction, where the bends meet the downtube's end: weld material and
the end's shape aren't modelled. The collision shape follows the tube with capsules every 25 mm, so
stacking contacts land on the real outline.

Not resolved: you said the end is bent 90°, but the scan reads 66° from the footplate. If the real chair is
90° (for example, square to the floor rather than to the tilted footplate), change `FOOTREST_BEND["deg"]`.

## Moving parts (URDF)

| Link | Joint | Axis | Range |
|---|---|---|---|
| `base_link` | frame, cushion, push handle, brake bar + back pocket, footrests, stacking basket | — | — |
| `rear_wheel_l/r` | continuous | Y (axle) | free (brake zip-tied off) |
| `caster_fork_l/r` | continuous | Z (swivel) | free, 360° |
| `caster_wheel_l/r` | continuous | Y (axle) | free |
| `anti_tip_wheel_l/r` | continuous | Y (axle) | free; touch down at ~10.7° tilt-back |
| `armrest_l/r` | revolute | −Y (rear hinge) | 0 → 110° flip-up |

Total mass 25.9 kg. Joint damping is in the URDF. Tyre friction isn't: URDF has no field for it, so each
simulator sets it (`sim_check.py`: μ = 1.0; `isaac_hallway.py`: a rubber material, 1.2 static / 1.0
dynamic).

## Validation (2026-11-23)

- **Fit to scan:** 93.9% of the scan surface within 12 mm of the CAD, 99.0% within 25 mm. The scan's tubes
  are inflated a few mm by noise, so "within 12 mm" counts as a match. The remaining red patches are a
  label on the backrest, the lowest back-pocket wire, floor fragments, and the unexplained material inside
  the left rear wheel (see **Rear wheels and parking brake**).
- **PyBullet** (`sim_check.py`, all checks pass):
  - It rests level on exactly the four wheels.
  - A 22 N push on the handle for 2 s rolls it 3.8 m dead straight (1.37 m/s), casters trailing.
  - A one-sided push turns it.
  - Tilting it back by the handle, from standstill, at ~15°/s (it starts to tip at 140 N), it lands on the
    anti-tip wheels at 10.7° and stays there under 130 N. Pushed while tipped, it rolls 1.4 m on the rear
    and anti-tip wheels, the anti-tip wheels rolling for 89% of the distance. Released, it drops back onto
    all four wheels.
  - An armrest lifted 50° falls back; one flipped to 110° stays up.
  - The tilt test changed with the rear-wheel rebuild. It used to press with a fixed ramp from 100 N
    straight after the turn test, while the chair was still sliding sideways. The rebuilt chair tips back
    more easily: the scan-true axle is 4 mm lower, and its centre of mass sits nearer above the axle. Under
    that ramp it hit the anti-tip wheels at ~1 rad/s and bounced over them, which an attendant tilting a
    chair wouldn't do. The old model passes the new test too (9.6°).
  - Tipping over is real physics, not a bug: the handle is 0.2 m behind the anti-tip wheels, so a hard
    enough press, or slamming it back, flips the empty chair.
- **Isaac Lab:** rolls straight down a 20 m hallway at 1.2 m/s, coasts, brakes and resets.

## Isaac Lab (3.0, WSL2)

Isaac Lab is installed at `~/Development/Learning/Robo/IsaacLab` as a uv project, and that project's
environment has Isaac Lab, Newton, the URDF importer and Viser. `uv run` normally uses the environment of
the folder you're in. The two launcher scripts point it at that one instead (`uv run --project`), so you
can run them from anywhere. Set `ISAACLAB=/other/path` if Isaac Lab moves.

```bash
cd ~/Development/Wheelmo/Modeling
bash staxi/make_usd.sh                                   # after every build.py
# an "attendant" pushes it down a hallway; watch in a browser at http://localhost:8080
bash staxi/run_hallway.sh --viz viser
# push it yourself: desktop window, right-click and drag any part of the chair
bash staxi/run_hallway.sh --viz newton_gl --interactive
```

`../Simulations` has its own uv environment, also with Isaac Lab 3.0. It can run the hallway with the Newton
GL window too, but it has neither Viser nor the URDF importer, so conversion still goes through the
checkout above.

Options:
- `--speed 1.5`: walking pace in m/s.
- `--follow`: the camera tracks the chair, but then you can't drag the view.
- `--loops N`: stop after N runs down the hallway.
- `--fast`: don't hold to real time.

**Newton GL window controls:**
- Right-click drag pulls the part under the cursor with a stiff, damped spring: force ≈ (10 + part mass) ×
  50 N/m × drag distance.
- Left-drag looks around; WASD and Q/E move; scroll zooms; F frames the model; H hides the panel.
- "Reset Episode" puts the chair back at the start.

What we learned getting it running:
- **Viewers in WSL2:**
  - The Isaac Sim viewport (`--viz kit`) needs Vulkan, which WSL2's NVIDIA driver doesn't have.
  - **Viser** draws in the Windows browser at 60 fps, but has no dragging.
  - The **Newton GL** window works. The script switches WSL's OpenGL from the CPU to the RTX 4060
    (`GALLIUM_DRIVER=d3d12`), but it still draws only about 15–27 fps. Each frame's part positions go
    GPU → CPU → OpenGL, and every OpenGL call crosses into Windows. The GPU isn't the limit: an empty window
    manages 255 fps. Physics still keeps real time, because several physics frames run per drawn frame.
- **The pointer was invisible in WSLg windows:** WSL had no cursor theme installed. Fixed by unpacking
  Ubuntu's DMZ-White theme into `~/.icons` (no sudo).
- **Physics engine:**
  - The script uses **Newton** (`--physics newton_mjwarp`).
  - In this Isaac Lab checkout (commit 30884d2, 2026-10-14), every PhysX articulation crashes on startup,
    the bundled tutorials included. In `isaaclab_physx/assets/articulation/articulation_data.py`,
    `joint_acc` passes `self.joint_vel` where Newton's version passes `self.joint_vel.warp`.
  - Once that's fixed upstream, `--physics isaacsim_physx` selects PhysX.
- **Newton needs `InteractiveScene`**, because it only builds what's registered there. **The floor must be
  a ground plane**: wheel cylinders on a box floor pick up sideways jitter and steer into a wall within
  about 4 m.
- **Frame rate:** don't let `sim.step()` render every physics step (240 Hz); it floods the viewer. Step with
  `render=False` and render at 60 Hz.
- **Round wheels:** keep wheel cylinders truly round. PyBullet turns URDF cylinders into many-sided hulls
  unless loaded with `URDF_USE_IMPLICIT_CYLINDER`; a loaded chair then needs a shove to start rolling.
- **Small wheels need tiny damping:** the anti-tip wheels need very low joint damping, or they stop between
  bounces and skid. PyBullet also caps joints at 100 rad/s, only 1.9 m/s for a 38 mm wheel, so
  `sim_check.py` lifts the cap for them.
- **Visual meshes only load with a viewer:** Isaac Lab loads them only when a viewer is open; headless runs
  carry just the collision primitives.
- **Friction and damping live only in the script:** the USD doesn't carry the tyre friction or the low
  joint damping (the importer drops the URDF's damping). Loading the USD on its own gives default friction
  and joints. Writing both into the USD is a to-do.

## Known estimates (confirm on the real chair)

- **Tube sizes:** base tubes oval 22 × 40 mm, rear posts, push handle and front downtubes oval 24 × 40 mm
  (from scan sections); footrests 16 mm (5/8"); seat rails and front crossbar 25.4 mm (1"), thin 15.9 mm
  (5/8"), brake bar 19 mm. The seat rails are hidden under the cushion in the scan, so they may be oval too. Scan noise is ±5 mm, so measure with calipers, especially the ovals if a clamp goes on them.
- **Rear wheel / brake:** the shapes are from the scan (±1–2 mm). Estimates: rod lift when squeezed
  (13 mm), the rod's top linkage, spring wire (2 mm) and coil count (9), axle bolt Ø 12 and head, spacer
  Ø 20. For a friction drive, measure: tyre diameter at the crown and its width, tyre hardness, tyre
  to post and tyre to base-tube gaps, the post's oval section where a clamp would go, and what's behind
  the top of the wheel on its inner side (see **Not resolved** above).
- **Armrest "down" position:** joint zero is the scanned pose (pads about 17° nose-up). If the real rest
  position is level, only the joint origin changes.
- **Zip-tied brake bar:** rotated 12.8° about its hinge, leaving 10 mm to the push handle.
- **Part masses:** rear wheels 1.2 kg, caster forks 0.25 kg, caster wheels 0.35 kg, armrests 0.75 kg,
  anti-tip wheels 0.06 kg, cushion 3.5 kg; the frame takes the rest of the 25.9 kg.
- **Anti-tip wheels:** 38 mm diameter (scan), 16 mm wide (estimate).
- **Front casters:** the shapes are from the scan (±2 mm). The axle diameter (8 mm), the rivet heads and
  the 3 mm tyre-to-hood clearance are estimates. For kit parts near the casters, measure with calipers:
  wheel diameter and tread width, wheel-centre width, inside width of the yoke, plate thickness, trail,
  axle diameter, the housing's outside diameter, and the floor-to-housing height.
  - **Swept envelope:** a caster turning about its axis sweeps a radius of 102 mm (trail 37 + wheel radius
    64, widest at axle height) from the floor to the top of the wheel, 128 mm up. Above that, the hood and
    head stay within 52 mm of the axis. Keep kit parts out of that volume.
- **Floor offset:** the scan's fitted floor sits 2–3 mm above the lowest point of every wheel (casters
  3.0 mm, rear wheels 1.3 and 2.3 mm). That's tyre squash or a floor fit that's a little high, so frame
  heights taken from the scan may read 2–3 mm low. The casters keep their measured internal dimensions
  and touch the CAD floor, so their heads sit 3 mm higher than the scanned housing bottom. The rear
  wheels instead keep the scanned axle height (97.5), so the frame-to-axle geometry is 1:1. Their
  unloaded tread circle (r 101) then reaches 3.5 mm below the CAD floor, so in simulation the chair rests
  3.5 mm higher at the back (0.4° nose-down).
- **Rolling resistance:** none modelled beyond the bearings, so the chair coasts almost without slowing.
- **Not modelled:** the label patch on the backrest; the texture colours (all colours are hand-picked).

## Using the model elsewhere

- **`../Simulations`:** import `staxi/out/staxi_wheelchair.urdf` (+ `out/meshes/`), or the USD from
  `make_usd.sh`. The plan is for the Staxi to replace the placeholder power chair there, with the drive kit
  and sensors added. When you do:
  - Copy the passive joint settings and the rubber-tyre material from `isaac_hallway.py`.
  - Follow `wheelmo_sim`'s visual/collision split.
- **Mechanical engineers:** `out/staxi_wheelchair.step` (mm, same frame). It's reconstructed from a phone
  scan, so it's good for layout and fit-checks. Measure anything you'll attach to on the real chair.
- **Git:** the raw scan (188 MB, over GitHub's 100 MB file limit), the cleaned STL, the caches and the
  USD are git-ignored. Everything else, including the generated URDF/meshes/STEP, is small enough to
  commit. Share the scan via Git LFS or a shared drive.

## Next steps

- STEP export as an assembly with named parts, plus separate STEPs for the parts the kit attaches to.
- Write tyre friction and joint damping into the USD.
- Bring the Staxi into `../Simulations`, with drive-kit and occupant placeholders.
- Sensor-placement study: see `../sensor_placement_strategy.md`.
- Optional: sample real colours from the scan texture; measure the known estimates above.
- Rear wheels: confirm or model the plate-like material inside the left wheel, and the rod-to-brake-bar
  linkage at the hinge (see **Rear wheels and parking brake → Not resolved**).
- `sim_check.py` reports both casters at 0° straight after a turn, with the old casters and the new ones
  alike. Check whether they really swivel in PyBullet, or only re-trail before the reading.
