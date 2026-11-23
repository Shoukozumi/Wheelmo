# Sensor placement strategy for the Wheelmo kit on the Staxi

How we plan to choose where the LiDAR and cameras go on the Staxi, using simulation before drilling or
clamping anything on the real chair. Started 2026-11-23. Status: plan, nothing built yet.

## Context and constraints

- **Platform:** the standard Staxi medical wheelchair (non-folding, stacks), modelled from a phone scan as
  parametric CAD. Wheelmo is an **add-on autonomy kit**: drive kit + sensors bolted or clamped onto the
  existing Staxi. The Staxi frame itself is not modified.
- **Sensors:**
  - 1 × **Livox Mid-360** LiDAR. Mount not decided.
  - 2 × 1080p RGB cameras. Cheap Arducams for now; the final model and lens are to be decided.
- **Sim code:** `Simulations/` (`wheelmo_sim`). Isaac Lab 3.0 without the Isaac Sim app: Newton physics,
  Warp ray-cast LiDAR, Newton Warp camera renderer. All of it runs in WSL2. Its current chair is a placeholder
  (a generic power chair) and will be replaced by the Staxi.
- **Staxi CAD:** `Modeling/staxi/` in this repo (overview: `Modeling/README.md`).
  - `params.py`: every dimension and tube centreline.
  - `out/`: STEP, URDF, meshes and the Isaac USD.

  Frame: X forward, Y left, Z up, mm in CAD / m in URDF. Origin on the floor, midway between the rear wheels,
  under the axle.

### Staxi landmarks that matter for mounting (from the CAD)

| Feature | Position (approx.) |
|---|---|
| Overall width | 0.69 m (rear wheels) |
| Push-handle crossbar (behind backrest) | x = −0.27 m, z = 1.00 m |
| Backrest top | z ≈ 0.96 m |
| Seat top | z ≈ 0.55 m |
| Armrest pads (flip up 110° on a rear hinge) | z ≈ 0.69–0.77 m |
| Front frame corners | x = 0.64 m, y = ±0.17 m, z = 0.20 m |
| Footrest loops | x ≈ 0.66–0.71 m, z ≈ 0.13–0.16 m |
| Anti-tip wheels (rear tips of base tubes) | x = −0.22 m, z = 0.055 m |
| Seated occupant head top (estimate) | z ≈ 1.4–1.5 m |

Moving or in-use parts a mount must not block: the **flip-up armrests**, the **push handle**, the **brake
squeeze bar** (zip-tied in our model), and possibly **stacking** (see open questions).

### Livox Mid-360 facts that drive the design (check against the datasheet)

- FOV: **360° horizontal × −7° to +52° vertical**, so it looks mostly *up*.
- Non-repetitive scan pattern: each frame is sparse, and coverage fills in over time.
- About 200k points/s, so about 20k points per 10 Hz frame.
- Minimum range about 0.1 m.
- Built-in IMU.
- Isaac Lab has **no Livox pattern** (it ships only grid, pinhole, Bpearl and generic spinning LiDAR), so we
  need a custom ray pattern. Start with random directions inside the FOV, about 20k per frame, re-drawn each
  frame. Swap in Livox's real pattern if we can get it.

How far from the sensor the floor and a low obstacle first become visible, from the FOV alone (flat floor, no
occlusion):

| Mount | Floor first visible at | Top of a 0.45 m bench first visible at |
|---|---|---|
| Upright, 1.5 m mast | 12.2 m | 8.6 m |
| Upside-down, 1.5 m mast (−52°..+7°) | 1.2 m | 0.8 m |
| Upright, low at 0.35 m | 2.9 m | always visible |

An upright Mid-360 mounted high is nearly blind near the floor. The mount orientation matters as much as the
location.

## Method: screen fast, then validate by driving

### Stage 1: geometry screening (no physics, thousands of candidates)

1. **Candidate mounts.**
   - Sample points every few cm along every Staxi tube (centrelines in `params.py`).
   - At each point, try short bracket offsets (0–15 cm) and, separately, masts of a few heights.
   - Orientations: Mid-360 upright / upside-down / tilted (0–45° in steps); cameras yaw and pitch on a grid.
   - Camera pairs: front + rear, two forward-facing (stereo-like), front + side.
2. **Scene.**
   - The Staxi (visual meshes).
   - A **seated occupant** in three sizes: small woman, average, large man. A capsule and box mannequin is
     enough for occlusion.
   - A **drive kit / battery placeholder** box, until its design is known.
3. **Ray casting** on the GPU with Warp (already in the `Simulations` environment):
   - Mid-360 pattern accumulated over 0.1 s (one frame) and over 1 s.
   - Camera frusta at the Arducam FOV plus a few alternative lenses (e.g. 70°, 100°, 120° horizontal).
4. **Test targets** placed around the chair: floor grid, low obstacles (0.15 m, 0.45 m), people (legs and
   torso), overhangs (table edge 0.75 m, shelf 1.2 m), drop-offs (stair edge, curb).
5. **Output:**
   - A score for each candidate and a ranked shortlist.
   - Heatmaps painted onto the chair (good and bad mount zones).
   - Polar coverage plots per sensor.

### Stage 2: closed-loop validation in `wheelmo_sim` (top 5–10 setups)

- Replace the placeholder chair with the Staxi plus drive kit, and add the occupant geometry.
- Scenario suite: cluttered corridor, doorway (0.8–0.9 m), people crossing, low obstacles, overhanging table,
  ramp (±6°), drop-off. Add outdoor items if needed (see open questions).
- Run the planner in the loop, many environments in parallel, headless.
- Measure: success rate, collisions, minimum clearance, time from an obstacle becoming visible to it being
  detected, false stops.

### Stage 3: real-world check

- Mount the best setup on a real Staxi.
- Record Mid-360 and camera data around known targets: boxes at measured distances, a stair edge, a person.
- Compare point counts and blind zones with the sim, and correct the sim (e.g. reflectivity, dropouts).

## Metrics

**LiDAR**
- **Floor blind zone:** distance from the chair's footprint edge to the first floor return, per direction
  (front / left / right / rear). Safety-critical for low obstacles and drop-offs.
- **Detection range by obstacle class:** low, person, overhang, drop-off.
- **Self-occlusion:** share of rays hitting the chair or occupant, and which azimuth sectors go blind.
- **Point density** on a person at 3 / 5 / 10 m.
- **Robustness:** spread of the scores across occupant sizes and ±6° chair pitch.

**Cameras**
- Nearest visible floor point in front of the footrests.
- Occlusion by the occupant's feet and legs and by the footrests.
- Drop-off visibility.
- Overlap with LiDAR coverage, for fusion.

**Hard constraints** (filters, not scores)
- Clamps onto an existing tube; no frame modification.
- Doesn't block the armrest flip, push handle or brake bar.
- No wider than the chair (0.69 m).
- Limits on mast height and wobble.
- Stacking, if required.

**Practical scores (tie-breakers)**
- Bracket length and stiffness (vibration).
- Cable run to the compute box.
- Exposure to knocks (doorframes, hands).
- Occupant comfort, and how much the kit gets in the way.

## Starting hypotheses for the Mid-360 (to be confirmed or killed by Stage 1)

| Family | Pros | Cons |
|---|---|---|
| Mast on the push-handle crossbar, above head (~1.6 m), **upside-down** | All-round view; floor from ~1.2 m | Occupant's head and shoulders block a forward cone; mast wobble; blocks stacking |
| **Low front**, near the footrest beams / front crossbar (~0.25 m), upright | Sees low obstacles and people ahead; looks up at overhangs | Chair and occupant's legs block the rear half; exposed to kicks and bumps |
| **Side**, on a rear post or armrest frame (~0.8 m) | Short, stiff bracket | Lopsided coverage; armrest-frame mounts move when the armrest flips |

Cameras: forward coverage of the floor ahead and of drop-offs probably matters most. Low front mounts see the
floor close in but get blocked by feet. High mounts see over the feet but leave a near-field gap.

## Open questions

1. **Stacking:** must a kitted chair still stack with other Staxis? If yes, most masts and rear protrusions
   are ruled out.
2. **Drive kit:** what form (hub motors in the rear wheels, a clip-on drive unit behind, something else), and
   where does the battery go? It takes up space and blocks some views.
3. **Operating environment:** hospital and airport corridors only, or outdoors too (curbs, ramps, uneven
   ground)? This sets the scenario suite and how much drop-off detection matters.
4. **Cameras:** final model and lens (FOV, rolling vs global shutter); what are they for (obstacle detection,
   drop-offs, people, signage)?
5. **Occupant:** which body-size range must we support?

## Implementation notes

- **Where the tooling goes:** Stage 1 lives in `wheelmo_sim`, reusing the Staxi CAD in `Modeling/staxi/`. Tube centrelines come from
  `params.py`; visual meshes and the USD come from `out/`.
- **Rebuilding the Staxi files:** after any CAD change, run `build.py` then `make_usd.sh` in `Modeling/staxi/`.
- **Sim lessons from `wheelmo_sim`:**
  - The LiDAR also hits visual geometry, so keep every sensor clear of its own bracket and of chair parts in
    its FOV.
  - Newton hides colliders on bodies that also have visuals.
- **Rendering:** in WSL, cameras use the Newton Warp renderer, which is fast with simple lighting. Fine for
  coverage and geometry. Photoreal rendering (OVRTX) is untested on WSL2; we'll need it if we train vision
  models on sim images.
- **GPU memory (8 GB):** two 1080p cameras per environment limit parallel runs. Render smaller images for RL
  and keep 1080p for checking.

## Next steps

1. Staxi + seated occupant + drive-kit placeholder as a ray-casting scene.
2. Mid-360 pattern (random within FOV, 20k rays per frame), plus camera frusta.
3. Candidate mount generator from the tube centrelines.
4. Stage 1 metrics, heatmaps and shortlist.
5. Swap the Staxi into `wheelmo_sim` as the robot (drive kit on the rear wheels as a placeholder), then Stage 2.

## Results log

(Fill in as studies run: date, candidate set, top configurations, key numbers, decision.)
