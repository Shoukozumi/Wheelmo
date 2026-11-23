# Wheelmo

Autonomy kit that bolts onto standard Staxi wheelchairs: drive kit, Livox Mid-360 LiDAR, two cameras.

| Folder / file | What |
|---|---|
| `Modeling/` | Staxi wheelchair CAD and simulation model, rebuilt from a phone scan (URDF / USD / STEP). See `Modeling/README.md` |
| `Simulations/` | Isaac Lab 3.0 simulation (`wheelmo_sim`): corridor scenes, sensors, baseline planner. Will import the Staxi from `Modeling/` |
| `sensor_placement_strategy.md` | How we'll choose LiDAR and camera mounts on the Staxi using simulation |
