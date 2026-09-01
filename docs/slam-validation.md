# SLAM Validation

The README shows the headline result (the map scored against the exact floor
plan). Here is the rest of the evidence: how it is measured, and why raw
odometry is not enough.

---

## One source of truth for the world and its floor plan

`src/axioma_gazebo/scripts/generate_office_world.py` emits **both** the Gazebo
world and an exact occupancy grid of that same world, from the same geometry
list, so the two cannot drift apart:

```bash
python3 src/axioma_gazebo/scripts/generate_office_world.py
# -> src/axioma_gazebo/worlds/office.world
# -> src/axioma_navigation/maps/ground_truth.{pgm,yaml}
```

Only geometry taller than the LiDAR plane (0.15 m) is rasterised as an
obstacle, and free space is flood-filled from the spawn point, so everything
outside the building stays unknown — the same structure real mapping produces.

`ground_truth` is **not** the map Nav2 navigates with. That one
(`maps/mapa.yaml`) comes from driving the robot with SLAM Toolbox running, just
as on the real robot. The exact floor plan exists so the quality of the SLAM
map can be **measured** rather than merely eyeballed.

## How it is scored

`src/axioma_slam/scripts/score_map.py` compares the SLAM map against that exact
floor plan and reports two numbers: how far each occupied cell of the SLAM map
is from the nearest real obstacle (how much of the map is invented), and how
many real obstacles inside explored territory the map actually marks.

```bash
python3 src/axioma_slam/scripts/score_map.py --saved out.png "run title"
```

## Odometry drift vs. SLAM

Raw wheel odometry drifts to **3.79 m** over the run, while SLAM Toolbox stays
within **0.14 m** of the Gazebo ground-truth pose throughout, with no jumps.

<div align="center">
<img src="../images/slam-telemetry.png" width="960"/>
</div>

## Localisation check

Live laser returns (green) projected onto the SLAM map and onto the inflated
global costmap, with AMCL's pose in red. They land on the walls, the reception
desk and the columns, confirming that the world, the map and the pose estimate
all agree with each other.

<div align="center">
<img src="../images/costmap-validation.png" width="960"/>
</div>

---

Back to the [README](../README.md).
