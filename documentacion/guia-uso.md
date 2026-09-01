# 🚀 Usage Guide

Complete reference for operating Axioma, in simulation and on the physical
robot. The README covers the essentials in its Quick Start; the rest is here.

---

## SLAM (mapping)

Launch the simulation with SLAM Toolbox and RViz:

```bash
ros2 launch axioma_bringup slam_bringup.launch.py
```

In another terminal, drive the robot to explore the environment:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Or with the teleoperation GUI:

```bash
ros2 launch axioma_teleop_gui teleop_gui.launch.py
```

Save the map once the environment is fully explored:

```bash
ros2 launch axioma_slam save_map.launch.py
```

With the workspace built using `--symlink-install` this writes straight into
`src/axioma_navigation/maps/mapa.{pgm,yaml}`, so the map ends up under version
control instead of being lost on the next build. Check how good it came out
with:

```bash
python3 src/axioma_slam/scripts/score_map.py
```

## VSLAM (RGB-D visual mapping)

3D mapping with the ZED 2i through RTAB-Map. Unlike SLAM Toolbox, which
projects everything onto a 2D grid, the pose graph here is left free in six
degrees of freedom — which is what the terrain world needs, since the warehouse
floor and the yard platform 28 cm above it fall on the same cells in 2D.

```bash
ros2 launch axioma_bringup vslam_bringup.launch.py
```

That opens Gazebo, RViz, the teleop GUI and RTAB-Map. You have to drive: a
stationary camera adds no nodes to the pose graph and closes no loops.

| Argument | Values | What it does |
|---|---|---|
| `world` | `terrain` (default), `office` | Which world to map |
| `odom_source` | `visual` (default), `ekf` | Who estimates the motion and, with it, who publishes `odom -> base_footprint` |
| `rtabmap_viz` | `false` (default), `true` | RTAB-Map's own window with the feature matches |

Outputs: `/cloud_map` (assembled 3D cloud), `/map` (Nav2-ready occupancy grid)
and the database at `~/.ros/axioma_vslam.db`.

To relocalise against an existing database instead of mapping again:

```bash
ros2 launch axioma_slam vslam.launch.py localization:=true
```

`vslam.launch.py` is agnostic of simulation or real robot. On the physical
robot the ZED wrapper publishes the same three topics under `/zed2i/zed_node/`,
so override the arguments rather than editing the file:

```bash
ros2 launch axioma_slam vslam.launch.py \
    use_sim_time:=false \
    rgb_topic:=/zed2i/zed_node/rgb/image_rect_color \
    depth_topic:=/zed2i/zed_node/depth/depth_registered \
    camera_info_topic:=/zed2i/zed_node/rgb/camera_info
```

### On visual odometry drift

With `odom_source:=visual` the camera is the only source of motion. Measured on
the terrain world: 3.4% short over a 2.62 m straight run, and roughly 15% off
after two 114-degree turns. If what you want is a metrically faithful map,
`odom_source:=ekf` leaves the mapping to the camera and the pose to the filter
that already fuses IMU and wheels.

## Autonomous navigation

Launch the simulation with Nav2 and RViz (requires a previously saved map):

```bash
ros2 launch axioma_bringup navigation_bringup.launch.py
```

In RViz2:

1. **2D Pose Estimate** to set the robot's initial pose
2. **2D Goal Pose** to send a navigation goal
3. Watch the global/local costmaps, the planned trajectories and AMCL's
   particle cloud

## Physical LiDAR

On the real robot, start the YDLIDAR X3 PRO with the profile this project
ships:

```bash
ros2 launch axioma_bringup lidar.launch.py            # /dev/ttyUSB0
ros2 launch axioma_bringup lidar.launch.py port:=/dev/ttyUSB1
```

`ydlidar_ros2_driver`'s own launch defaults to `params/ydlidar.yaml`, which is
an X4 profile: 128000 baud, dual channel, 9 kHz. Starting an X3 PRO with that
profile makes the driver wait for a device info reply the hardware never sends,
and it logs `Failed to get scan` indefinitely. The profile this project ships
also stamps the scans with `base_scan`, the frame the URDF, AMCL, both costmaps
and SLAM Toolbox all look for; the stock `laser_frame` breaks every TF lookup
on the real robot.

## Useful commands

```bash
# Monitoring
ros2 node list                           # Active nodes
ros2 topic list                          # Active topics
ros2 topic hz /scan                      # LiDAR rate
ros2 topic echo /cmd_vel                 # Velocity commands
ros2 run tf2_ros tf2_echo map base_link  # TF query
ros2 run tf2_tools view_frames           # TF tree diagram

# Debugging
ros2 node info /slam_toolbox
ros2 param list /controller_server
ros2 bag record -a -o navigation_data
```

---

Back to the [README](../README.md).
