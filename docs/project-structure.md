# Project Structure

```
Axioma_robot/
├── src/
│   ├── axioma_bringup/            # High-level launch orchestrators
│   │   ├── config/
│   │   │   └── ydlidar_x3_pro.yaml    # Driver parameters for the physical LiDAR
│   │   └── launch/
│   │       ├── slam_bringup.launch.py
│   │       ├── navigation_bringup.launch.py
│   │       ├── vslam_bringup.launch.py   # Simulation + RTAB-Map + RViz + teleop
│   │       └── lidar.launch.py        # Physical YDLIDAR X3 PRO
│   │
│   ├── axioma_description/        # URDF model, meshes, RViz configs
│   │   ├── urdf/
│   │   │   ├── axioma.urdf.xacro      # Robot description (xacro, not plain URDF)
│   │   │   └── zed2i_macro.urdf.xacro # ZED 2i frame tree (from Stereolabs)
│   │   ├── meshes/
│   │   ├── rviz/
│   │   └── launch/
│   │
│   ├── axioma_gazebo/             # Ignition Gazebo (Fortress) simulation
│   │   ├── axioma_gazebo/
│   │   │   └── odom_to_tf.py      # Odometry to TF
│   │   ├── models/axioma_v2/      # SDF model with meshes
│   │   ├── scripts/
│   │   │   ├── generate_office_world.py    # Emits the world and the Nav2 map together
│   │   │   └── generate_terrain_world.py   # Same, for the warehouse + yard world
│   │   ├── rviz/
│   │   │   └── terrain_demo.rviz  # Fused pose, ZED cloud, LiDAR and TF
│   │   ├── worlds/
│   │   │   ├── office.world       # Reception, 2 offices, meeting room
│   │   │   ├── terrain.world      # Warehouse, 2 ramps at 8.21° and asphalt/dirt/sand yard
│   │   │   └── empty.world
│   │   └── launch/
│   │       ├── simulation.launch.py
│   │       └── terrain_demo.launch.py   # Gazebo + RViz + teleop GUI
│   │
│   ├── axioma_perception/         # Attitude-aware localisation (EKF)
│   │   ├── config/
│   │   │   ├── ekf_sim.yaml       # Simulation: IMU + wheel odometry
│   │   │   └── ekf_zed.yaml       # Real robot: IMU + ZED 2i tracking
│   │   └── launch/
│   │       ├── ekf.launch.py      # robot_localization; publishes odom→base_footprint
│   │       └── zed2i.launch.py    # Physical ZED 2i camera (needs the ZED SDK)
│   │
│   ├── axioma_slam/               # SLAM Toolbox and RTAB-Map configuration
│   │   ├── config/
│   │   │   ├── slam_params.yaml
│   │   │   └── vslam_params.yaml  # RGB-D visual SLAM, tuned for a low-texture scene
│   │   ├── scripts/
│   │   │   └── score_map.py       # Scores a SLAM map against the exact floor plan
│   │   ├── rviz/
│   │   │   ├── slam.rviz
│   │   │   └── vslam.rviz         # RTAB-Map cloud and grid, visual odometry, EKF
│   │   └── launch/
│   │       ├── slam.launch.py
│   │       ├── vslam.launch.py    # rgbd_odometry + rtabmap; sim or real robot
│   │       └── save_map.launch.py
│   │
│   ├── axioma_navigation/         # Nav2 configuration and maps
│   │   ├── config/
│   │   │   └── nav2_params.yaml
│   │   ├── maps/
│   │   ├── rviz/
│   │   └── launch/
│   │       └── navigation.launch.py
│   │
│   └── axioma_teleop_gui/         # PyQt5 teleoperation interface
│       ├── axioma_teleop_gui/
│       │   ├── main.py
│       │   ├── main_window.py
│       │   ├── ros_node.py
│       │   └── widgets/
│       │       ├── keyboard_mode.py
│       │       ├── joystick_mode.py
│       │       └── slider_mode.py
│       └── launch/
│           └── teleop_gui.launch.py
│
├── docs/
│   ├── mathematical-model/         # Kinematics, control and physical parameters
│   ├── installation-guide.md
│   ├── usage-guide.md
│   ├── slam-validation.md
│   ├── project-history.md
│   └── project-structure.md     # this document
│
├── images/                        # Documentation images
└── README.md
```

---

Back to the [README](../README.md).
