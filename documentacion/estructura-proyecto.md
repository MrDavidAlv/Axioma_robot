# 📁 Estructura del Proyecto

```
Axioma_robot/
├── src/
│   ├── axioma_bringup/            # Orquestadores de lanzamiento de alto nivel
│   │   ├── config/
│   │   │   └── ydlidar_x3_pro.yaml    # Parámetros del driver para el LiDAR real
│   │   └── launch/
│   │       ├── slam_bringup.launch.py
│   │       ├── navigation_bringup.launch.py
│   │       └── lidar.launch.py        # YDLIDAR X3 PRO físico
│   │
│   ├── axioma_description/        # Modelo URDF, mallas, configs de RViz
│   │   ├── urdf/
│   │   │   ├── axioma.urdf.xacro      # Descripción del robot (xacro, no URDF plano)
│   │   │   └── zed2i_macro.urdf.xacro # Árbol de frames de la ZED 2i (de Stereolabs)
│   │   ├── meshes/
│   │   ├── rviz/
│   │   └── launch/
│   │
│   ├── axioma_gazebo/             # Simulación en Ignition Gazebo (Fortress)
│   │   ├── axioma_gazebo/
│   │   │   └── odom_to_tf.py      # Odometría a TF
│   │   ├── models/axioma_v2/      # Modelo SDF con mallas
│   │   ├── scripts/
│   │   │   ├── generate_office_world.py    # Genera el mundo y el mapa de Nav2 juntos
│   │   │   └── generate_terrain_world.py   # Ídem para el mundo de nave + patio
│   │   ├── rviz/
│   │   │   └── terrain_demo.rviz  # Pose fusionada, nube ZED, LiDAR y TF
│   │   ├── worlds/
│   │   │   ├── office.world       # Recepción, 2 oficinas, sala de reuniones
│   │   │   ├── terrain.world      # Nave, 2 rampas de 8.21° y patio asfalto/tierra/arena
│   │   │   └── empty.world
│   │   └── launch/
│   │       ├── simulation.launch.py
│   │       └── terrain_demo.launch.py   # Gazebo + RViz + GUI de teleoperación
│   │
│   ├── axioma_perception/         # Localización consciente de la actitud (EKF)
│   │   ├── config/
│   │   │   ├── ekf_sim.yaml       # Simulación: IMU + odometría de ruedas
│   │   │   └── ekf_zed.yaml       # Robot real: IMU + tracking de la ZED 2i
│   │   └── launch/
│   │       ├── ekf.launch.py      # robot_localization; publica odom→base_footprint
│   │       └── zed2i.launch.py    # Cámara ZED 2i física (necesita el SDK de ZED)
│   │
│   ├── axioma_slam/               # Configuración de SLAM Toolbox
│   │   ├── config/
│   │   │   └── slam_params.yaml
│   │   ├── scripts/
│   │   │   └── score_map.py       # Compara un mapa de SLAM contra el plano exacto
│   │   ├── rviz/
│   │   └── launch/
│   │       ├── slam.launch.py
│   │       └── save_map.launch.py
│   │
│   ├── axioma_navigation/         # Configuración de Nav2 y mapas
│   │   ├── config/
│   │   │   └── nav2_params.yaml
│   │   ├── maps/
│   │   ├── rviz/
│   │   └── launch/
│   │       └── navigation.launch.py
│   │
│   └── axioma_teleop_gui/         # Interfaz de teleoperación en PyQt5
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
├── documentacion/
│   ├── modelo-matematico/         # Cinemática, control y parámetros físicos
│   ├── guia-instalacion.md
│   ├── guia-uso.md
│   ├── validacion-slam.md
│   ├── historial-proyecto.md
│   └── estructura-proyecto.md     # este documento
│
├── images/                        # Imágenes de la documentación
└── README.md
```

---

Vuelve al [README](../README.md).
