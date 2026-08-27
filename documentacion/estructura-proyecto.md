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
│   │   ├── meshes/
│   │   ├── rviz/
│   │   └── launch/
│   │
│   ├── axioma_gazebo/             # Simulación en Ignition Gazebo (Fortress)
│   │   ├── axioma_gazebo/
│   │   │   └── odom_to_tf.py      # Odometría a TF
│   │   ├── models/axioma_v2/      # Modelo SDF con mallas
│   │   ├── scripts/
│   │   │   └── generate_office_world.py   # Genera el mundo y el mapa de Nav2 juntos
│   │   ├── worlds/
│   │   │   ├── office.world       # Recepción, 2 oficinas, sala de reuniones
│   │   │   └── empty.world
│   │   └── launch/
│   │       └── simulation.launch.py
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
