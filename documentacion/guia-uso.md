# 🚀 Guía de Uso

Referencia completa de cómo operar el Axioma, en simulación y en el robot
físico. El README cubre lo esencial en su Quick Start; aquí está el resto.

---

## SLAM (mapeo)

Lanza la simulación con SLAM Toolbox y RViz:

```bash
ros2 launch axioma_bringup slam_bringup.launch.py
```

En otra terminal, conduce el robot para explorar el entorno:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

O con la interfaz gráfica de teleoperación:

```bash
ros2 launch axioma_teleop_gui teleop_gui.launch.py
```

Guarda el mapa cuando el entorno esté completamente explorado:

```bash
ros2 launch axioma_slam save_map.launch.py
```

Con el workspace compilado con `--symlink-install` esto escribe directo en
`src/axioma_navigation/maps/mapa.{pgm,yaml}`, así que el mapa queda bajo
control de versiones en vez de perderse en la siguiente compilación. Revisa
qué tan bien quedó con:

```bash
python3 src/axioma_slam/scripts/score_map.py
```

## Navegación autónoma

Lanza la simulación con Nav2 y RViz (requiere un mapa ya guardado):

```bash
ros2 launch axioma_bringup navigation_bringup.launch.py
```

En RViz2:

1. **2D Pose Estimate** para fijar la pose inicial del robot
2. **2D Goal Pose** para enviar un objetivo de navegación
3. Observa los costmaps global/local, las trayectorias planeadas y la nube de
   partículas de AMCL

## LiDAR físico

En el robot real, arranca el YDLIDAR X3 PRO con el perfil que trae el
proyecto:

```bash
ros2 launch axioma_bringup lidar.launch.py            # /dev/ttyUSB0
ros2 launch axioma_bringup lidar.launch.py port:=/dev/ttyUSB1
```

El launch propio de `ydlidar_ros2_driver` usa por defecto
`params/ydlidar.yaml`, que es un perfil de X4: 128000 baudios, doble canal,
9 kHz. Arrancar un X3 PRO con ese perfil hace que el driver espere una
respuesta de info del dispositivo que el hardware nunca envía, y registra
`Failed to get scan` indefinidamente. El perfil que trae este proyecto también
estampa los escaneos con `base_scan`, el frame que buscan el URDF, AMCL, los
dos costmaps y SLAM Toolbox; el `laser_frame` de fábrica rompe cada búsqueda de
TF en el robot real.

## Comandos útiles

```bash
# Monitoreo
ros2 node list                           # Nodos activos
ros2 topic list                          # Tópicos activos
ros2 topic hz /scan                      # Frecuencia del LiDAR
ros2 topic echo /cmd_vel                 # Comandos de velocidad
ros2 run tf2_ros tf2_echo map base_link  # Consulta de TF
ros2 run tf2_tools view_frames           # Diagrama del árbol de TF

# Depuración
ros2 node info /slam_toolbox
ros2 param list /controller_server
ros2 bag record -a -o navigation_data
```

---

Vuelve al [README](../README.md).
