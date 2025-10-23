# Mapas Disponibles para Navigation2

## Mapas Copiados del Proyecto Original

Se han copiado 3 mapas del proyecto Axioma_robot original:

### 1. my_world (Mapa de Simulación Simple)
- **Archivos:**
  - `my_world.yaml`
  - `my_world2.pgm`
- **Descripción:** Mapa simple para pruebas de simulación
- **Resolución:** 0.01 m/píxel (1 cm)
- **Origen:** [-0.5, -0.5, 0.0]
- **Tamaño:** 1.2 MB

**Uso:**
```bash
ros2 launch axioma_navigation navigation.launch.py \
  map:=$HOME/ros2/axioma_humble_ws/src/axioma_navigation/maps/my_world.yaml
```

---

### 2. SlamToolboxSimulation (Mapa SLAM en Simulación)
- **Archivos:**
  - `SlamToolboxSimulation.yaml`
  - `SlamToolboxSimulation.pgm`
- **Descripción:** Mapa creado con SLAM Toolbox en simulación
- **Resolución:** 0.05 m/píxel (5 cm)
- **Origen:** [-10.0, -10.0, 0.0]
- **Tamaño:** 1.2 MB

**Uso:**
```bash
ros2 launch axioma_navigation navigation.launch.py \
  map:=$HOME/ros2/axioma_humble_ws/src/axioma_navigation/maps/SlamToolboxSimulation.yaml
```

---

### 3. MapSAMSUNG_Cartographer (Mapa Real con Cartographer)
- **Archivos:**
  - `MapSAMSUNG_Cartographer.yaml`
  - `MapSAMSUNG_Cartographer.pgm`
- **Descripción:** Mapa real creado con Cartographer
- **Resolución:** 0.05 m/píxel (5 cm)
- **Origen:** [-7.34, -30.5, 0.0]
- **Modo:** Trinary
- **Tamaño:** 1.2 MB

**Uso:**
```bash
ros2 launch axioma_navigation navigation.launch.py \
  map:=$HOME/ros2/axioma_humble_ws/src/axioma_navigation/maps/MapSAMSUNG_Cartographer.yaml
```

---

## Mapa Recomendado para Empezar

**Para simulación en Gazebo:**
```bash
SlamToolboxSimulation.yaml
```

Este mapa es ideal porque:
- Fue creado en simulación similar a tu entorno
- Tiene buena resolución (5cm)
- Es compatible con SLAM Toolbox

---

## Cómo Usar un Mapa

### Opción 1: Con el path completo

```bash
cd ~/ros2/axioma_humble_ws
source install/setup.bash
ros2 launch axioma_navigation navigation.launch.py \
  map:=$HOME/ros2/axioma_humble_ws/src/axioma_navigation/maps/SlamToolboxSimulation.yaml
```

### Opción 2: Con path relativo (desde el workspace)

```bash
ros2 launch axioma_navigation navigation.launch.py \
  map:=src/axioma_navigation/maps/my_world.yaml
```

---

## Crear Tu Propio Mapa

Para crear un mapa personalizado con SLAM:

```bash
# Terminal 1: Gazebo
ros2 launch axioma_gazebo complete_simulation.launch.py

# Terminal 2: SLAM
ros2 launch axioma_navigation slam.launch.py

# Terminal 3: Control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Cuando termines de mapear:
ros2 run nav2_map_server map_saver_cli -f src/axioma_navigation/maps/mi_nuevo_mapa
```

Esto creará:
- `mi_nuevo_mapa.pgm` (imagen del mapa)
- `mi_nuevo_mapa.yaml` (configuración)

---

## Verificar un Mapa

Para visualizar un mapa sin Navigation2:

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=src/axioma_navigation/maps/my_world.yaml
```

En otra terminal:
```bash
ros2 run rviz2 rviz2
```

En RViz:
- Add → Map
- Topic: /map

---

## Estructura de un Archivo YAML de Mapa

```yaml
image: nombre_del_mapa.pgm      # Archivo de imagen
resolution: 0.050000             # Resolución en metros/píxel
origin: [-10.0, -10.0, 0.0]     # Origen del mapa [x, y, yaw]
negate: 0                        # Invertir colores (0=no, 1=si)
occupied_thresh: 0.65            # Umbral para celda ocupada
free_thresh: 0.196               # Umbral para celda libre
mode: trinary                    # Modo (opcional): trinary, scale, raw
```

---

## Troubleshooting

### Error: "Could not load map"

Verificar:
1. El path al archivo YAML es correcto y absoluto
2. El archivo PGM existe en el mismo directorio
3. El nombre del PGM en el YAML coincide con el archivo

### El mapa no se ve en RViz

1. Verifica que Fixed Frame = "map"
2. Agrega display tipo "Map"
3. Topic debe ser "/map"
4. Revisa que nav2 esté publicando: `ros2 topic echo /map --once`

### El robot aparece fuera del mapa

1. Usa "2D Pose Estimate" en RViz
2. Click en la posición real del robot en el mapa
3. Arrastra para indicar la orientación

---

## Lista de Mapas Disponibles

```bash
ls -lh ~/ros2/axioma_humble_ws/src/axioma_navigation/maps/
```

Deberías ver:
- my_world.yaml + my_world2.pgm
- SlamToolboxSimulation.yaml + SlamToolboxSimulation.pgm
- MapSAMSUNG_Cartographer.yaml + MapSAMSUNG_Cartographer.pgm

---

## Próximos Pasos

1. **Probar un mapa existente:**
   ```bash
   ros2 launch axioma_navigation navigation.launch.py \
     map:=$HOME/ros2/axioma_humble_ws/src/axioma_navigation/maps/SlamToolboxSimulation.yaml
   ```

2. **Crear tu propio mapa** con SLAM

3. **Navegar** usando RViz con "2D Pose Estimate" y "Nav2 Goal"
