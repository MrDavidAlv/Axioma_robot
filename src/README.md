# Axioma Robot - ROS2 Humble

Robot móvil autónomo con SLAM y navegación Nav2 en Gazebo.

## 📦 Estructura del Proyecto

```
src/
├── axioma_description/      # Modelos URDF/SDF, meshes, worlds, configuraciones RViz
├── axioma_gazebo/           # Paquete de soporte para Gazebo (vacío)
├── axioma_navigation/       # Configuraciones Nav2, SLAM y mapas
└── axioma_bringup/          # 3 Launches principales del sistema
    ├── runmap.launch.py              ✅ SLAM (Mapeo)
    ├── save_map.launch.py            ✅ Guardar mapa
    └── navigation_bringup.launch.py  ✅ Navegación autónoma
```

---

## 🚀 Los 3 Launches Esenciales

### 1️⃣ **SLAM** - Crear mapas nuevos
```bash
ros2 launch axioma_bringup runmap.launch.py
```

**¿Qué hace?**
- Lanza Gazebo con el robot Axioma
- Inicia SLAM Toolbox para mapeo en tiempo real
- Abre RViz con vista de SLAM
- Habilita control con joystick Xbox

**Archivos que usa:**
- `axioma_description/worlds/empty.world`
- `axioma_description/models/axioma_v2/model.sdf`
- `axioma_description/urdf/axioma.urdf`
- `axioma_navigation/config/slam_params.yaml`
- `axioma_description/rviz/slam-toolbox.yaml.rviz`

**Cómo usar:**
1. Ejecuta el launch
2. Mueve el robot con el joystick Xbox (stick izquierdo = adelante/atrás, stick derecho = giro)
3. Observa en RViz cómo se construye el mapa en tiempo real
4. Cuando termines, usa el launch #2 para guardar el mapa

---

### 2️⃣ **Guardar Mapa** - Exportar mapa creado
```bash
ros2 launch axioma_bringup save_map.launch.py
```

**¿Qué hace?**
- Guarda el mapa actual en `axioma_navigation/maps/mapa.yaml` y `mapa.pgm`

**IMPORTANTE:**
- Ejecuta esto **MIENTRAS** `runmap.launch.py` está corriendo
- El mapa se guarda automáticamente en la ubicación configurada

**Alternativa manual:**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ruta/nombre_mapa
```

---

### 3️⃣ **Navegación Autónoma** - Usar mapa guardado
```bash
ros2 launch axioma_bringup navigation_bringup.launch.py
```

**¿Qué hace?**
- Lanza Gazebo con el robot Axioma
- Carga el mapa estático (`mapa.yaml`)
- Inicia AMCL para localización
- Inicia Nav2 Stack completo (controller, planner, behavior server)
- Abre RViz con herramientas de navegación
- Publica transformada estática `map->odom` inicial

**Archivos que usa:**
- `axioma_description/worlds/empty.world`
- `axioma_description/models/axioma_v2/model.sdf`
- `axioma_description/urdf/axioma.urdf`
- `axioma_navigation/config/nav2_params.yaml`
- `axioma_navigation/maps/mapa.yaml`
- `axioma_description/rviz/navigation.yaml.rviz`

**Cómo usar:**
1. Asegúrate de tener un mapa guardado en `axioma_navigation/maps/mapa.yaml`
2. Ejecuta el launch
3. Espera a que todo cargue (Gazebo + Nav2)
4. En RViz:
   - Usa **"2D Pose Estimate"** (botón con flecha verde) para establecer la posición inicial del robot en el mapa
   - Usa **"Nav2 Goal"** (botón con flecha roja) para enviar objetivos de navegación
5. El robot navegará autónomamente evitando obstáculos

**Visualizaciones en RViz:**
- Mapa (gris/blanco = libre, negro = obstáculos)
- Robot (modelo 3D)
- Nube de partículas AMCL (flechitas rojas)
- Pose estimada de AMCL (flecha amarilla)
- Laser scan (puntos blancos)
- Plan global (línea verde)

---

## 📁 Archivos de Configuración

### SLAM
- `axioma_navigation/config/slam_params.yaml` - Configuración de SLAM Toolbox

### Navegación Nav2
- `axioma_navigation/config/nav2_params.yaml` - Todos los parámetros de Nav2
  - AMCL (localización)
  - Controller (seguimiento de trayectorias)
  - Planner (planificación global)
  - Behavior Server (comportamientos de recovery)
  - Costmaps (local y global)

### Visualización RViz
- `axioma_description/rviz/slam-toolbox.yaml.rviz` - Configuración para SLAM
- `axioma_description/rviz/navigation.yaml.rviz` - Configuración para navegación

---

## ⚙️ Compilar el Proyecto

```bash
cd ~/ros2/axioma_humble_ws
colcon build
source install/setup.bash
```

---

## 🎮 Control

### Joystick Xbox (SLAM)
- **Stick izquierdo vertical**: Movimiento adelante/atrás
- **Stick derecho horizontal**: Rotación izquierda/derecha
- **Velocidad lineal máxima**: 0.5 m/s
- **Velocidad angular máxima**: 2.0 rad/s

### RViz (Navegación)
- **2D Pose Estimate**: Establecer posición inicial (obligatorio)
- **Nav2 Goal**: Enviar objetivo de navegación
- **Publish Point**: Marcar puntos en el mapa

---

## 📊 Parámetros Clave de Nav2

### AMCL (Localización)
```yaml
max_particles: 5000          # Más partículas = mejor localización
min_particles: 1000
update_min_d: 0.1            # Actualiza cada 10cm de movimiento
update_min_a: 0.1            # Actualiza cada 0.1 rad de rotación
alpha1-5: 0.05               # Confianza en odometría (menor = más confianza)
recovery_alpha_fast: 0.1     # Recovery automático si se pierde
```

### Controller (DWB Local Planner)
```yaml
max_vel_x: 0.26 m/s          # Velocidad lineal máxima (ajustada para SLAM)
max_vel_theta: 1.0 rad/s     # Velocidad angular máxima (ajustada para navegación)
sim_time: 2.0                # Mira 2 segundos adelante
```

### Global Costmap
```yaml
unknown_cost_value: 0        # SLAM: áreas desconocidas = navegables
                             # Mapa estático: cambiar a 255 para evitar salir del mapa
inflation_radius: 0.8        # Distancia de seguridad a obstáculos
```

---

## 🐛 Solución de Problemas

### El mapa no se ve en RViz (navegación)
- **Causa**: Fixed Frame incorrecto o QoS mismatch
- **Solución**:
  - Fixed Frame debe ser `map`
  - Topic `/map` debe tener Durability = `Transient Local`

### El robot se pierde en navegación
- **Causa**: AMCL tiene pocas partículas o pose inicial incorrecta
- **Solución**:
  - Usa "2D Pose Estimate" para reinicializar
  - Aumenta `max_particles` en `nav2_params.yaml`
  - Reduce `alpha` values para confiar más en odometría

### El robot sale del área mapeada
- **Causa**: `allow_unknown: true` en planner
- **Solución**: Cambiar `unknown_cost_value: 255` en global_costmap (tratar desconocido como obstáculo)

### SLAM no actualiza el mapa
- **Causa**: Robot no se mueve o laser no detecta cambios
- **Solución**: Mueve el robot para que vea áreas nuevas

### Nav2 dice "No valid path"
- **Causa**: Objetivo fuera del mapa o bloqueado
- **Solución**:
  - Coloca el objetivo dentro del área mapeada
  - Verifica que no haya obstáculos bloqueando el camino

---

## 📝 Flujo de Trabajo Típico

1. **Crear un mapa nuevo:**
   ```bash
   ros2 launch axioma_bringup runmap.launch.py
   # Mueve el robot con el joystick explorando el entorno
   # En otra terminal:
   ros2 launch axioma_bringup save_map.launch.py
   ```

2. **Navegar con el mapa:**
   ```bash
   ros2 launch axioma_bringup navigation_bringup.launch.py
   # En RViz: "2D Pose Estimate" → "Nav2 Goal"
   ```

---

## 🔧 Parámetros Importantes

- **use_sim_time: true** - Siempre activado para simulación Gazebo
- **robot_radius: 0.15m** - Tamaño del robot para costmaps
- **resolution: 0.05m** - Resolución de los mapas (5cm por píxel)

---

## 📖 Referencias

- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Gazebo ROS](http://gazebosim.org/tutorials?cat=connect_ros)
