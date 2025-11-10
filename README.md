# 🤖 Robot Autónomo Axioma 4WD

<div align="center">
<img src="images/portada.png" width="300"/>
</div>

[![Lenguaje C++](https://img.shields.io/badge/C++-17-blue)](#)
[![Lenguaje Python](https://img.shields.io/badge/Python-3.8+-yellow?logo=python)](#)
[![Sistema Operativo](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)](#)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros)](#)
[![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic%2011-orange?logo=gazebo)](#)
[![CMake](https://img.shields.io/badge/CMake-3.16+-064F8C?logo=cmake)](#)
[![Colcon](https://img.shields.io/badge/Build-Colcon-22314E)](#)
[![Nav2](https://img.shields.io/badge/Nav2-Humble-00599C)](#)
[![SLAM Toolbox](https://img.shields.io/badge/SLAM-Toolbox-green)](#)
[![Git](https://img.shields.io/badge/Git-2.34+-F05032?logo=git)](#)
[![VS Code](https://img.shields.io/badge/IDE-VS%20Code-007ACC?logo=visualstudiocode)](#)
[![Shell](https://img.shields.io/badge/Shell-Bash-4EAA25?logo=gnubash)](#)
<!-- [![Docker](https://img.shields.io/badge/Container-Docker-2496ED?logo=docker)](#) -->
<!-- [![Docker Compose](https://img.shields.io/badge/Docker--Compose-Blue?logo=docker)](#) -->
[![Arquitectura](https://img.shields.io/badge/CPU-x86_64%20%7C%20ARM64-lightgrey?logo=amd)](#)
[![License](https://img.shields.io/badge/License-BSD-green.svg)](LICENSE)
[![Versión Actual](https://img.shields.io/badge/Versión-v1.0.0-blue)](#)
[![Repositorio](https://img.shields.io/badge/GitHub-MrDavidAlv-181717?logo=github)](https://github.com/MrDavidAlv/Axioma_robot)

---

## 🚀 Quick Start

```bash
# 1. Instalar ROS2 Humble (Ubuntu 22.04)
sudo apt update && sudo apt install ros-humble-desktop

# 2. Instalar dependencias del proyecto
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete \
                     gazebo ros-humble-gazebo-ros-pkgs \
                     ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
                     ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
                     ros-humble-joy ros-humble-teleop-twist-keyboard \
                     ros-humble-rviz2 ros-humble-xacro ros-humble-tf2-tools

# 3. Clonar y compilar el proyecto
mkdir -p ~/ros2/axioma_humble_ws/src
cd ~/ros2/axioma_humble_ws/src
git clone https://github.com/MrDavidAlv/Axioma_robot.git .
cd ~/ros2/axioma_humble_ws
colcon build --symlink-install
source install/setup.bash

# 4. Lanzar SLAM para crear mapas
ros2 launch axioma_bringup slam_bringup.launch.py

# O lanzar navegación autónoma (requiere mapa previo)
ros2 launch axioma_bringup navigation_bringup.launch.py
```

**📖 Ver [Instalación Detallada](#-instalación) | [Guía de Ejecución](#-ejecución)**

---

## 📖 Descripción

Este proyecto desarrolla software de navegación autónoma con ROS2 para la plataforma robótica móvil **Axioma.io**, diseñada por el Semillero de Robótica SIRO. El sistema convierte el robot en una plataforma autónoma capaz de percibir su entorno mediante sensores LiDAR, calcular trayectorias óptimas y transportar productos entre puntos designados sin intervención humana.

La solución implementa algoritmos avanzados de SLAM (Simultaneous Localization and Mapping) para mapeo en tiempo real, localización mediante AMCL (Adaptive Monte Carlo Localization), y planificación de trayectorias con Nav2, cumpliendo los requerimientos de automatización logística en cadenas de producción industrial con altos niveles de servicio, calidad y eficiencia.

### 🔑 Palabras Clave

Robot móvil, navegación autónoma, logística industrial, planificación de trayectorias, ROS2 Humble, Nav2, SLAM, differential drive, skid-steering

### 🎯 Objetivo General

Diseñar, simular e implementar software de planificación de trayectorias robóticas para la plataforma de robótica móvil Axioma.io, permitiendo el transporte autónomo de productos desde un punto inicial hasta un punto final dentro de espacios de trabajo determinados, automatizando la gestión y coordinación logística en cadenas y procesos de producción.

### 📋 Objetivos Específicos

- Diseñar un entorno de simulación tridimensional que emula áreas de trabajo con obstáculos estáticos y dinámicos
- Instrumentar el robot virtual con sensores de orientación, posición, navegación y mapeo (LiDAR, encoders, IMU)
- Programar el ecosistema ROS2 con nodos especializados para localización (AMCL), control (differential drive), navegación (Nav2) y mapeo (SLAM Toolbox)
- Desarrollar e implementar técnicas de planificación de trayectorias que calculen rutas óptimas considerando obstáculos y restricciones cinemáticas
- Integrar el software desarrollado en el robot físico Axioma.io con sensores reales para odometría y cálculo de velocidades angulares

<div align="center">
  <img src="https://github.com/MrDavidAlv/Axioma_robot/blob/main/image/axioma.jpeg" alt="Robot Axioma físico" width="400">
  <p><i>Robot Axioma.io - Plataforma física 4WD</i></p>
</div>

<div align="center">
  <img src="https://github.com/MrDavidAlv/Axioma_robot/blob/main/image/open_software.jpeg" alt="Open Source Software" width="400">
  <p><i>Stack tecnológico open source utilizado</i></p>
</div>

---

## 📑 Tabla de Contenidos

- [Quick Start](#-quick-start)
- [Descripción](#-descripción)
- [Características Principales](#-características-principales)
- [Galería del Robot](#-galería-del-robot)
- [Videos del Sistema](#-videos-del-sistema)
- [Arquitectura del Sistema](#-arquitectura-del-sistema)
- [Modelo Matemático](#-modelo-matemático)
- [Requisitos del Sistema](#-requisitos-del-sistema)
- [Instalación](#-instalación)
- [Compilación](#-compilación)
- [Ejecución](#-ejecución)
- [Estructura del Proyecto](#-estructura-del-proyecto)

---

## 🌟 Características Principales

<div align="center">

| Feature | Descripción |
|---------|-------------|
| 🗺️ **SLAM en Tiempo Real** | Mapeo simultáneo y localización con SLAM Toolbox asíncrono |
| 🎯 **Navegación Autónoma** | Sistema Nav2 completo con planificación global (NavFn) y local (DWB) |
| 🚧 **Evitación de Obstáculos** | Detección y evasión en tiempo real con LiDAR 360° RPLidar A1 |
| 🎮 **Control Teleoperable** | Soporte para Xbox controller y teleop_twist_keyboard durante mapeo |
| 📊 **Visualización Completa** | RViz2 con costmaps dinámicos, trayectorias planificadas y partículas AMCL |
| 🤖 **Robot Diferencial 4WD** | Odometría robusta con encoders de 1000 PPR y control skid-steering |
| 🔧 **Totalmente Configurable** | Parámetros Nav2, AMCL, SLAM y DWB ajustables según aplicación |
| 💻 **Código Abierto** | Licencia BSD - Libre para uso académico, investigación y comercial |

</div>

---

## 📸 Galería del Robot

<div align="center">
<table>
  <tr>
    <td><img src="images/robot1.jpg" width="400"/></td>
    <td><img src="images/robot2.jpg" width="400"/></td>
  </tr>
  <tr>
    <td><img src="images/robot3.jpg" width="400"/></td>
    <td><img src="images/robot4.jpg" width="400"/></td>
  </tr>
  <tr>
    <td><img src="images/robot5.png" width="400"/></td>
    <td><img src="images/robot6.jpg" width="400"/></td>
  </tr>
</table>
</div>

---

## 🎥 Videos del Sistema

### Demostración Completa - ROS2 Humble

Demostración integral del sistema funcionando: SLAM en tiempo real, guardado de mapa y navegación autónoma Nav2.

<div align="center">

[![Video Demostración Completa Axioma](https://img.youtube.com/vi/hl_HeULvuvQ/maxresdefault.jpg)](https://www.youtube.com/watch?v=hl_HeULvuvQ)

**[▶️ Ver video completo en YouTube](https://www.youtube.com/watch?v=hl_HeULvuvQ)**

</div>

---

### Videos del Sistema Funcionando

> **Nota:** Los siguientes videos corresponden a la versión con ROS2 Foxy. La funcionalidad en Humble es idéntica con mejoras en rendimiento, estabilidad y nuevas características de Nav2.

<div align="center">

| **Navegación Autónoma** | **SLAM y Mapeo** |
|:------------------------:|:-----------------:|
| [![Axioma Navigation Part 1](https://img.youtube.com/vi/U28n4vSAwDk/0.jpg)](https://youtu.be/U28n4vSAwDk) | [![Axioma SLAM Part 2](https://img.youtube.com/vi/A-7UMoYXUBQ/0.jpg)](https://youtu.be/A-7UMoYXUBQ) |
| *Navegación autónoma en entorno con mapa cargado* | *Mapeo en tiempo real con LiDAR y SLAM Toolbox* |

| **Sensores y Frames TF** | **Ensamblaje Mecánico** |
|:---------------------:|:-----------------:|
| [![Axioma Sensors Part 3](https://img.youtube.com/vi/dHnnpMOO5yg/0.jpg)](https://youtu.be/dHnnpMOO5yg) | [![Axioma Assembly](https://img.youtube.com/vi/buS84GiqQug/0.jpg)](https://youtu.be/buS84GiqQug) |
| *Visualización RViz: sensores, transformadas y odometría* | *Diseño CAD y ensamblaje en Autodesk Inventor* |

| **Competencia Mercury Robotics** | **Control Teleoperado** |
|:-----------------------------:|:--------------------------:|
| [![Mercury Challenge 2019](https://img.youtube.com/vi/8E0mYynNUog/0.jpg)](https://youtu.be/8E0mYynNUog) | [![Axioma Teleop](https://img.youtube.com/vi/sHgdL3dffgw/0.jpg)](https://youtu.be/sHgdL3dffgw) |
| *Axioma One en Mercury Robotics Challenge 2019* | *Plataforma teleoperada con Raspberry Pi y Flask* |

</div>

### 🏆 Características Técnicas Demostradas

- **Planificación de Trayectorias**: Algoritmos NavFn (Dijkstra) y DWB (Dynamic Window Approach)
- **SLAM Robusto**: Mapeo y localización simultáneos con corrección de bucles (loop closure)
- **Sistema de Transformadas**: Árbol TF completo `map → odom → base_link → sensors`
- **Diseño Mecánico**: Estructura de aluminio con tracción 4WD y caster ball posterior
- **Interfaz de Teleoperación**: Control remoto WebSocket y Xbox controller

---

## 🏗️ Arquitectura del Sistema

### Transformadas (TF Tree)
<div align="center">
<img src="images/URDF-TF.png" width="800"/>
</div>

El árbol de transformadas define las relaciones espaciales entre todos los componentes del robot: `map → odom → base_footprint → base_link → sensors`. El plugin differential drive publica la transformada `odom → base_link`, mientras que AMCL publica `map → odom` para corrección de deriva odométrica.

### Sistema SLAM
<div align="center">
<img src="images/SLAM.png" width="800"/>
</div>

SLAM Toolbox implementa un algoritmo de graph-based SLAM que genera mapas de ocupación 2D en tiempo real. El sistema procesa datos del LiDAR a 5.5 Hz y odometría diferencial a 50 Hz, aplicando optimización de pose-graph para mantener consistencia global del mapa.

### Sistema de Navegación
<div align="center">
<img src="images/Navigation.png" width="800"/>
</div>

Nav2 stack integra múltiples componentes: planificador global NavFn para búsqueda de caminos óptimos, controlador local DWB para generación de trayectorias suaves, costmaps estáticos y dinámicos para representación del entorno, y behaviors de recuperación para manejo de situaciones de bloqueo.

---

## 📐 Modelo Matemático

<div align="center">
<img src="images/modelo-matematico.png" width="800"/>
</div>

El robot Axioma implementa un modelo matemático completo de cinemática diferencial para robots 4WD con configuración skid-steering. El modelo describe las relaciones entre velocidades de ruedas y velocidades del robot, dinámica del sistema, y parámetros de control Nav2.

### 📚 Documentación Técnica Completa

La documentación matemática detallada está disponible en [`documentacion/modelo-matematico/`](./documentacion/modelo-matematico/):

| Documento | Descripción |
|-----------|-------------|
| **[README](./documentacion/modelo-matematico/README.md)** | Introducción, notación matemática, ecuaciones fundamentales y estructura del modelo |
| **[Cinemática](./documentacion/modelo-matematico/cinematica.md)** | Modelos directo e inverso para differential drive, odometría y límites cinemáticos |
| **[Control](./documentacion/modelo-matematico/control.md)** | Plugin Gazebo diff_drive, Nav2 DWB controller, Velocity Smoother y AMCL |
| **[Parámetros](./documentacion/modelo-matematico/parametros.md)** | Parámetros físicos, geométricos, dinámicos e inerciales con valores reales verificados |
| **[Diagrama Excalidraw](./documentacion/modelo-matematico/modelo-axioma.excalidraw)** | Representación visual completa y editable del modelo (formato JSON) |

### 🔬 Resumen Técnico

**Parámetros Geométricos**:
- Radio de rueda: $r = 0.0381$ m
- Separación entre ruedas: $W = 0.1725$ m
- Distancia entre ejes: $L = 0.1356$ m
- Masa total: $m = 5.525$ kg

**Límites Operacionales**:
- Velocidad lineal máxima: $v_{max} = 0.26$ m/s
- Velocidad angular máxima: $\omega_{max} = 1.0$ rad/s
- Aceleración lineal máxima: $a_{max} = 2.5$ m/s²
- Aceleración angular máxima: $\alpha_{max} = 3.2$ rad/s²

**Cinemática Diferencial**:

Para un robot differential drive con 4 ruedas motrices en configuración skid-steering:

$$
v = \frac{r(\omega_R + \omega_L)}{2}
$$

$$
\omega = \frac{r(\omega_R - \omega_L)}{W}
$$

**Cinemática Inversa**:

$$
\omega_L = \frac{v - \omega \cdot W/2}{r}
$$

$$
\omega_R = \frac{v + \omega \cdot W/2}{r}
$$

Donde:
- $v$: Velocidad lineal del centro del robot [m/s]
- $\omega$: Velocidad angular (yaw rate) del robot [rad/s]
- $\omega_L$, $\omega_R$: Velocidades angulares de pares de ruedas izquierda/derecha [rad/s]
- $r = 0.0381$ m (radio de rueda)
- $W = 0.1725$ m (separación entre centros de ruedas)

📖 **Documentación matemática completa**: [documentacion/modelo-matematico/](./documentacion/modelo-matematico/)

---

## 💻 Requisitos del Sistema

### Software Base
- **Sistema Operativo**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS2**: Humble Hawksbill
- **Gazebo**: Classic 11
- **Python**: 3.8 o superior
- **CMake**: 3.16 o superior

### Dependencias ROS2

#### Paquetes Core
```bash
ros-humble-ros-base
ros-humble-gazebo-ros-pkgs
ros-humble-robot-state-publisher
ros-humble-joint-state-publisher
```

#### Navegación y SLAM
```bash
ros-humble-navigation2
ros-humble-nav2-bringup
ros-humble-slam-toolbox
```

#### Control y Teleoperation
```bash
ros-humble-joy
ros-humble-teleop-twist-joy
ros-humble-teleop-twist-keyboard
ros-humble-rviz2
ros-humble-xacro
```

#### Herramientas y Utilidades
```bash
ros-humble-tf2-tools
ros-humble-tf-transformations
ros-humble-rqt-robot-steering
```

### Hardware Recomendado
- **CPU**: Intel i5 8th Gen / AMD Ryzen 5 o superior (4 núcleos mínimo)
- **RAM**: 8 GB mínimo, 16 GB recomendado para simulación y Nav2
- **GPU**: NVIDIA con soporte CUDA (opcional, para aceleración de costmaps)
- **Almacenamiento**: 10 GB libres para workspace y dependencias

---

## 🔧 Instalación

### 1. Instalar ROS2 Humble

```bash
# Configurar locale UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Agregar repositorio ROS2
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar ROS2 Humble Desktop (incluye RViz2, demos y tutoriales)
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

### 2. Instalar Dependencias del Proyecto

```bash
# Instalar todas las dependencias en un solo comando
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-argcomplete \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-joy \
  ros-humble-joy-linux \
  ros-humble-teleop-twist-joy \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rviz2 \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-tools \
  ros-humble-tf-transformations \
  ros-humble-rqt-robot-steering

# Inicializar rosdep (solo primera vez)
sudo rosdep init
rosdep update
```

### 3. Clonar el Repositorio

```bash
# Crear workspace ROS2
mkdir -p ~/ros2/axioma_humble_ws/src
cd ~/ros2/axioma_humble_ws/src

# Clonar repositorio desde GitHub
git clone https://github.com/MrDavidAlv/Axioma_robot.git .
```

---

## 🔨 Compilación

### Compilar todos los paquetes

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Compilar workspace completo con enlaces simbólicos
cd ~/ros2/axioma_humble_ws
colcon build --symlink-install

# Source el workspace compilado
source install/setup.bash

# Agregar al .bashrc para sourcing automático
echo "source ~/ros2/axioma_humble_ws/install/setup.bash" >> ~/.bashrc
```

### Compilar paquetes específicos

```bash
# Compilar solo axioma_bringup
colcon build --packages-select axioma_bringup

# Compilar navegación y descripción
colcon build --packages-select axioma_navigation axioma_description

# Compilar con output verbose para debugging
colcon build --packages-select axioma_bringup --event-handlers console_direct+
```

### Limpiar y recompilar

```bash
# Eliminar directorios de build anteriores
cd ~/ros2/axioma_humble_ws
rm -rf build/ install/ log/

# Recompilar desde cero
colcon build --symlink-install
```

---

## 🚀 Ejecución

### 1️⃣ SLAM (Mapeo en Tiempo Real)

Ejecuta simulación completa con Gazebo, control teleoperable y SLAM Toolbox para crear mapas del entorno:

```bash
source ~/ros2/axioma_humble_ws/install/setup.bash
ros2 launch axioma_bringup slam_bringup.launch.py
```

**Componentes lanzados**:
- Gazebo Classic con mundo de prueba predefinido
- Plugin `libgazebo_ros_diff_drive.so` para control y odometría
- SLAM Toolbox en modo asíncrono para mapeo en tiempo real
- RViz2 con configuración SLAM (mapa, scan, transformadas)
- Soporte para teleoperación con teclado o joystick

**Control del robot**:
- **Teclado**: Ejecutar en terminal separada `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- **Xbox Controller**: Plugin joy habilitado automáticamente con `teleop_twist_joy`

**Proceso de mapeo**:
1. Iniciar SLAM launch
2. Mover el robot lentamente por todo el entorno
3. Observar construcción del mapa en RViz2
4. Asegurar cobertura completa del área de interés
5. Guardar mapa al finalizar (ver siguiente sección)

### 2️⃣ Guardar Mapa Generado

Una vez completado el mapeo con SLAM:

```bash
# En terminal separada (mantener SLAM corriendo)
cd ~/ros2/axioma_humble_ws
ros2 run nav2_map_server map_saver_cli -f src/axioma_navigation/maps/mi_mapa

# Especificar formato y resolución
ros2 run nav2_map_server map_saver_cli -f src/axioma_navigation/maps/mapa_detallado --occ 65 --free 25
```

**Archivos generados**:
- `mi_mapa.pgm`: Imagen del mapa en escala de grises (ocupado=negro, libre=blanco, desconocido=gris)
- `mi_mapa.yaml`: Metadatos del mapa (resolución, origen, thresholds)

**Configurar mapa por defecto**:
```bash
# Editar nav2_params.yaml para usar nuevo mapa
nano src/axioma_navigation/config/nav2_params.yaml
# Cambiar: yaml_filename: "/path/to/mi_mapa.yaml"
```

### 3️⃣ Navegación Autónoma (Nav2)

Ejecuta navegación autónoma con Nav2 usando mapa previamente guardado:

```bash
source ~/ros2/axioma_humble_ws/install/setup.bash
ros2 launch axioma_bringup navigation_bringup.launch.py
```

**Componentes lanzados**:
- Gazebo Classic con mismo mundo del mapa
- Plugin differential drive para control
- Nav2 stack completo:
  - **AMCL**: Localización con filtro de partículas
  - **Map Server**: Carga mapa estático guardado
  - **Planner Server**: NavFn para rutas globales
  - **Controller Server**: DWB para trayectorias locales
  - **Behavior Server**: Comportamientos de recuperación
  - **BT Navigator**: Árbol de comportamiento principal
- RViz2 con herramientas de navegación

**Uso de navegación**:
1. **Establecer pose inicial**:
   - En RViz2, clic en botón `2D Pose Estimate`
   - Clic y arrastrar en posición aproximada del robot en el mapa
   - Flecha indica orientación inicial
   - Las partículas AMCL deben converger alrededor del robot

2. **Enviar objetivo de navegación**:
   - Clic en botón `2D Goal Pose`
   - Clic y arrastrar en destino deseado del mapa
   - Flecha indica orientación final
   - El robot planificará y ejecutará trayectoria automáticamente

3. **Monitoreo**:
   - Costmap global (azul): Mapa estático inflado
   - Costmap local (rojo): Obstáculos dinámicos detectados
   - Path global (verde): Ruta planificada por NavFn
   - Path local (amarillo): Trayectoria suave ejecutada por DWB

### Visualización Standalone con RViz2

```bash
# Visualizar modelo URDF y transformadas TF
ros2 launch axioma_description display.launch.py

# Abrir configuración personalizada de SLAM
rviz2 -d ~/ros2/axioma_humble_ws/src/axioma_navigation/config/slam.rviz

# Abrir configuración personalizada de navegación
rviz2 -d ~/ros2/axioma_humble_ws/src/axioma_navigation/config/navigation.rviz
```

---

## 📂 Estructura del Proyecto

```
axioma_humble_ws/
├── src/
│   ├── axioma_bringup/                    # Launch files principales
│   │   ├── launch/
│   │   │   ├── slam_bringup.launch.py             ⭐ SLAM completo + Gazebo
│   │   │   └── navigation_bringup.launch.py       ⭐ Nav2 autónomo + Gazebo
│   │   └── package.xml
│   │
│   ├── axioma_navigation/                 # Configuración Nav2 y SLAM
│   │   ├── config/
│   │   │   ├── nav2_params.yaml                   Parámetros completos Nav2
│   │   │   ├── slam_params.yaml                   Parámetros SLAM Toolbox
│   │   │   ├── navigation.rviz                    Configuración RViz Nav2
│   │   │   └── slam.rviz                          Configuración RViz SLAM
│   │   ├── maps/
│   │   │   ├── mapa.pgm                           Mapa de ocupación (imagen)
│   │   │   └── mapa.yaml                          Metadatos del mapa
│   │   └── package.xml
│   │
│   └── axioma_description/                # Modelo URDF/SDF del robot
│       ├── models/axioma_v2/
│       │   └── model.sdf                          Modelo Gazebo con plugins
│       ├── urdf/
│       │   └── axioma.urdf                        Descripción URDF para TF
│       ├── meshes/                                Geometrías 3D (DAE/STL)
│       ├── worlds/                                Mundos Gazebo (.world)
│       └── package.xml
│
├── documentacion/
│   └── modelo-matematico/                 # Documentación técnica
│       ├── README.md                              Introducción al modelo
│       ├── cinematica.md                          Cinemática diferencial
│       ├── control.md                             Sistema de control Nav2
│       ├── parametros.md                          Parámetros físicos reales
│       └── modelo-axioma.excalidraw               Diagrama visual completo
│
├── images/                                 # Recursos visuales
│   ├── portada.png
│   ├── modelo-matematico.png
│   ├── Navigation.png
│   ├── SLAM.png
│   ├── URDF-TF.png
│   └── robot1-6.jpg
│
└── README.md                               # Documentación principal
```

---

## 🔍 Flujo de Datos del Sistema

### Control Manual (Teleoperación)
```
Teclado / Xbox Controller
    ↓
/cmd_vel (geometry_msgs/Twist)
    ↓
Gazebo Plugin: libgazebo_ros_diff_drive.so
    ↓ (aplica cinemática inversa)
4 Ruedas: 2 pares sincronizados (ω_L, ω_R)
    ↓
Simulación Física Gazebo
    ↓
Odometría Calculada: /odom (nav_msgs/Odometry)
    ↓
TF publicado: odom → base_link
```

### SLAM (Mapeo)
```
Gazebo LiDAR Plugin: gpu_ray
    ↓
/scan (sensor_msgs/LaserScan) @ 5.5 Hz
    ↓
SLAM Toolbox (Karto SLAM)
    ├─ Odometría: /odom
    ├─ Scan matching
    ├─ Graph optimization
    └─ Loop closure detection
    ↓
/map (nav_msgs/OccupancyGrid)
TF publicado: map → odom (corrección de deriva)
    ↓
RViz2 (Visualización)
```

### Navegación Autónoma (Nav2)
```
Usuario: 2D Goal Pose en RViz2
    ↓
/goal_pose (geometry_msgs/PoseStamped)
    ↓
BT Navigator (Behavior Tree)
    ↓
Planner Server (NavFn: algoritmo Dijkstra)
    ├─ Input: Map, Start Pose, Goal Pose
    └─ Output: /plan (Global Path)
    ↓
Controller Server (DWB Local Planner)
    ├─ Input: Global Path, Costmaps, Odometry
    ├─ Sampling: 20 vx × 20 vθ trayectorias
    ├─ Critics: Obstacle, PathAlign, GoalAlign
    └─ Output: Trayectoria óptima local
    ↓
Velocity Smoother (filtro aceleración)
    ↓
/cmd_vel (geometry_msgs/Twist)
    ↓
Plugin Differential Drive → Ejecución
    ↓
AMCL (Localización continua)
    ├─ Filtro de partículas Monte Carlo
    ├─ Sensor model: Likelihood field
    ├─ Motion model: Differential
    └─ /amcl_pose (Pose corregida)
```

---

## 🤖 Sistema de Control

### Plugin Differential Drive (Gazebo)

El robot utiliza `libgazebo_ros_diff_drive.so`, plugin estándar de Gazebo que implementa control cinemático directo para robots diferenciales:

**Configuración** (`model.sdf:427-454`):
```xml
<plugin name='skid_steer_drive' filename='libgazebo_ros_diff_drive.so'>
  <update_rate>50</update_rate>
  <num_wheel_pairs>2</num_wheel_pairs>

  <!-- Par izquierdo: ruedas frontales y traseras izquierdas -->
  <left_joint>base_to_wheel1</left_joint>
  <left_joint>base_to_wheel2</left_joint>

  <!-- Par derecho: ruedas frontales y traseras derechas -->
  <right_joint>base_to_wheel3</right_joint>
  <right_joint>base_to_wheel4</right_joint>

  <!-- Parámetros geométricos -->
  <wheel_separation>0.1725</wheel_separation>
  <wheel_diameter>0.0762</wheel_diameter>

  <!-- Límites dinámicos -->
  <max_wheel_torque>20</max_wheel_torque>
  <max_wheel_acceleration>1.0</max_wheel_acceleration>

  <!-- Publicación de odometría -->
  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
  <odometry_frame>odom</odometry_frame>
  <robot_base_frame>base_link</robot_base_frame>
</plugin>
```

**Funcionamiento interno**:
1. **Suscripción**: Lee comandos de velocidad desde `/cmd_vel` → $(v, \omega)$
2. **Cinemática Inversa**: Calcula velocidades angulares de ruedas
   - $\omega_L = (v - \omega \cdot W/2) / r$
   - $\omega_R = (v + \omega \cdot W/2) / r$
3. **Control de Motores**: Aplica velocidades a 2 pares de ruedas sincronizadas
4. **Odometría**: Integra velocidades de ruedas para estimar pose
5. **Publicación**: Envía `/odom` y TF `odom → base_link` a 50 Hz

### Controlador Local Nav2 DWB

Dynamic Window Approach (DWB) genera y evalúa trayectorias locales considerando restricciones dinámicas.

**Configuración** (`nav2_params.yaml:108-170`):
- **Velocidades máximas**: $v_{max} = 0.26$ m/s, $\omega_{max} = 1.0$ rad/s
- **Aceleraciones máximas**: $a_{max} = 2.5$ m/s², $\alpha_{max} = 3.2$ rad/s²
- **Muestreo de trayectorias**: 20 velocidades lineales × 20 angulares = 400 trayectorias
- **Tiempo de simulación**: 1.7 s hacia adelante
- **Frecuencia de control**: 20 Hz

**Critics (funciones de costo)**:
- `BaseObstacle`: Penaliza cercanía a obstáculos
- `PathAlign`: Alineación con path global
- `GoalAlign`: Orientación hacia objetivo final
- `PathDist`: Distancia al path global
- `GoalDist`: Distancia al goal
- `RotateToGoal`: Rotación final hacia orientación del goal

---

## 📝 Parámetros Físicos del Robot

### Geometría y Masa

| Parámetro | Valor | Fuente |
|-----------|-------|--------|
| **Masa total** | 5.525 kg | Suma masas SDF |
| **Masa base_link** | 5.0 kg | `model.sdf:323` |
| **Masa por rueda** | 0.1 kg × 4 | `model.sdf:72,149,226,303` |
| **Masa caster** | 0.125 kg | `model.sdf:383` |
| **Dimensiones (L×W×H)** | 0.1356 × 0.1725 × 0.1 m | Geometría SDF |
| **Radio de rueda** | 0.0381 m | `model.sdf:72` |
| **Ancho de rueda** | 0.03 m | `model.sdf:73` |

### Propiedades Dinámicas

| Parámetro | Valor |
|-----------|-------|
| **Coeficiente fricción μ** | 1.0 (ruedas), 0.0 (caster) |
| **Coeficiente fricción μ₂** | 1.0 (ruedas), 0.0 (caster) |
| **Torque máximo por rueda** | 20 N·m |
| **Aceleración máxima** | 1.0 m/s² (plugin), 2.5 m/s² (Nav2) |
| **Update rate control** | 50 Hz (plugin diff_drive) |

### Sensor LiDAR (RPLidar A1)

- **Tipo**: `gpu_ray` sensor en Gazebo
- **Muestras por barrido**: 360
- **Rango angular**: 360° (0 a 2π rad)
- **Rango de distancia**: 0.15 m (min) - 12.0 m (max)
- **Frecuencia de barrido**: 5.5 Hz
- **Resolución angular**: 1° (π/180 rad)
- **Frame**: `base_scan`

---

## 🔄 Comandos Útiles

### Comandos de Setup
```bash
# Setup completo en cada terminal nueva
source /opt/ros/humble/setup.bash
source ~/ros2/axioma_humble_ws/install/setup.bash

# O agregar a ~/.bashrc para automático
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2/axioma_humble_ws/install/setup.bash" >> ~/.bashrc
```

### Comandos de Ejecución
```bash
# SLAM para mapeo
ros2 launch axioma_bringup slam_bringup.launch.py

# Navegación autónoma
ros2 launch axioma_bringup navigation_bringup.launch.py

# Teleoperación con teclado
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Guardar mapa generado
ros2 run nav2_map_server map_saver_cli -f ~/maps/mi_mapa
```

### Comandos de Monitoreo
```bash
# Listar nodos activos
ros2 node list

# Listar tópicos activos
ros2 topic list

# Ver frecuencia de publicación
ros2 topic hz /scan
ros2 topic hz /odom

# Monitorear cmd_vel en tiempo real
ros2 topic echo /cmd_vel

# Ver transformadas en tiempo real
ros2 run tf2_ros tf2_echo map base_link

# Generar diagrama del árbol TF
ros2 run tf2_tools view_frames
evince frames.pdf
```

### Comandos de Debugging
```bash
# Ver información detallada de nodo
ros2 node info /slam_toolbox

# Inspeccionar interfaz de tópico
ros2 topic info /scan --verbose

# Listar parámetros de nodo
ros2 param list /controller_server

# Obtener valor de parámetro
ros2 param get /controller_server controller_frequency

# Grabar datos a rosbag para análisis posterior
ros2 bag record -a -o datos_navegacion

# Reproducir rosbag grabado
ros2 bag play datos_navegacion
```

---

## 📞 Contacto y Soporte

**Autor**: Mario David Alvarez Vallejo
**Institución**: Semillero de Robótica SIRO
**Repositorio**: [github.com/MrDavidAlv/Axioma_robot](https://github.com/MrDavidAlv/Axioma_robot)
**Licencia**: BSD - Libre para uso académico, investigación y comercial

---

<div align="center">

**Desarrollado por el Semillero de Robótica SIRO**

*Automatización de Logística Industrial con ROS2*

</div>
