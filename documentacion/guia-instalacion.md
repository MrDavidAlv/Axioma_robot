# 🛠️ Guía de Instalación

Todo lo necesario para tener el Axioma corriendo, de un Ubuntu limpio a
`colcon build`. El README solo enlaza aquí; esta es la referencia completa.

---

## 📋 Requisitos

### Software

| Componente | Versión |
|------------|---------|
| Sistema operativo | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo | Ignition Fortress (gz-sim 6) — la versión contra la que está compilado `ros-humble-ros-gz` |
| Python | 3.8+ |
| CMake | 3.16+ |

> `ros-humble-ros-gz` está compilado contra Fortress. Instalar Gazebo Harmonic
> además no cambia qué simulador arrancan los launch de este proyecto.

### Hardware recomendado

| Recurso | Mínimo | Recomendado |
|---------|--------|-------------|
| CPU | Intel i5 8ª gen / AMD Ryzen 5 | 4+ núcleos |
| RAM | 8 GB | 16 GB |
| Disco | 10 GB libres | — |

---

## 1. Instalar ROS 2 Humble

```bash
# Configurar locale y repositorio
sudo apt update && sudo apt install locales curl
sudo locale-gen en_US en_US.UTF-8
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar ROS 2 Humble Desktop
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
```

## 2. Instalar Ignition Gazebo Fortress

```bash
sudo apt install ignition-fortress ros-humble-ros-gz
```

## 3. Instalar las dependencias del proyecto

```bash
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard ros-humble-joy \
  ros-humble-robot-state-publisher ros-humble-tf2-tools

sudo rosdep init && rosdep update
```

## 4. Clonar el repositorio

```bash
mkdir -p ~/ros2/axioma_ws/src
cd ~/ros2/axioma_ws/src
git clone https://github.com/MrDavidAlv/Axioma_robot.git .
```

## 5. Compilar

```bash
cd ~/ros2/axioma_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` importa: los scripts guardan mapas y figuras directo en
`src/`, así que quedan bajo control de versiones en vez de perderse en `build/`
en la siguiente compilación.

Para no tener que sourcear en cada terminal nueva:

```bash
echo "source ~/ros2/axioma_ws/install/setup.bash" >> ~/.bashrc
```

---

Con esto compilado, sigue con la [Guía de Uso](guia-uso.md) o vuelve al
[README](../README.md).
