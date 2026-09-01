# 🛠️ Installation Guide

Everything needed to get Axioma running, from a clean Ubuntu to
`colcon build`. The README only links here; this is the full reference.

---

## 📋 Requirements

### Software

| Component | Version |
|------------|---------|
| Operating system | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo | Ignition Fortress (gz-sim 6) — the version `ros-humble-ros-gz` is built against |
| Python | 3.8+ |
| CMake | 3.16+ |

> `ros-humble-ros-gz` is built against Fortress. Installing Gazebo Harmonic
> alongside it does not change which simulator this project's launch files
> start.

### Recommended hardware

| Resource | Minimum | Recommended |
|---------|--------|-------------|
| CPU | Intel i5 8th gen / AMD Ryzen 5 | 4+ cores |
| RAM | 8 GB | 16 GB |
| Disk | 10 GB free | — |

---

## 1. Install ROS 2 Humble

```bash
# Set up locale and repository
sudo apt update && sudo apt install locales curl
sudo locale-gen en_US en_US.UTF-8
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
```

## 2. Install Ignition Gazebo Fortress

```bash
sudo apt install ignition-fortress ros-humble-ros-gz
```

## 3. Install the project dependencies

```bash
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard ros-humble-joy \
  ros-humble-robot-state-publisher ros-humble-tf2-tools \
  ros-humble-robot-localization ros-humble-rtabmap-ros

sudo rosdep init && rosdep update
```

`robot-localization` is the EKF behind attitude-aware odometry, and
`rtabmap-ros` provides both the RGB-D visual SLAM and the node that reprojects
the ZED depth image into `/zed2i/points`. Neither is optional: the default
simulation launch uses both.

## 4. Clone the repository

```bash
mkdir -p ~/ros2/axioma_ws/src
cd ~/ros2/axioma_ws/src
git clone https://github.com/MrDavidAlv/Axioma_robot.git .
```

## 5. Build

```bash
cd ~/ros2/axioma_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` matters: the scripts save maps and figures straight into
`src/`, so they end up under version control instead of being lost in `build/`
on the next build.

To avoid sourcing in every new terminal:

```bash
echo "source ~/ros2/axioma_ws/install/setup.bash" >> ~/.bashrc
```

---

With that built, continue with the [Usage Guide](guia-uso.md) or go back to the
[README](../README.md).
