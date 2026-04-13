# LIMO Rover Simulation

A complete ROS 2 Humble and Gazebo Classic simulation package for the LIMO rover.

## ⚠️ Prerequisites

This package requires **Ubuntu 22.04 LTS**. If you are on a different OS, please use a dual-boot setup, virtual machine, or WSL2.

---

## 1. System Setup (For First-Time Users)

If you do not have ROS 2 Humble or Gazebo installed, run the following commands in your terminal to set up your environment.

### Install ROS 2 Humble

```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

### Install Gazebo Classic & ROS 2 Integration

```bash
sudo apt install gazebo
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-xacro
```

### Install RTAB-Map (For 3D SLAM)

```bash
sudo apt install ros-humble-rtabmap-ros
```

### Install Nav2 (For Autonomous Navigation)

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-pointcloud-to-laserscan
```

---

## 2. Workspace Setup

Once ROS 2 is installed, set up your workspace and clone this repository.

```bash
# Source the base ROS 2 installation
source /opt/ros/humble/setup.bash
# Create a workspace
mkdir -p ~/rover_ws/src
cd ~/rover_ws/src
# Clone the repositories
git clone -b research https://github.com/spabhut/ProjectVanguard.git rover
git clone -b research https://github.com/spabhut/limo_ros2.git
# Install missing dependencies automatically
cd ~/rover_ws
rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
# Build the workspace
colcon build --symlink-install
```

---

## 3. Running the Simulation

Every time you open a new terminal, source the workspace first:

```bash
cd ~/rover_ws
source install/setup.bash
```

You will need **three separate terminals** — one for Gazebo, one for SLAM, and one for Nav2.

### Terminal 1 — Gazebo Simulation

```bash
cd ~/rover_ws && source install/setup.bash
ros2 launch rover rover.launch.py
```

### Terminal 2 — SLAM (RTAB-Map)

```bash
cd ~/rover_ws && source install/setup.bash
ros2 launch rover slam.launch.py
```

### Terminal 3 — Nav2 Navigation Stack

```bash
cd ~/rover_ws && source install/setup.bash
ros2 launch rover nav2.launch.py
```

---

## Package Structure

```
rover/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── nav2_params.yaml      # Nav2 navigation parameters
├── include/
│   └── rover/                # C++ headers (if any)
├── launch/
│   ├── nav2.launch.py        # Nav2 navigation stack launch
│   ├── rover.launch.py       # Gazebo simulation launch
│   └── slam.launch.py        # SLAM (RTAB-Map) launch
├── rviz/
│   └── rover.rviz            # Pre-configured RViz2 layout
├── src/                      # C++ source files (if any)
└── worlds/
    └── rover.world           # Custom Gazebo world
```

---

## Troubleshooting

**Gazebo doesn't open / crashes immediately**
```bash
# Make sure Gazebo is properly sourced
source /usr/share/gazebo/setup.sh
```

**`colcon build` fails with missing packages**
```bash
# Re-run rosdep to catch any remaining dependencies
rosdep install --from-paths src -y --ignore-src
```

**RViz shows no robot model**
```bash
# Confirm the robot_description topic is being published
ros2 topic echo /robot_description
```

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.