# 6-Wheeled Rover Simulation

A complete ROS 2 Humble and Gazebo Classic simulation package for a 6-wheeled skid-steer rover equipped with an Intel RealSense D455 depth camera.

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

---

## 2. Workspace Setup

Once ROS 2 is installed, set up your workspace and clone this repository.

```bash
# Source the base ROS 2 installation
source /opt/ros/humble/setup.bash

# Create a workspace
mkdir -p ~/rover_ws/src
cd ~/rover_ws/src

# Clone the repository
git clone https://github.com/YOUR_USERNAME/rover.git

# Install missing dependencies automatically
cd ~/rover_ws
rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src

# Build the workspace
colcon build --symlink-install
```

> **Note:** Replace `YOUR_USERNAME` with the actual GitHub username where this repo is hosted.

---

## 3. Running the Simulation

Every time you open a new terminal to run this project, you must source the workspace first.

```bash
cd ~/rover_ws
source install/setup.bash

# Launch the simulation
ros2 launch rover rover.launch.py
```

### What to Expect

| # | What Opens | What It Shows |
|---|-----------|---------------|
| 1 | **Gazebo Classic** | 6-wheeled rover spawned in an empty world |
| 2 | **RViz2** | Pre-configured 3D rotatable view |
| 3 | **RViz2 topics** | Robot model, raw D455 camera feed, live 3D PointCloud |

---

## Package Structure

```
rover/
├── launch/
│   └── rover.launch.py       # Main launch file
├── urdf/
│   └── rover.urdf.xacro      # Robot description with D455 camera
├── worlds/
│   └── empty.world           # Default Gazebo world
├── rviz/
│   └── rover.rviz            # Pre-configured RViz2 layout
├── config/
│   └── ...                   # Controller and sensor configs
└── README.md
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
