# 6-Wheeled Rover — Hardware (Jetson Orin + RealSense D455)

A ROS 2 Humble package for a 6-wheeled skid-steer rover running on a **NVIDIA Jetson Orin** with an **Intel RealSense D455** depth camera. This branch has no Gazebo simulation — it runs fully on real hardware.

## ⚠️ Prerequisites

- NVIDIA Jetson Orin with JetPack 5.x or 6.x
- Ubuntu 22.04 LTS
- Intel RealSense D455 connected via USB 3.x
- `librealsense2` already installed on the Orin

---

## 1. Install ROS 2 Humble

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

---

## 2. Install Dependencies

```bash
sudo apt install \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description \
  ros-humble-rtabmap-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-rviz-imu-plugin
```

---

## 3. Verify Camera

Before running anything, confirm the D455 is detected:

```bash
rs-enumerate-devices        # should list D455 serial number
rs-depth                    # quick live depth preview
```

---

## 4. Workspace Setup

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Create workspace
mkdir -p ~/rover_ws/src
cd ~/rover_ws/src

# Clone the hardware branch
git clone -b hardware https://github.com/spabhut/ProjectVanguard.git rover

# Install any remaining dependencies
cd ~/rover_ws
rosdep install --from-paths src -y --ignore-src

# Build
colcon build --symlink-install
```

Auto-source so every new terminal is ready:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/rover_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 5. Running on Hardware

Open **3 separate terminals** and source the workspace in each if not auto-sourced:

```bash
source ~/rover_ws/install/setup.bash
```

### Terminal 1 — Robot + Camera + RViz

```bash
ros2 launch rover rover.launch.py
```

Starts: Robot State Publisher, Joint State Publisher, RealSense D455 node, RViz2.

### Terminal 2 — SLAM (wait for Terminal 1 to fully start first)

```bash
ros2 launch rover slam.launch.py
```

Starts: RTAB-Map 3D SLAM using live D455 RGB-D feed.

### Terminal 3 — Nav2 Navigation Stack

```bash
ros2 launch rover nav2.launch.py
```

Starts: pointcloud → laserscan conversion + Nav2 navigation stack.

### Optional — Keyboard Teleoperation

```bash
ros2 run rover teleop_key
```

---

## 6. Updating the Repository

To pull the latest changes from GitHub:

```bash
cd ~/rover_ws/src/rover
git pull origin hardware

cd ~/rover_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Key Topic Map

| Data | Topic |
|---|---|
| Color image | `/d455/d455/color/image_raw` |
| Depth image | `/d455/d455/depth/image_rect_raw` |
| Point cloud | `/d455/depth/color/points` |
| IMU | `/d455/imu` |
| Laser scan (derived) | `/scan` |
| Velocity command | `/cmd_vel` |
| Map | `/map` |

---

## Package Structure

```
rover/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── nav2_params.yaml      # Nav2 navigation parameters
├── launch/
│   ├── rover.launch.py       # Robot + RealSense D455 + RViz
│   ├── slam.launch.py        # RTAB-Map SLAM
│   └── nav2.launch.py        # Nav2 navigation stack
├── rviz/
│   └── rover.rviz            # Pre-configured RViz2 layout
├── scripts/
│   └── teleop_key.py         # Keyboard teleoperation script
└── urdf/
    └── rover.xacro           # Robot description (no Gazebo plugins)
```

---

## Troubleshooting

**Camera not detected**
```bash
rs-enumerate-devices    # check USB 3.x connection
ros2 topic list | grep d455    # confirm topics are publishing
```

**TF tree errors in SLAM**
```bash
ros2 run tf2_tools view_frames    # visualize full TF tree
ros2 run tf2_ros tf2_echo odom base_link    # check odom->base_link
```

**colcon build fails**
```bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install 2>&1 | cat    # see full error
```

**RViz shows no robot model**
```bash
ros2 topic echo /robot_description    # confirm URDF is published
```

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
