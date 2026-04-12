# AgileX Limo - Hardware Deployment

A ROS 2 Humble workspace for running the physical **AgileX Limo** robot. This setup completely bypasses Gazebo simulation to run SLAM (RTAB-Map) and Navigation (Nav2) directly on the physical hardware using the onboard camera, LiDAR, and motor controllers.

## ⚠️ Prerequisites

- Physical AgileX Limo robot
- Ubuntu 22.04 LTS
- ROS 2 Humble

---

## 1. Install ROS 2 Humble

*(If you already have ROS 2 Humble installed, skip to Step 2).*

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

## 2. Install Hardware & SLAM Dependencies

Install the core ROS packages, localization tools, teleop utilities, and the C++ serial libraries required for the Limo's motor drivers.

```bash
sudo apt install \
  build-essential cmake libboost-all-dev libserial-dev \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-rtabmap-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-diagnostic-updater \
  ros-humble-teleop-twist-keyboard \
  ros-humble-teleop-twist-joy \
  ros-humble-joy
```

---

## 3. Configure Serial Permissions (Crucial for Hardware)

To allow the ROS 2 driver to communicate with the physical Limo motor controllers, your user must be part of the `dialout` group.

```bash
sudo usermod -a -G dialout $USER
```
**⚠️ Note:** You *must* log out and log back in (or reboot) for this permission change to take effect before launching the robot.

---

## 4. Workspace Setup

Create the workspace and clone both the Limo hardware drivers and your custom Vanguard rover package.

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Create workspace
mkdir -p ~/rover_ws/src
cd ~/rover_ws/src

# 1. Clone the AgileX Limo drivers (hardware branch)
git clone -b hardware https://github.com/spabhut/limo_ros2.git

# 2. Clone the Project Vanguard hardware package (limo branch)
git clone -b limo https://github.com/spabhut/ProjectVanguard.git rover

# Install any remaining ROS package dependencies
cd ~/rover_ws
rosdep install --from-paths src -y --ignore-src

# Build the workspace
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

Open **3 separate terminals** (ensure the workspace is sourced in each if not auto-sourced).

### Terminal 1 — Hardware Bringup (Motors & Sensors)
Start the physical robot drivers. This talks to the chassis, camera, and LiDAR.
```bash
ros2 launch limo_base start_limo.launch.py
```

### Terminal 2 — SLAM & State Publisher
*(Wait for Terminal 1 to finish launching before running this).*

Start RTAB-Map to build the map using live hardware data. Ensure your `slam.launch.py` has `use_sim_time: False` and loads the robot URDF.
```bash
ros2 launch rover slam.launch.py
```

### Terminal 3 — Nav2 Navigation Stack
Start the Nav2 stack for autonomous routing and obstacle avoidance.
```bash
ros2 launch rover nav2.launch.py
```

### Optional — Manual Driving Test
To verify the motor controllers are receiving commands before turning on Nav2:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Key Hardware Topic Map

| Data | Typical Limo Topic Name |
|---|---|
| Depth image | `/camera/depth/image_raw` |
| RGB image | `/camera/color/image_raw` |
| Camera Info | `/camera/depth/camera_info` |
| Laser scan (LiDAR) | `/scan` |
| IMU | `/imu` |
| Odometry (Wheels) | `/odom` |
| Velocity command | `/cmd_vel` |
| Map | `/map` |

*(Note: Use `ros2 topic list` after running `start_limo.launch.py` to confirm exact camera topic names, as they can vary slightly depending on the specific camera model shipped with your Limo).*

---

## Troubleshooting

**Serial Port/Motor Error when launching `limo_base`**
* Did you remember to run `sudo usermod -a -G dialout $USER` and reboot?
* Check if the port exists: `ls /dev/ttyTHS*` or `ls /dev/ttyUSB*`.

**SLAM freezes or shows nothing in RViz**
* Check `slam.launch.py`. If `'use_sim_time': True` is present anywhere, change it to `False`. Simulation time will freeze real hardware setups indefinitely.

**No Image Data**
* Use `ros2 topic hz /camera/depth/image_raw` to see if the physical camera node is actually publishing frames.
