# WALL-E ROBOT — COMMAND CHEAT SHEET

**Platform:** ROS 2 Humble
**Robot computer:** Orange Pi 5
**Middleware:** Cyclone DDS
**ROS Domain ID:** `0`

---

## 1. Build the ROS 2 Workspace

### 1.1 First build / build all packages

```bash
colcon build \
  --base-paths wall-e-robot wall-e-robot/diffdrive_arduino
```

### 1.2 Build only the `wall-e-robot` package

```bash
colcon build \
  --packages-select wall-e-robot \
  --symlink-install
```

### 1.3 Clean and rebuild from scratch

Use this when you change package structure, `CMakeLists.txt`, dependencies, or hit a hard-to-diagnose build error.

```bash
rm -rf build/ install/ && colcon build \
  --base-paths wall-e-robot wall-e-robot/diffdrive_arduino \
  --symlink-install
```

### 1.4 Source the workspace and launch the robot

```bash
source install/setup.bash
ros2 launch wall-e-robot launch_robot.launch.py
```

---

## 2. Install Cyclone DDS

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

> Note: the correct package name is `ros-humble-rmw-cyclonedds-cpp`.

---

## 3. Run on the Orange Pi (robot computer)

```bash
export ROS_DOMAIN_ID=0
source install/setup.bash
ros2 launch wall-e-robot launch_robot.launch.py
```

---

## 4. Run on Ubuntu (teleoperation)

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.2 -p turn:=0.5
```

---

## 5. Start RViz

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
rviz2 -d config/nav2.rviz
```

---

## 6. Check Odometry

**Orientation**

```bash
ros2 topic echo /diff_drive_controller/odom --once | grep -A 5 "orientation:"
```

**Position**

```bash
ros2 topic echo /diff_drive_controller/odom --once | grep -A 3 "position:"
```

---

## 7. LiDAR — RPLIDAR

**Terminal 1 (ter1) — LiDAR**

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
ros2 run rplidar_ros rplidar_node \
  --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser_frame
```

---

## 8. Robot / SLAM / Nav2 Terminals

**Terminal 2 (ter2) — Robot**

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
source install/setup.bash
ros2 launch wall-e-robot launch_robot.launch.py
```

**Terminal 3 (ter3) — SLAM (mapping only)**

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
source install/setup.bash
ros2 launch wall-e-robot slam.launch.py use_sim_time:=false
```

**Terminal 4 (ter4) — Nav2**

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
source install/setup.bash
ros2 launch wall-e-robot nav_bringup.launch.py
```

---

## 9. Which Terminals to Run

### 9.1 Normal navigation

Run **ter1 + ter2**.

### 9.2 Building a map (SLAM)

Run **ter1 + ter3**, then on the **Ubuntu PC**, run teleop to drive the robot around and build the map, while also opening **RViz** to observe the map as it's built.

```
Orange Pi:  ter1 (LiDAR) + ter3 (SLAM)
Ubuntu PC:  teleop_twist_keyboard  +  rviz2
```

---

## 10. Save Map

### 10.1 Save map using SLAM Toolbox

```bash
ros2 service call \
  /slam_toolbox/save_map \
  slam_toolbox_msgs/srv/SaveMap \
  "{name: {data: 'my_map2'}}"
```

### 10.2 Save map using Nav2 Map Saver

Install if needed:

```bash
sudo apt update
sudo apt install ros-humble-nav2-map-server
```

Then:

```bash
ros2 run nav2_map_server map_saver_cli -f my_map2
```

---

## 11. Multi-Waypoint Navigation

```bash
python3 multi_waypoint_nav.py \
  --waypoints "1.3059,0.1227,0.0 | 1.8084,0.9360,0.0" \
  --delay 5.0
```

**Waypoint format:** `x, y, yaw`

---

## 12. Quick Reference

```bash
# Environment
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Build
colcon build --base-paths wall-e-robot wall-e-robot/diffdrive_arduino --symlink-install

# Robot
ros2 launch wall-e-robot launch_robot.launch.py

# SLAM (mapping)
ros2 launch wall-e-robot slam.launch.py use_sim_time:=false

# Nav2
ros2 launch wall-e-robot nav_bringup.launch.py

# RViz
rviz2 -d config/nav2.rviz

# Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.5

# LiDAR
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=115200 -p frame_id:=laser_frame
```

> **Tip:** Add `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` to `~/.bashrc` so you don't need to `export` them in every new terminal. Still run `source install/setup.bash` after every workspace build.