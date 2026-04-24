# IMU Robot — Demonstrate IMU Readings using a Mobile Robot

A ROS 2 (Humble) package that simulates a differential-drive mobile robot in Gazebo and demonstrates live IMU data processing.

---

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble (full desktop)
- Gazebo Classic 11
- Python `matplotlib`

```bash
sudo apt install ros-humble-gazebo-ros-pkgs \
                 ros-humble-robot-state-publisher \
                 ros-humble-rviz2 \
                 ros-humble-xacro \
                 python3-matplotlib
```

---

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws_imu
colcon build --symlink-install
source install/setup.bash
```

---

## Run

**Terminal 1 — Launch Gazebo + all nodes + RViz2**

```bash
ros2 launch imu_robot imu_demo.launch.py
```

**Terminal 2 — Drive robot onto the ramp**

```bash
ros2 run imu_robot teleop
# Press ENTER when prompted
```

**Terminal 3 — Check IMU CSV log**

```bash
cat ~/imu_logs/imu_log.csv
```

---

## Nodes

| Node | What it does |
|---|---|
| `imu_reader` | Filters 9 IMU channels, logs to CSV at 5 Hz |
| `tilt_detector` | Stops robot if tilt exceeds 15°, resumes below 10° |
| `motion_classifier` | Classifies motion: STATIONARY / MOVING / TURNING |
| `imu_graph` | Live 4-panel matplotlib dashboard + 2-D path |
| `teleop` | Auto-drives robot forward onto ramp |

---

## Key Topics

| Topic | Type |
|---|---|
| `/imu/data` | `sensor_msgs/Imu` |
| `/cmd_vel` | `geometry_msgs/Twist` |

---

*ROS 2 Humble · Gazebo Classic 11 · Apache License 2.0*
