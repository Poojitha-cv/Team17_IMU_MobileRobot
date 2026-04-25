# 🤖 Team 17 — IMU Readings using a Mobile Robot (Simulation)

**Course:** UE23CS343BB7 — Mobile and Autonomous Robot  
**Project:** Demonstrate IMU Readings using a Mobile Robot (Simulation)  
**Tools:** ROS 2 Humble · Gazebo Classic 11 · RViz2 · Python 3 (rclpy)

---

## 👥 Team Members & Contributions

| Member | Role |
|---|---|
| Poojitha CV | Robot Simulation Design + Tilt Detection |
| Pragna | ROS2 Integration + Motion Classification |
| Nithya | IMU Data Pipeline + Orientation Tracking |

---

## 📁 Project Structure

```
Team17_IMU_MobileRobot/
├── README.md
├── .gitignore
└── src/
    └── imu_robot/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── imu_robot
        ├── config/
        │   └── params.yaml
        ├── imu_robot/
        │   ├── __init__.py
        │   ├── tilt_detector.py       ← Poojitha
        │   ├── motion_classifier.py   ← Pragna
        │   ├── imu_reader.py          ← Nithya
        │   ├── imu_graph.py           ← Nithya
        │   └── teleop.py
        ├── launch/
        │   └── imu_demo.launch.py     ← Pragna
        ├── urdf/
        │   └── imu_robot.urdf.xacro   ← Poojitha
        ├── rviz/
        │   └── imu_robot.rviz         ← Poojitha
        ├── worlds/
        │   └── ramp_world.world       ← Poojitha
        └── test/
            ├── test_copyright.py
            ├── test_flake8.py
            └── test_pep257.py
```

---

## 🏗️ System Architecture

```
Gazebo Simulation
      │
      │  /imu/data  (sensor_msgs/Imu @ 50 Hz)
      ▼
┌─────────────────────────────────────────┐
│              ROS 2 Nodes                │
│                                         │
│  tilt_detector   ──► /cmd_vel (stop)    │  ← Poojitha
│  motion_classifier                      │  ← Pragna
│  imu_reader      ──► imu_log.csv        │  ← Nithya
│  imu_graph       ──► live dashboard     │  ← Nithya
└─────────────────────────────────────────┘
      │
      ▼
    RViz2
```

**TF Tree:**
```
odom → base_link → imu_link
                 → left_wheel
                 → right_wheel
                 → caster_wheel
```

---

## 📡 ROS2 Topics

| Topic | Type | Direction |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | Published by Gazebo, subscribed by all nodes |
| `/cmd_vel` | `geometry_msgs/Twist` | Published by `tilt_detector` (emergency stop) |
| `/odom` | `nav_msgs/Odometry` | Published by diff-drive plugin |

---

## ⚙️ Prerequisites

- Ubuntu 22.04
- ROS 2 Humble (full desktop)
- Gazebo Classic 11
- Python 3 with `matplotlib`

```bash
sudo apt install ros-humble-gazebo-ros-pkgs \
                 ros-humble-robot-state-publisher \
                 ros-humble-rviz2 \
                 ros-humble-xacro \
                 python3-matplotlib
```

---

## 🔧 Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws_imu
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Run

**Terminal 1 — Launch Gazebo + all nodes + RViz2**
```bash
ros2 launch imu_robot imu_demo.launch.py
```

**Terminal 2 — Drive robot onto ramp**
```bash
ros2 run imu_robot teleop
# Press ENTER when prompted
```

**Terminal 3 — Check IMU CSV log**
```bash
cat ~/imu_logs/imu_log.csv
```

**Terminal 4 — Live graph dashboard (optional)**
```bash
ros2 run imu_robot imu_graph
```

---

## 👩‍💻 Poojitha CV — Robot Simulation Design + Tilt Detection

**Role:** Simulation Modeling & Stability Detection

### Robot URDF/Xacro — `imu_robot.urdf.xacro`

Designed a differential-drive robot with the following links:

| Link | Shape | Mass |
|---|---|---|
| `base_link` | Box 0.4×0.3×0.1 m | 5.0 kg |
| `left_wheel` | Cylinder r=0.07 m | 0.5 kg |
| `right_wheel` | Cylinder r=0.07 m | 0.5 kg |
| `caster_wheel` | Sphere r=0.035 m | 0.2 kg |
| `imu_link` | Box 0.05×0.05×0.02 m | 0.01 kg |

### IMU Gazebo Plugin Tuning

| Parameter | Value |
|---|---|
| `update_rate` | 50 Hz |
| Angular velocity noise (stddev) | 0.01 |
| Linear acceleration noise (stddev) | 0.01 |
| Orientation noise (stddev) | 0.005 |
| Output topic | `/imu/data` |

### Tilt Detector Node — `tilt_detector.py`

Subscribes to `/imu/data`, converts quaternion → roll/pitch using exact formula with sinp clamp, applies LPF, and uses hysteresis to trigger emergency stop.

**Logic:**
- Tilt **detected** → `max(|roll|, |pitch|) > 15°` for 5 frames (0.1s) → publishes `/cmd_vel = 0`
- Tilt **cleared** → `max(|roll|, |pitch|) < 10°` for 5 frames (0.1s) → robot resumes

**Threshold Justification:**  
Stationary calibration shows noise ±0.8° → dead band = 6× noise → enter=15°, exit=10°

**Parameters (`params.yaml`):**

| Parameter | Value |
|---|---|
| `tilt_enter` | 15.0° |
| `tilt_exit` | 10.0° |
| `alpha` (LPF) | 0.2 |
| `imu_topic` | `/imu/data` |
| Hold frames | 5 (0.1s at 50Hz) |

**Run individually:**
```bash
ros2 run imu_robot tilt_detector
```

---

## 👩‍💻 Pragna — ROS2 Integration + Motion Classification

**Role:** System Integration & Motion State Detection

### ROS2 Package Setup

Created the ROS2 ament_python package using `ros2 pkg create`. Configured `package.xml` and `setup.py` with all dependencies: `rclpy`, `sensor_msgs`, `geometry_msgs`, `gazebo_ros`, `robot_state_publisher`, `xacro`, `rviz2`.

### Launch File — `imu_demo.launch.py`

Launches the full simulation pipeline in one command:
- Gazebo with `ramp_world.world`
- `robot_state_publisher` with processed URDF
- `spawn_entity.py` (robot spawned at x=-1.0, behind ramp)
- `tilt_detector` node with params
- `motion_classifier` node with params
- `imu_reader` node with params
- `imu_graph` node
- `RViz2` with config

### Motion Classifier Node — `motion_classifier.py`

Classifies robot motion state from IMU data using LPF + velocity integration + hysteresis.

**States:**

| State | Condition |
|---|---|
| `STATIONARY` | `\|lin_x\| < 0.15` and `\|est_vel\| < 0.08` and `\|ang_z\| < 0.10` |
| `MOVING_STRAIGHT` | Linear motion detected |
| `TURNING` | Angular motion detected |
| `MOVING_AND_TURNING` | Both detected |

**Noise Calibration:**

| Channel | Noise Floor | Threshold (2×) |
|---|---|---|
| `lin_x` | ±0.15 m/s² | 0.15 m/s² |
| `ang_z` | ±0.05 rad/s | 0.10 rad/s |

**Watchdog:** Timer checks `/imu/data` timeout > 2s and logs error.

**Run individually:**
```bash
ros2 run imu_robot motion_classifier
```

---

## 👩‍💻 Nithya — IMU Data Pipeline + Orientation Tracking

**Role:** Data Processing & Analysis

### IMU Reader Node — `imu_reader.py`

Subscribes to `/imu/data` and runs a full 9-channel LPF pipeline.

**Pipeline:**

| Channel | Processing |
|---|---|
| Orientation (roll, pitch, yaw) | Quaternion → Euler with sinp clamp, then LPF (alpha=0.2) |
| Linear acceleration (x, y, z) | LPF (alpha=0.2) |
| Angular velocity (x, y, z) | LPF (alpha=0.2) |

**CSV Logger:**
- Logs to `~/imu_logs/imu_log.csv` at 5 Hz (every 10 messages)
- Columns: `timestamp, roll_deg, pitch_deg, yaw_deg, lin_acc_x, lin_acc_y, lin_acc_z, ang_vel_x, ang_vel_y, ang_vel_z`
- Terminal print at 1 Hz (every 50 messages)

**Watchdog:** Fires error if no `/imu/data` for more than 2 seconds.

### Live Graph — `imu_graph.py`

4-panel live matplotlib dashboard:

| Panel | Content |
|---|---|
| ① Orientation | Roll / Pitch / Yaw (degrees), scrolling 200 samples |
| ② Linear Acceleration | X / Y / Z (m/s²) |
| ③ Angular Velocity | X / Y / Z (rad/s) |
| ④ 2D Robot Path | Dead-reckoning from IMU with gravity compensation |

**Run individually:**
```bash
ros2 run imu_robot imu_graph
```

---

## 📊 Node Summary

| Node | Subscribes | Publishes | Logs |
|---|---|---|---|
| `tilt_detector` | `/imu/data` | `/cmd_vel` | Terminal warnings |
| `motion_classifier` | `/imu/data` | — | Terminal state changes |
| `imu_reader` | `/imu/data` | — | `~/imu_logs/imu_log.csv` |
| `imu_graph` | `/imu/data` | — | Live matplotlib window |

---

## 📄 License

Apache License 2.0

---

*ROS 2 Humble · Gazebo Classic 11 · Course UE23CS343BB7*
