# OriginCar — 21st National College Student Intelligent Car Competition (D-Robotics Category) System Architecture

> **Team**: 杭电地瓜2队
> **School**: Hangzhou Dianzi University
> **Team Leader**: 黄启超
> **RDK Model**: RDK X5 (arm64)
> **Competition Mode**: Fully Autonomous
> **Submission**: Public
> **Date**: 2026-07-14

---

## Table of Contents

1. [Overall System Architecture](#1-overall-system-architecture)
2. [Hardware Selection & Connectivity](#2-hardware-selection--connectivity)
3. [Software System Design](#3-software-system-design)
4. [Key Task Implementation Strategy](#4-key-task-implementation-strategy)
5. [Competition Task & Rule Adaptation](#5-competition-task--rule-adaptation)
6. [System Startup & Deployment](#6-system-startup--deployment)
7. [Path Point Calibration Tool](#7-path-point-calibration-tool)
8. [Technical Highlights & Innovation](#8-technical-highlights--innovation)

---

## 1. Overall System Architecture

### 1.1 System Overview

OriginCar is a fully autonomous driving robot built on the OriginCar Ackermann chassis platform. The system adopts an **STM32 lower computer + RDK X5 upper computer** architecture. The software stack is based on **ROS2 Humble** and operates in a single unified `dev_ws` workspace, integrating LiDAR-based SLAM + AMCL localization, a custom Pure Pursuit controller with LiDAR + YOLO cone fused obstacle avoidance, BPU-accelerated QR code detection, cloud-based VLM mark sign recognition, and I2C voice broadcast — enabling the complete autonomous execution of the competition mission within a 180-second time limit.

### 1.2 System Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────────────┐
│                        Competition Mission Layer                         │
│                                                                          │
│  ┌───────────────────────────────────────────────────────────────────┐   │
│  │                     TaskMaster (C++ Node)                         │   │
│  │                                                                   │   │
│  │   FSM (10 Hz):                                                    │   │
│  │     WAIT_TF → IDLE → NAV_P_QR_B → RING_NAV → NAV_TO_P → PARK → DONE  │
│  │                                                                   │   │
│  │   Custom RPP Controller (20 Hz):                                  │   │
│  │     Pure Pursuit + Adaptive Lookahead + Reacquire                 │   │
│  │     + LiDAR + YOLO Cone Fused Obstacle Avoidance                  │   │
│  └───────────────┬──────────────────────┬────────────────────────────┘   │
│                  │ /cmd_vel (Twist)     │ /announcement, /capture_trigger│
│                  ▼                      ▼                                │
│  ┌──────────────────────────┐  ┌────────────────────────────────────┐   │
│  │  connect_to_pc           │  │  origincar_broadcast               │   │
│  │  HTTP → PC VLM           │  │  I2C Yabo TTS Module               │   │
│  │  /image_description      │  │  Queue + Dedup Broadcast           │   │
│  └──────────────────────────┘  └────────────────────────────────────┘   │
└───────────────────────────────────┬──────────────────────────────────────┘
                                    │
┌───────────────────────────────────┼──────────────────────────────────────┐
│                      Navigation & Localization Layer                     │
│  ┌────────────────────────────────┼──────────────────────────────────┐   │
│  │           Nav2 Stack (localization only)                          │   │
│  │  ┌──────────────┐  ┌──────────────┐                              │   │
│  │  │ AMCL (2D)    │  │ Map Server   │                              │   │
│  │  │ localization │  │ + Keepout    │                              │   │
│  │  └──────┬───────┘  └──────────────┘                              │   │
│  │         │ /amcl_pose, map→odom_combined TF                        │   │
│  └─────────┼────────────────────────────────────────────────────────┘   │
└─────────────┼───────────────────────────────────────────────────────────┘
              │
┌─────────────┼───────────────────────────────────────────────────────────┐
│                           Sensor Fusion Layer                           │
│  ┌──────────┴──────────────────────────────────────────────────────┐   │
│  │              robot_localization (EKF Node)                       │   │
│  │              /odom + /imu/data → /odom_combined (30 Hz)         │   │
│  │              odom_combined → base_footprint TF                   │   │
│  └─────────────┬──────────────────────┬─────────────────────────────┘   │
│                │                      │                                  │
└────────────────┼──────────────────────┼──────────────────────────────────┘
                 │                      │
┌────────────────┼──────────────────────┼──────────────────────────────────┐
│                             Driver Layer                                 │
│  ┌─────────────┴───────┐  ┌───────────┴────────────┐                    │
│  │  origincar_base     │  │  lslidar_driver         │                   │
│  │  STM32 Serial +     │  │  LSN10 LiDAR Driver     │                   │
│  │  Ackermann Converter│  │  /scan (LaserScan)      │                   │
│  │  /odom, /imu/data   │  │                         │                   │
│  └──────────┬──────────┘  └────────────────────────┘                    │
│             │ /dev/ttyACM0 115200 bps                                     │
└─────────────┼────────────────────────────────────────────────────────────┘
              │
┌─────────────┴────────────────────────────────────────────────────────────┐
│                          Perception Layer                                │
│  ┌─────────────────────┐  ┌──────────────────────┐  ┌───────────────┐   │
│  │ Aurora930 Camera    │  │ qr_bpu_detector      │  │ racing_obs    │   │
│  │ Depth Camera Driver │  │ BPU Infer + ZBar ROI │  │ YOLO Cone     │   │
│  │ /aurora/rgb/image_raw│  │ /qr_direction         │  │ /racing_obs...│   │
│  └─────────────────────┘  └──────────────────────┘  └───────────────┘   │
└──────────────────────────────────────────────────────────────────────────┘
              │
┌─────────────┴────────────────────────────────────────────────────────────┐
│                            Hardware Layer                                │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌───────────────────────┐   │
│  │ STM32 MCU│  │ LSN10    │  │ Aurora930│  │ Yabo I2C TTS Module   │   │
│  │ +MPU6050 │  │ LiDAR    │  │ Camera   │  │                       │   │
│  │ +Motor   │  │          │  │          │  │                       │   │
│  │ +Servo   │  │          │  │          │  │                       │   │
│  └──────────┘  └──────────┘  └──────────┘  └───────────────────────┘   │
└──────────────────────────────────────────────────────────────────────────┘
```

### 1.3 Data Flow

```
Aurora930 Camera                   LSN10 LiDAR
    │ /aurora/rgb/image_raw            │ /scan
    ├──────────────┬──────────┐        ├────────────────────┐
    ▼              ▼          ▼        ▼                    ▼
qr_bpu_detector  connect_to_pc      AMCL              TaskMaster
BPU Infer         HTTP→PC VLM     localization       LiDAR OA
    │              │                  │                    │
    ▼              ▼                  │ map→odom_combined  │
/qr_direction  /image_description     │ TF                 │
(String)       (String)              │                    │
    │              │                  │                    │
    └──────────────┼──────────────────┼────────────────────┘
                   │                  │            ▲
                   ▼                  ▼            │
          origincar_broadcast    TaskMaster    /racing_obstacle_detection
          I2C TTS Broadcast      (FSM+Controller)  YOLO Cone
                   │                  │            │
                   │                  ▼            │
                   │           /cmd_vel (Twist)     │
                   │                  │            │
                   │          origincar_base       │
                   │          + Ackermann Conv     │
                   │                  │            │
                   │          STM32 (Serial)       │
                   │                  │            │
                   │          Motor + Servo        │
                   └──────────────────┘
                              (Voice Output)
```

---

## 2. Hardware Selection & Connectivity

### 2.1 Hardware List

| Module | Model | Function | Interface |
|:---|:---|:---|:---|
| Upper Computer | **RDK X5** (arm64, Ubuntu 22.04) | ROS2 runtime, BPU QR inference, YOLO cone detection, navigation, task orchestration | — |
| Lower Computer | **STM32** (HAL Library) | Motor & servo control, MPU6050 IMU data acquisition, wheel odometry | USB Serial `/dev/ttyACM0`, 115200 bps |
| IMU | **MPU6050** (integrated on STM32 board) | 3-axis gyroscope + 3-axis accelerometer | Via STM32 serial frame |
| LiDAR | **LSN10** (LeiShen LS Series) | 360° 2D laser ranging: SLAM mapping, AMCL localization, real-time obstacle avoidance | UART |
| Camera | **Aurora930** (Depth Camera, RGB used only) | Competition field image capture for QR detection and VLM mark sign recognition | USB |
| Voice Module | **Yabo Intelligent TTS Module** | Text-to-speech broadcast for QR direction result and VLM mark sign recognition | I2C |
| Chassis | **OriginCar Ackermann** | Ackermann steering, 5-link independent suspension | Serial command via STM32 |

### 2.2 Hardware Connection Diagram

```
                           ┌───────────────────────────┐
                           │         RDK X5             │
                           │      (ROS2 Humble)         │
                           │                            │
              USB ─────────┤ Aurora930 Camera           │
              UART ────────┤ LSN10 LiDAR                │
              USB-UART ────┤ /dev/ttyACM0 → STM32       │
              I2C ─────────┤ Yabo TTS Module            │
                           │                            │
                           └───────────────────────────┘
                                        │
                          USB Serial (115200 bps)
                                        │
                           ┌────────────┴──────────┐
                           │       STM32 MCU        │
                           │                        │
                           │  ┌──────────────────┐  │
                           │  │ MPU6050 (IMU)    │  │
                           │  └──────────────────┘  │
                           │                        │
                           │  Motor Driver ──── DC Motor (Rear wheel drive)   │
                           │  Servo Driver ──── Servo (Front wheel steering)  │
                           └────────────────────────┘
```

### 2.3 Serial Communication Protocol

Communication between RDK X5 and STM32 uses a custom 24-byte binary protocol with XOR checksum.

- **Baud Rate**: 115200 bps
- **Receive Frame** (STM32→RDK, 24 bytes):
  - Header(1) + StopFlag(1) + VelocityX(2) + VelocityY(2) + VelocityZ(2) + AccelX(2) + AccelY(2) + AccelZ(2) + GyroX(2) + GyroY(2) + GyroZ(2) + Voltage(2) + Checksum(1) + Tail(1)
- **Send Frame** (RDK→STM32, 11 bytes):
  - Header(1) + Reserved(2) + Speed(2) + Reserved(2) + Steering(2) + Checksum(1) + Tail(1)
- **Command Flow**: `/cmd_vel` (Twist) → `cmd_vel_to_ackermann_drive` → `ackermann_cmd` → `origincar_base` → STM32 Serial
- **Ackermann Mode**: `akmcar=true` — Twist `linear.x` → vehicle speed, Twist `angular.z` → steering angle

---

## 3. Software System Design

### 3.1 Software Stack

| Layer | Technology | Details |
|:---|:---|:---|
| OS | Ubuntu 22.04 (RDK X5 arm64) | — |
| Middleware | ROS2 Humble | colcon build, --symlink-install |
| Build | CMake (C++), setuptools (Python) | Dual-platform: x86 dev / arm64 deploy |
| Localization | AMCL (Nav2) + EKF (robot_localization) | Map-based localization with odom+IMU fusion |
| Motion Control | Custom Pure Pursuit (RPP) | 20 Hz, adaptive lookahead, Catmull-Rom smoothing |
| AI Inference | BPU (RDK X5) — QR detection | TensorRT .bin model, zero CPU overhead |
| Vision | OpenCV, ZBar | QR ROI decoding |
| Cone Detection | YOLOv5s/v8s (TensorRT, BPU) | racing_obstacle_detection_yolo (upstream verbatim) |
| VLM | Aliyun DashScope (qwen-vl-plus) | Via HTTP REST from PC, result bridged to `/image_description` |
| Voice | Yabo I2C TTS → I2C | origincar_broadcast: multi-topic queue + dedup |
| Serial | serial (custom fork) | Cross-platform serial with timeout |
| Coordinate Frames | TF2 (static transforms only) | No URDF, no robot_state_publisher |

### 3.2 ROS2 Package Architecture

```
dev_ws/
├── src/
│   ├── ackermann_msgs/                        # Ackermann drive message definitions (upstream)
│   ├── serial/                                # Cross-platform serial communication library
│   │
│   ├── origincar_base/                        # Chassis driver & static TF
│   │   ├── src/origincar_base.cpp             #   STM32 serial I/O, IMU parsing, odometry integration
│   │   ├── src/cmd_vel_to_ackermann_drive.cpp #   Twist → AckermannDriveStamped converter
│   │   ├── src/static_tf_node.cpp             #   base_footprint→base_link→laser static TF
│   │   ├── src/odom_tf_node.cpp               #   Simple odom→base_footprint TF (EKF-less fallback)
│   │   ├── config/ekf.yaml                    #   EKF: odom + imu → odom_combined (30 Hz)
│   │   ├── config/slam_mapping.yaml           #   slam_toolbox online async mapping
│   │   ├── launch/base_serial.launch.py       #   Serial driver + optional Ackermann converter
│   │   ├── launch/slim_bringup.launch.py      #   Minimal bringup (no URDF, light RViz)
│   │   ├── map/                               #   SLAM maps: race_modify, race_keepout
│   │   └── param/                             #   Nav2 param: param_mini_akm.yaml
│   │
│   ├── origincar_msg/                         # Custom ROS2 message definitions
│   │
│   ├── origincar_bringup/                     # Unified launch orchestration
│   │   ├── launch/base.launch.py              #   Chassis + Ackermann + EKF + Static TF
│   │   ├── launch/perception.launch.py        #   Camera + QR BPU + YOLO Cone
│   │   ├── launch/mission.launch.py           #   TaskMaster + PC Bridge + Broadcast
│   │   ├── launch/competition.launch.py       #   All-in-one entry (all params exposed)
│   │   ├── launch/task.launch.py              #   Terminal 1: TaskMaster only
│   │   ├── launch/vehicle_stack.launch.py     #   Terminal 2: Base + Localization + Perception + Speech
│   │   └── config/competition.yaml            #   TaskMaster default parameters
│   │
│   ├── origincar_task/                        # Competition task state machine
│   │   ├── src/task_master.cpp                #   FSM (10Hz) + Custom RPP Controller (20Hz)
│   │   ├── include/origincar_task/            #   task_master.hpp, mission_policy.hpp
│   │   ├── config/waypoints_flowpath_custom_rpp.yaml  #   Primary waypoint file
│   │   ├── config/waypoints.yaml              #   Legacy coordinate system waypoints
│   │   └── scripts/                           #   imu_calibrate, odom_calibrate, generate_map_semantics
│   │
│   ├── qr_bpu_detector/                       # BPU-accelerated QR detection
│   │   ├── src/qr_bpu_detector_node.cpp       #   BPU model inference (TensorRT .bin)
│   │   ├── src/qr_roi_decoder_node.cpp        #   ROI crop + ZBar continuous decode
│   │   ├── launch/qr_bpu_minimal.launch.py    #   Race minimal: BPU infer + ROI decode only
│   │   └── config/                            #   BPU model (.bin) + class list
│   │
│   ├── racing_obstacle_detection_yolo/        # YOLO cone detection (verbatim upstream)
│   │   ├── src/sample.cpp                     #   BPU inference main
│   │   ├── src/image_utils.cpp                #   Image pre/post processing
│   │   └── src/parser.cpp                     #   Model output parsing
│   │
│   ├── connect_to_pc/                         # Vehicle→PC HTTP bridge
│   │   ├── connect_to_pc/car_pc_bridge_node.py  # HTTP image sender + callback receiver
│   │   └── launch/car_pc_bridge.launch.py
│   │
│   ├── origincar_broadcast/                   # Voice broadcast manager
│   │   ├── origincar_broadcast/broadcast_manager_node.py  # Multi-topic queue + dedup
│   │   ├── origincar_broadcast/speech_client.py           # I2C speech client
│   │   └── launch/broadcast.launch.py
│   │
│   ├── lslidar_driver/                        # LSN10 LiDAR driver
│   ├── lslidar_msgs/                          # LiDAR custom messages
│   └── utils/                                 # Image transport utility node
│
├── docs/
│   ├── PROJECT_FRAMEWORK.md                   # Project framework & architecture reference
│   └── origincar_plan.md                      # Development plan & progress tracking
├── CHANGELOG.md
└── README.md                                  # This document
```

### 3.3 Three-Terminal Architecture

The competition runtime uses a three-terminal process isolation strategy to prevent single-process failure cascading and manage RDK X5 memory pressure:

```
┌─────────────────┐    ┌──────────────────────┐    ┌───────────────────┐
│   Terminal 1    │    │      Terminal 2       │    │    Terminal 3      │
│   task.launch.py│    │ vehicle_stack.launch.py│    │ car_pc_bridge.     │
│                 │    │                       │    │ launch.py          │
│                 │    │                       │    │                    │
│  TaskMaster     │    │  origincar_base       │    │  connect_to_pc     │
│  (FSM+Controller)│   │  + Ackermann Conv     │    │  HTTP→PC VLM       │
│                 │    │  Static TF            │    │                    │
│                 │    │  EKF                  │    │                    │
│                 │    │  AMCL + Map Server    │    │                    │
│                 │    │  LiDAR Driver         │    │                    │
│                 │    │  Aurora930 Camera     │    │                    │
│                 │    │  QR BPU Detector      │    │                    │
│                 │    │  YOLO Cone Detection  │    │                    │
│                 │    │  Broadcast Manager    │    │                    │
└─────────────────┘    └──────────────────────┘    └───────────────────┘
```

Unified single-terminal entry for convenience:
```bash
ros2 launch origincar_bringup competition.launch.py
```

### 3.4 TF Tree

```
map ──→ odom_combined ──→ base_footprint ──→ base_link ──→ laser
(AMCL)    (EKF, 30Hz)     (TF static, 0,0,0)  (0.092,0,0)  (x=laser_x, z=0.102)
                                                             yaw=laser_yaw
```

- `map → odom_combined`: Published by AMCL (global localization correction)
- `odom_combined → base_footprint`: Published by EKF node (fused odom + IMU, 30 Hz)
- `base_footprint → base_link`: Static TF (X offset for rear-axle centering, 0.092m)
- `base_link → laser`: Static TF with runtime-tunable yaw via `laser_yaw` parameter (LiDAR calibration)
- **No URDF, no robot_state_publisher** — all transforms are static TF nodes for minimal memory footprint
- EKF starts 4 seconds after base serial to avoid DDS initialization memory peaks

### 3.5 Core Topics

| Topic | Type | Publisher | QoS | Description |
|:---|:---|:---|:---|:---|
| `/odom` | `nav_msgs/Odometry` | `origincar_base` | Default | Wheel odometry from STM32 serial |
| `/imu/data` | `sensor_msgs/Imu` | `origincar_base` | Default | Raw IMU from MPU6050 (via STM32) |
| `/odom_combined` | `nav_msgs/Odometry` | `ekf_node` | Default | EKF-fused odometry (30 Hz) |
| `/scan` | `sensor_msgs/LaserScan` | `lslidar_driver` | Sensor Data | 2D LiDAR scan for AMCL + obstacle avoidance |
| `/aurora/rgb/image_raw` | `sensor_msgs/Image` | Aurora930 camera | Sensor Data | Raw RGB image for QR detection and VLM |
| `/qr_direction` | `std_msgs/String` | `qr_roi_decoder_node` | Default | QR content → direction (odd→CW, even→CCW) |
| `/racing_obstacle_detection` | `ai_msgs/PerceptionTargets` | YOLO cone node | Sensor Data | Cone detection results (bounding boxes + confidence) |
| `/image_description` | `std_msgs/String` | `connect_to_pc` | Default | VLM mark sign recognition result from PC |
| `/announcement` | `std_msgs/String` | TaskMaster | Default | Text to broadcast via I2C TTS |
| `/capture_trigger` | `std_msgs/Empty` | TaskMaster | Default | Capture signal to PC bridge at mark sign |
| `/cmd_vel` | `geometry_msgs/Twist` | TaskMaster | Default | Velocity command (20 Hz) to chassis |

---

## 4. Key Task Implementation Strategy

### 4.1 TaskMaster FSM

The TaskMaster is the central brain of the competition system, implemented as a single C++ node with two timing loops:

```
                   ┌──────────┐
                   │ WAIT_TF  │  Wait for map→base_footprint TF stability
                   │ (startup) │  (5 consecutive stable frames + timeout guard)
                   └────┬─────┘
                        │ TF stable for ≥5 frames
                   ┌────▼─────┐
                   │   IDLE   │  Wait 2s, announce "任务开始"
                   └────┬─────┘
                        │
                   ┌────▼──────────┐
                   │ NAV_P_QR_B    │  Cruise: P → QR scan area → B channel entry
                   │ (merged path) │  QR: parity-based direction lock (odd→CW, even→CCW)
                   │               │  QR deadline: auto-lock to default if no QR by deadline
                   └────┬──────────┘
                        │ Path complete (QR locked), seamless transition
                   ┌────▼─────┐
                   │ RING_NAV │  C zone ring loop (CW or CCW based on QR)
                   │           │  Mark sign capture at designated waypoints
                   │           │  LiDAR + YOLO cone fused obstacle avoidance
                   └────┬─────┘
                        │ Ring complete
                   ┌────▼─────┐
                   │ NAV_TO_P │  Return from B channel exit → P point
                   └────┬─────┘
                        │ P point reached
                   ┌────▼─────┐
                   │   PARK   │  Stop for 3s, announce "任务完成"
                   └────┬─────┘
                        │
                   ┌────▼─────┐
                   │   DONE   │  Terminal state
                   └──────────┘
```

**Per-phase 90s timeout**: any phase exceeding 90 seconds forces FAILED → PARK to prevent infinite looping.

### 4.2 Custom RPP Controller

The TaskMaster does NOT use Nav2 for motion control. Instead, it implements a 20 Hz custom Pure Pursuit controller with the following features:

| Feature | Description |
|:---|:---|
| **Adaptive Lookahead** | Shortens lookahead distance when upcoming curvature is high (≥0.5 rad cumulative yaw → 0.78×; ≥1.0 rad → 0.55×) |
| **Reacquire** | If the robot drifts off-path, jumps to the nearest remaining waypoint within a 20-point window |
| **Lookahead Progress Commit** | Commits passed waypoints when the lookahead target advances, enabling smooth continuous motion |
| **Precision Waypoints** | First/last 2 points, motion transition points, QR scan/capture/pause points use tight pass radii |
| **Per-Waypoint Gains** | Each waypoint has `speed_gain` and `angular_gain` for fine-grained speed/steering control |
| **QR Zone Slowdown** | `qr_scan_speed_gain=0.85` applied near QR scan waypoints (pending QR state only) |
| **Capture Zone Slowdown** | `capture_speed_gain=0.60` applied near capture waypoints |
| **Turn Gain** | `turn_speed_gain=0.50`, `turn_angular_gain=1.50` applied to waypoints with `angular_gain > 1.0` |
| **Path Trimming** | At state transition, trims the new path to start from the nearest point to the robot's current pose |

```
Control Loop (20 Hz):
  1. Lookup robot pose (map→base_footprint TF)
  2. Handle pause (if paused waypoint)
  3. Reacquire progress (if drifted)
  4. Advance passed waypoints
  5. Select adaptive lookahead target
  6. Compute speed (base × waypoint gain × zone gain)
  7. Compute Pure Pursuit curvature → angular velocity
  8. Apply LiDAR + YOLO cone fused obstacle avoidance
  9. Publish /cmd_vel
```

### 4.3 LiDAR + YOLO Cone Fused Obstacle Avoidance

The obstacle avoidance system fuses LiDAR range data with YOLO cone detection for reliable obstacle handling:

```
Sensor Input                    State Machine                 Control Output
─────────────                   ─────────────                 ──────────────
LiDAR /scan ─────┐
                 ├──→ Sector Analysis ──→ ObstacleState:
YOLO Cone ───────┘    front/left/right      ├── CLEAR: normal RPP control
                      min ranges             ├── AVOIDING: speed × slow_scale
                                             │            + angular bias
                                             ├── BACKUP_ONCE: reverse for
                                             │   backup_distance or timeout
                                             └── WAIT_CLEAR: zero velocity,
                                                 wait for cone timeout + hold

Fusion Rules:
  • Cone detection (confidence ≥ cone_confidence_threshold) is required to enter AVOIDING
  • Lidar-only obstacle within emergency distance → SAFETY STOP only (no avoidance)
  • No valid cone for > cone_detection_timeout_sec + obstacle_clear_hold_sec → REACQUIRE_PATH → CLEAR
  • Backup executed once per encounter; second backup prohibited → WAIT_CLEAR
  • Avoid direction: based on left/right range gap + cone image position
```

| Parameter | Default | Description |
|:---|:---|:---|
| `cone_confidence_threshold` | 0.35 | Minimum YOLO confidence to treat as a confirmed cone |
| `obstacle_backup_distance_threshold` | 0.20 m | Distance at which backup is triggered |
| `obstacle_avoid_distance` | 0.45 m | Distance at which avoidance steering begins |
| `obstacle_slow_distance` | 0.45 m | Distance at which speed reduction begins |
| `obstacle_backup_distance` | 0.40 m | How far to back up |
| `obstacle_backup_timeout_sec` | 1.50 s | Max backup duration |
| `cone_detection_timeout_sec` | 0.25 s | Max cone data age before treating as stale |
| `obstacle_clear_hold_sec` | 0.40 s | Hold time after last cone detection before clearing state |
| `rear_clearance_distance` | 0.25 m | Minimum rear clearance required for backup |
| `avoid_proximity_max_angular_z` | 1.10 rad/s | Max angular velocity during avoidance |

### 4.4 QR Code Detection

Two-stage BPU + CPU pipeline:

```
Aurora930 Camera (/aurora/rgb/image_raw)
    │
    ▼
qr_bpu_detector_node
    │  BPU inference (TensorRT .bin model)
    │  14~15 FPS, avg_infer ~5.7~6.4 ms
    │  Output: detection bounding boxes around QR codes
    ▼
qr_roi_decoder_node
    │  Crop ROI from image using BPU bbox
    │  ZBar continuous decode
    │  Publish only valid numeric results (0-9999)
    ▼
/qr_direction (String)
    │
    ▼
TaskMaster
    │  Parse number → odd=CW, even=CCW
    │  Lock direction (QrDecisionState: PENDING→CW_LOCKED/CCW_LOCKED)
    │  Only accepts QR during NAV_P_QR_B state
    │  QR deadline waypoint forces default direction lock
```

### 4.5 Cloud VLM Mark Sign Recognition

```
TaskMaster reaches capture waypoint
    │
    ▼
/capture_trigger (Empty) → /person_trigger (Int32)
    │
    ▼
connect_to_pc (car_pc_bridge_node)
    │  Captures latest Aurora930 frame
    │  HTTP POST → PC VLM server (192.168.3.12:9999/predict)
    │  Receives text recognition result via HTTP callback
    ▼
/image_description (String)
    │
    ▼
origincar_broadcast → I2C → Yabo TTS Module → Voice Output
```

**Design rationale**: VLM computation is fully offloaded to a separate PC to preserve RDK X5 computational resources for real-time navigation and obstacle avoidance.

### 4.6 Voice Broadcast

```
/qr_direction ──────────┐
                        ├──→ origincar_broadcast
/image_description ─────┘    broadcast_manager_node
/announcement ──────────────→  • Multi-topic subscription
                               • Queue-based ordering (first-arrive-first-serve)
                               • Deduplication
                               • Serial I2C access
                                    │
                                    ▼
                              speech_client.py → I2C → Yabo TTS Module
```

### 4.7 Path Structure

The waypoint file `waypoints_flowpath_custom_rpp.yaml` defines four routes:

| Route | Waypoints | Description |
|:---|:---|:---|
| `p_to_qr_to_b_rpp` | ~33 | P → QR scan area → B channel entry (merged, forward-only) |
| `ring_cw` | ~88 | C zone ring loop clockwise |
| `ring_ccw` | ~88 | C zone ring loop counter-clockwise |
| `return_to_p_rpp` | ~16 | B channel exit → P point return path |

Each waypoint supports:
- `motion`: `forward` / `reverse`
- `pass_radius`: waypoint pass radius (tight for precision points)
- `speed_gain` / `angular_gain`: per-point control gains
- `pause`: pause duration in seconds
- `qr_scan`: QR scanning zone marker
- `qr_deadline`: force QR direction lock if no decision yet
- `capture`: trigger VLM mark sign capture

---

## 5. Competition Task & Rule Adaptation

### 5.1 Competition Field

```
┌───────────────────────────┐
│                           │   5m × 5m field
│    A Zone (Blue)          │   Outer fence boards only
│                           │   No physical internal walls
│    B Zone (Yellow)        │   ABC zones: ground color only
│                           │
│    ┌──────────┐           │
│    │ C Zone   │           │
│    │ (Green + │           │
│    │  Yellow) │           │
│    └──────────┘           │
│                           │
└───────────────────────────┘

LiDAR sees:   Outer fence boards only
LiDAR cannot see: B channel boundaries, C zone boundaries, yellow ring, all ground color
AMCL quality: Good at corners (L-shaped features) / Poor mid-wall (symmetric) / Poor at center
```

### 5.2 Physical vs. Semantic Constraint Separation

The AMCL localization map (`race_modify`) contains **only** the real LiDAR-visible outer fence walls. B channel boundaries, C zone forbidden areas, and yellow ring paths — which are all ground-color/semantic features invisible to LiDAR — are **NOT** drawn as walls in the localization map.

Instead, semantic constraints are enforced through:
- **Waypoint placement**: Path points guide the robot through the B channel
- **Keepout filter mask** (`race_keepout`): Virtual obstacle zones for forbidden areas
- **Speed zone differentiation**: Reduced speed (`speed_gain`) in C ring curves

### 5.3 Rule Compliance

| Rule | Implementation |
|:---|:---|
| 180s time limit | Optimized path density + `cruise_speed=0.85 m/s` + per-phase 90s timeout fallback |
| Obstacle collision +10s/次 | LiDAR + YOLO cone fused avoidance: emergency stop at 0.20m, slowdown at 0.45m, backup+wait for stuck recovery |
| QR scan for direction | BPU detection + ZBar decode → parity-based direction lock (odd→CW, even→CCW) |
| QR deadline fallback | Configurable `default_qr_direction` (cw/ccw) auto-lock if QR not detected by deadline waypoint |
| B channel traversal | Waypoint-guided path through B channel + keepout virtual boundary walls |
| C ring + mark sign | Full ring path (88 waypoints) with `capture: true` flags at designated positions → VLM recognition |
| Return to P and park | `return_to_p_rpp` route ending at P with `goal_tolerance=0.35m` |
| Voice broadcast | I2C Yabo TTS module with queue-based broadcast; no display screen required |
| DDS interference in shared network | `ROS_DOMAIN_ID=42` + FastDDS Discovery Server ready |

---

## 6. System Startup & Deployment

### 6.1 Build

```bash
cd dev_ws
source /opt/ros/humble/setup.bash

# x86 build (skip BPU packages)
colcon build --symlink-install \
  --packages-skip qr_bpu_detector racing_obstacle_detection_yolo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# RDK X5 full build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 6.2 Three-Terminal Launch (Competition)

```bash
# Recommended: single-script start
ros2 run origincar_bringup start_competition_3term.sh

# Equivalent manual commands:
# Terminal 1 — TaskMaster
ros2 launch origincar_bringup task.launch.py

# Terminal 2 — Vehicle Stack (chassis, TF, EKF, AMCL, LiDAR, camera, QR, YOLO, broadcast)
ros2 launch origincar_bringup vehicle_stack.launch.py

# Terminal 3 — PC VLM Bridge
ros2 launch connect_to_pc car_pc_bridge.launch.py
```

### 6.3 All-In-One Launch (Debug)

```bash
# All subsystems in one terminal with full parameter control
ros2 launch origincar_bringup competition.launch.py

# Override speed
ros2 launch origincar_bringup competition.launch.py cruise_speed:=0.90

# Override QR default direction
ros2 launch origincar_bringup competition.launch.py default_qr_direction:=ccw

# Start partial subsystems
ros2 launch origincar_bringup competition.launch.py \
  start_camera:=false start_qr:=false start_cone_detection:=false \
  start_task:=false start_pc_bridge:=false start_broadcast:=false
```

### 6.4 Key Runtime Parameters

```
TaskMaster:
  cruise_speed: 0.85 m/s         (base forward speed)
  qr_scan_speed_gain: 0.85       (speed scale in QR scan zone)
  turn_speed_gain: 0.50          (speed scale for sharp turns)
  turn_angular_gain: 1.50        (angular gain for sharp turns)
  capture_speed_gain: 0.60       (speed scale for VLM capture zone)
  max_angular_z: 1.80 rad/s      (steering rate limit)
  control_frequency: 20.0 Hz     (RPP controller rate)
  lookahead_dist: 0.22 m         (base lookahead distance)
  pass_radius: 0.35 m            (default waypoint pass radius)
  reverse_pass_radius: 0.30 m    (reverse waypoint pass radius)
  goal_tolerance: 0.35 m         (final waypoint tolerance)

LiDAR Extrinsics (base→laser):
  laser_x: -0.10 m               (X offset from base_link to laser)
  laser_yaw: 0.05 rad            (yaw calibration offset)
```

### 6.5 DDS Competition Network Isolation

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 daemon stop
```

---

## 7. Path Point Calibration Tool

An HTML-based manual waypoint editor is provided for efficient field calibration:

- **Location**: `origin_car/tools/manual-waypoint-editor-origin.html`
- **Features**:
  - Drag-and-drop waypoint placement on the SLAM map (`race_modify` / `race_keepout`)
  - Visual overlays: zones, forbidden areas, QR scan regions
  - Multi-route support (clockwise / counterclockwise)
  - Yaw angle visualization with draggable handles
  - Pass radius configuration per waypoint
  - Undo/redo history
  - YAML export compatible with `waypoints_flowpath_custom_rpp.yaml` format
- **Supporting maps**: `origin_car/tools/map/race_modify.png`, `race_keepout.png`

```
tools/
├── manual-waypoint-editor-origin.html  # Waypoint calibration tool
└── map/
    ├── race_modify.png                 # AMCL localization map overlay
    └── race_keepout.png                # Keepout mask overlay
```

---

## 8. Technical Highlights & Innovation

### 8.1 Unified Single-Workspace Architecture

All 14 runtime packages are consolidated into a single `dev_ws` workspace with a unified launch parameter contract. Every parameter is exposed at the top-level `competition.launch.py`, enabling single-command override of any runtime parameter without editing configuration files.

### 8.2 Three-Terminal Process Isolation

TaskMaster, vehicle infrastructure, and PC bridge run in separate terminal processes. If the PC bridge fails, the robot continues autonomous navigation; if TaskMaster encounters an error, the vehicle stack remains alive and stop-safe. This process isolation also mitigates RDK X5 memory pressure by spreading DDS discovery across staggered starts.

### 8.3 Physical-Semantic Constraint Separation

The AMCL localization map contains only real LiDAR-visible walls. Semantic constraints (B/C zone boundaries, forbidden areas) are enforced through waypoints, keepout masks, and speed zones. This prevents the common pitfall of AMCL matching scan data against non-existent "walls" drawn for semantic purposes.

### 8.4 LiDAR + Vision Fused Obstacle Avoidance

YOLO cone detection provides semantic confirmation before entering avoidance maneuvers, while LiDAR provides precise ranging. The fusion prevents false-positive avoidance from LiDAR artifacts while maintaining safety-critical LiDAR-only emergency stop capability.

### 8.5 Cloud-VLM Hybrid Architecture

VLM computation is fully offloaded to a separate PC via HTTP, preserving all RDK X5 BPU/CPU resources for real-time QR detection, YOLO cone detection, and motion control at 20 Hz.

### 8.6 Adaptive Pure Pursuit with Path Reacquisition

The custom RPP controller features curvature-adaptive lookahead distance, per-waypoint gain control, and automatic path reacquisition — enabling robust path tracking even after obstacle avoidance maneuvers cause trajectory deviations.

---

## Build Status

| Platform | Status | Notes |
|:---|:---|:---|
| x86 (dev) | 待验证 | Skip BPU packages (`qr_bpu_detector`, `racing_obstacle_detection_yolo`) |
| RDK X5 (deploy) | 待验证 | Full build with all packages |

---

## License

Proprietary — 第21届全国大学生智能汽车竞赛（地瓜机器人赛项）
