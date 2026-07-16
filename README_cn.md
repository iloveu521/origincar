# OriginCar — 第21届全国大学生智能汽车竞赛（地瓜机器人赛项）系统架构

> **队伍**: 杭电地瓜2队
> **学校**: 杭州电子科技大学 (Hangzhou Dianzi University)
> **队长**: 黄启超
> **RDK 型号**: RDK X5 (arm64)
> **参赛模式**: 全自动 (Fully Autonomous)
> **方案可见性**: 公开
> **提交日期**: 2026-07-14

---

## 目录

1. [整体系统架构](#1-整体系统架构)
2. [硬件选型与连接方式](#2-硬件选型与连接方式)
3. [软件系统设计](#3-软件系统设计)
4. [关键任务实现策略](#4-关键任务实现策略)
5. [竞赛任务与规则适配](#5-竞赛任务与规则适配)
6. [系统启动与部署](#6-系统启动与部署)
7. [路径点标定工具](#7-路径点标定工具)
8. [技术亮点与创新](#8-技术亮点与创新)

---

## 1. 整体系统架构

### 1.1 系统概述

OriginCar 是一个基于 OriginCar 阿克曼底盘的完全自主驾驶机器人。系统采用 **STM32 下位机 + RDK X5 上位机** 架构，软件基于 **ROS2 Humble** 构建，在单一 `dev_ws` 工作空间中融合了 LiDAR SLAM + AMCL 定位、自定义 Pure Pursuit 控制器 + LiDAR/YOLO 锥桶融合避障、BPU QR 码检测、云端 VLM 标记牌识别和 I2C 语音播报，实现 180 秒限时内全自主完成任务。

### 1.2 系统架构图

```
┌──────────────────────────────────────────────────────────────────────────┐
│                            竞赛任务层                                     │
│                                                                          │
│  ┌───────────────────────────────────────────────────────────────────┐   │
│  │                     TaskMaster (C++ 节点)                         │   │
│  │                                                                   │   │
│  │   状态机 (10 Hz):                                                  │   │
│  │     WAIT_TF → IDLE → NAV_P_QR_B → RING_NAV → NAV_TO_P → PARK → DONE  │
│  │                                                                   │   │
│  │   自定义 RPP 控制器 (20 Hz):                                       │   │
│  │     Pure Pursuit + 自适应前视 + 重捕获                             │   │
│  │     + LiDAR + YOLO 锥桶融合避障                                    │   │
│  └───────────────┬──────────────────────┬────────────────────────────┘   │
│                  │ /cmd_vel (Twist)     │ /announcement, /capture_trigger│
│                  ▼                      ▼                                │
│  ┌──────────────────────────┐  ┌────────────────────────────────────┐   │
│  │  connect_to_pc           │  │  origincar_broadcast               │   │
│  │  HTTP → PC VLM           │  │  I2C 亚博 TTS 模块                  │   │
│  │  /image_description      │  │  队列 + 去重播报                    │   │
│  └──────────────────────────┘  └────────────────────────────────────┘   │
└───────────────────────────────────┬──────────────────────────────────────┘
                                    │
┌───────────────────────────────────┼──────────────────────────────────────┐
│                        导航与定位层                                       │
│  ┌────────────────────────────────┼──────────────────────────────────┐   │
│  │           Nav2 Stack (仅定位)                                      │   │
│  │  ┌──────────────┐  ┌──────────────┐                              │   │
│  │  │ AMCL (2D)    │  │ Map Server   │                              │   │
│  │  │ 定位         │  │ + Keepout    │                              │   │
│  │  └──────┬───────┘  └──────────────┘                              │   │
│  │         │ /amcl_pose, map→odom_combined TF                        │   │
│  └─────────┼────────────────────────────────────────────────────────┘   │
└─────────────┼───────────────────────────────────────────────────────────┘
              │
┌─────────────┼───────────────────────────────────────────────────────────┐
│                         传感器融合层                                     │
│  ┌──────────┴──────────────────────────────────────────────────────┐   │
│  │              robot_localization (EKF 节点)                      │   │
│  │              /odom + /imu/data → /odom_combined (30 Hz)         │   │
│  │              odom_combined → base_footprint TF                   │   │
│  └─────────────┬──────────────────────┬─────────────────────────────┘   │
│                │                      │                                  │
└────────────────┼──────────────────────┼──────────────────────────────────┘
                 │                      │
┌────────────────┼──────────────────────┼──────────────────────────────────┐
│                            驱动层                                        │
│  ┌─────────────┴───────┐  ┌───────────┴────────────┐                    │
│  │  origincar_base     │  │  lslidar_driver         │                   │
│  │  STM32 串口 +       │  │  LSN10 激光雷达驱动      │                   │
│  │  阿克曼转换器        │  │  /scan (LaserScan)      │                   │
│  │  /odom, /imu/data   │  │                         │                   │
│  └──────────┬──────────┘  └────────────────────────┘                    │
│             │ /dev/ttyACM0 115200 bps                                     │
└─────────────┼────────────────────────────────────────────────────────────┘
              │
┌─────────────┴────────────────────────────────────────────────────────────┐
│                            感知层                                        │
│  ┌─────────────────────┐  ┌──────────────────────┐  ┌───────────────┐   │
│  │ Aurora930 摄像头    │  │ qr_bpu_detector      │  │ racing_obs    │   │
│  │ 深度相机驱动         │  │ BPU 推理 + ZBar ROI  │  │ YOLO 锥桶     │   │
│  │ /aurora/rgb/image_raw│  │ /qr_direction         │  │ /racing_obs...│   │
│  └─────────────────────┘  └──────────────────────┘  └───────────────┘   │
└──────────────────────────────────────────────────────────────────────────┘
              │
┌─────────────┴────────────────────────────────────────────────────────────┐
│                            硬件层                                        │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌───────────────────────┐   │
│  │ STM32 MCU│  │ LSN10    │  │ Aurora930│  │ 亚博智能语音合成模块     │   │
│  │ +MPU6050 │  │ 激光雷达  │  │ 摄像头   │  │                       │   │
│  │ +电机    │  │          │  │          │  │                       │   │
│  │ +舵机    │  │          │  │          │  │                       │   │
│  └──────────┘  └──────────┘  └──────────┘  └───────────────────────┘   │
└──────────────────────────────────────────────────────────────────────────┘
```

### 1.3 数据流向

```
Aurora930 摄像头                   LSN10 激光雷达
    │ /aurora/rgb/image_raw            │ /scan
    ├──────────────┬──────────┐        ├────────────────────┐
    ▼              ▼          ▼        ▼                    ▼
qr_bpu_detector  connect_to_pc      AMCL              TaskMaster
BPU 推理          HTTP→PC VLM      定位               LiDAR 避障
    │              │                  │                    │
    ▼              ▼                  │ map→odom_combined  │
/qr_direction  /image_description     │ TF                 │
(String)       (String)              │                    │
    │              │                  │                    │
    └──────────────┼──────────────────┼────────────────────┘
                   │                  │            ▲
                   ▼                  ▼            │
          origincar_broadcast    TaskMaster    /racing_obstacle_detection
          I2C TTS 播报           (状态机+控制器)  YOLO 锥桶
                   │                  │            │
                   │                  ▼            │
                   │           /cmd_vel (Twist)     │
                   │                  │            │
                   │          origincar_base       │
                   │          + 阿克曼转换          │
                   │                  │            │
                   │          STM32 (串口)         │
                   │                  │            │
                   │          电机 + 舵机          │
                   └──────────────────┘
                              (语音输出)
```

---

## 2. 硬件选型与连接方式

### 2.1 硬件清单

| 模块 | 型号 | 功能 | 接口 |
|:---|:---|:---|:---|
| 上位机 | **RDK X5** (arm64, Ubuntu 22.04) | ROS2 运行时、BPU QR 推理、YOLO 锥桶检测、导航、任务编排 | — |
| 下位机 | **STM32** (HAL 库) | 电机与舵机控制、MPU6050 IMU 数据采集、轮式里程计 | USB 串口 `/dev/ttyACM0`, 115200 bps |
| IMU | **MPU6050** (集成在 STM32 板上) | 三轴陀螺仪 + 三轴加速度计 | 通过 STM32 串口帧 |
| 激光雷达 | **LSN10** (镭神 LS 系列) | 360° 2D 激光测距：SLAM 建图、AMCL 定位、实时避障 | UART |
| 摄像头 | **Aurora930** (深度相机, 仅使用 RGB) | 竞赛场地图像采集，用于 QR 检测和 VLM 标记牌识别 | USB |
| 语音模块 | **亚博智能语音合成播报模块** (Yabo Intelligent TTS) | QR 方向结果和 VLM 标记牌识别结果的语音合成播报 | I2C |
| 底盘 | **OriginCar 阿克曼** | 阿克曼转向，五连杆独立悬挂 | 通过 STM32 串口指令 |

### 2.2 硬件连接图

```
                           ┌───────────────────────────┐
                           │         RDK X5             │
                           │      (ROS2 Humble)         │
                           │                            │
              USB ─────────┤ Aurora930 摄像头            │
              UART ────────┤ LSN10 激光雷达              │
              USB-UART ────┤ /dev/ttyACM0 → STM32       │
              I2C ─────────┤ 亚博 TTS 模块               │
                           │                            │
                           └───────────────────────────┘
                                        │
                          USB 串口 (115200 bps)
                                        │
                           ┌────────────┴──────────┐
                           │       STM32 MCU        │
                           │                        │
                           │  ┌──────────────────┐  │
                           │  │ MPU6050 (IMU)    │  │
                           │  └──────────────────┘  │
                           │                        │
                           │  电机驱动 ──── 直流电机 (后轮驱动)   │
                           │  舵机驱动 ──── 舵机 (前轮转向)      │
                           └────────────────────────┘
```

### 2.3 串口通信协议

RDK X5 与 STM32 之间采用自定义 24 字节二进制协议，XOR 校验。

- **波特率**: 115200 bps
- **接收帧** (STM32→RDK, 24 字节):
  - Header(1) + StopFlag(1) + VelocityX(2) + VelocityY(2) + VelocityZ(2) + AccelX(2) + AccelY(2) + AccelZ(2) + GyroX(2) + GyroY(2) + GyroZ(2) + Voltage(2) + Checksum(1) + Tail(1)
- **发送帧** (RDK→STM32, 11 字节):
  - Header(1) + Reserved(2) + Speed(2) + Reserved(2) + Steering(2) + Checksum(1) + Tail(1)
- **指令流向**: `/cmd_vel` (Twist) → `cmd_vel_to_ackermann_drive` → `ackermann_cmd` → `origincar_base` → STM32 串口
- **阿克曼模式**: `akmcar=true` — Twist `linear.x` → 车速, Twist `angular.z` → 转向角

---

## 3. 软件系统设计

### 3.1 软件技术栈

| 层级 | 技术 | 详情 |
|:---|:---|:---|
| 操作系统 | Ubuntu 22.04 (RDK X5 arm64) | — |
| 中间件 | ROS2 Humble | colcon build, --symlink-install |
| 构建 | CMake (C++), setuptools (Python) | 双平台: x86 开发 / arm64 部署 |
| 定位 | AMCL (Nav2) + EKF (robot_localization) | 基于地图的定位 + 里程计/IMU 融合 |
| 运动控制 | 自定义 Pure Pursuit (RPP) | 20 Hz, 自适应前视, Catmull-Rom 平滑 |
| AI 推理 | BPU (RDK X5) — QR 检测 | TensorRT .bin 模型, 零 CPU 开销 |
| 视觉 | OpenCV, ZBar | QR ROI 解码 |
| 锥桶检测 | YOLOv5s/v8s (TensorRT, BPU) | racing_obstacle_detection_yolo (上游原样引用) |
| 大模型 | 阿里云 DashScope (qwen-vl-plus) | PC 端 HTTP REST, 结果桥接到 `/image_description` |
| 语音 | 亚博 I2C TTS → I2C | origincar_broadcast: 多话题队列 + 去重 |
| 串口 | serial (自定义分支) | 跨平台串口, 带超时 |
| 坐标系 | TF2 (仅静态变换) | 无 URDF, 无 robot_state_publisher |

### 3.2 ROS2 包架构

```
dev_ws/
├── src/
│   ├── ackermann_msgs/                        # 阿克曼驱动消息定义 (上游)
│   ├── serial/                                # 跨平台串口通信库
│   │
│   ├── origincar_base/                        # 底盘驱动层
│   │   ├── src/origincar_base.cpp             #   STM32 串口 I/O, IMU 解析, 里程计积分
│   │   ├── src/cmd_vel_to_ackermann_drive.cpp #   Twist → AckermannDriveStamped 转换器
│   │   ├── src/static_tf_node.cpp             #   base_footprint→base_link→laser 静态 TF
│   │   ├── src/odom_tf_node.cpp               #   简易 odom→base_footprint TF (无 EKF 回退)
│   │   ├── config/ekf.yaml                    #   EKF: odom + imu → odom_combined (30 Hz)
│   │   ├── config/slam_mapping.yaml           #   slam_toolbox 在线异步建图
│   │   ├── launch/base_serial.launch.py       #   串口驱动 + 可选阿克曼转换器
│   │   ├── launch/slim_bringup.launch.py      #   最小启动 (无 URDF, 轻量 RViz)
│   │   ├── map/                               #   SLAM 地图: race_modify, race_keepout
│   │   └── param/                             #   Nav2 参数: param_mini_akm.yaml
│   │
│   ├── origincar_msg/                         # 自定义 ROS2 消息定义
│   │
│   ├── origincar_bringup/                     # 统一启动编排
│   │   ├── launch/base.launch.py              #   底盘 + 阿克曼 + EKF + 静态 TF
│   │   ├── launch/perception.launch.py        #   摄像头 + QR BPU + YOLO 锥桶
│   │   ├── launch/mission.launch.py           #   TaskMaster + PC 桥接 + 播报
│   │   ├── launch/competition.launch.py       #   一键启动入口 (所有参数可覆盖)
│   │   ├── launch/task.launch.py              #   终端1: 仅 TaskMaster
│   │   ├── launch/vehicle_stack.launch.py     #   终端2: 底盘 + 定位 + 感知 + 语音
│   │   └── config/competition.yaml            #   TaskMaster 默认参数
│   │
│   ├── origincar_task/                        # 竞赛任务状态机
│   │   ├── src/task_master.cpp                #   状态机 (10Hz) + 自定义 RPP 控制器 (20Hz)
│   │   ├── include/origincar_task/            #   task_master.hpp, mission_policy.hpp
│   │   ├── config/waypoints_flowpath_custom_rpp.yaml  #   主路径点文件
│   │   ├── config/waypoints.yaml              #   旧版坐标系路径点
│   │   └── scripts/                           #   imu_calibrate, odom_calibrate, generate_map_semantics
│   │
│   ├── qr_bpu_detector/                       # BPU 加速 QR 码检测
│   │   ├── src/qr_bpu_detector_node.cpp       #   BPU 模型推理 (TensorRT .bin)
│   │   ├── src/qr_roi_decoder_node.cpp        #   ROI 裁剪 + ZBar 持续解码
│   │   ├── launch/qr_bpu_minimal.launch.py    #   竞赛最小化: 仅 BPU 推理 + ROI 解码
│   │   └── config/                            #   BPU 模型 (.bin) + 类别列表
│   │
│   ├── racing_obstacle_detection_yolo/        # YOLO 锥桶检测 (上游原样引用)
│   │   ├── src/sample.cpp                     #   BPU 推理主程序
│   │   ├── src/image_utils.cpp                #   图像预处理/后处理
│   │   └── src/parser.cpp                     #   模型输出解析
│   │
│   ├── connect_to_pc/                         # 车端→PC HTTP 桥接
│   │   ├── connect_to_pc/car_pc_bridge_node.py  # HTTP 图像发送 + 回调接收
│   │   └── launch/car_pc_bridge.launch.py
│   │
│   ├── origincar_broadcast/                   # 语音播报管理
│   │   ├── origincar_broadcast/broadcast_manager_node.py  # 多话题队列 + 去重
│   │   ├── origincar_broadcast/speech_client.py           # I2C 语音客户端
│   │   └── launch/broadcast.launch.py
│   │
│   ├── lslidar_driver/                        # LSN10 镭神激光雷达驱动
│   ├── lslidar_msgs/                          # 激光雷达自定义消息
│   └── utils/                                 # 图像传输工具节点
│
├── docs/
│   ├── PROJECT_FRAMEWORK.md                   # 项目框架与架构参考
│   └── origincar_plan.md                      # 开发计划与进度跟踪
├── CHANGELOG.md
└── README.md                                  # 本文档
```

### 3.3 三终端架构

比赛运行时采用三终端进程隔离策略，防止单进程故障级联并管理 RDK X5 内存压力：

```
┌─────────────────┐    ┌──────────────────────┐    ┌───────────────────┐
│     终端1       │    │        终端2          │    │      终端3         │
│   task.launch.py│    │ vehicle_stack.launch.py│    │ car_pc_bridge.     │
│                 │    │                       │    │ launch.py          │
│                 │    │                       │    │                    │
│  TaskMaster     │    │  origincar_base       │    │  connect_to_pc     │
│  (状态机+控制器) │    │  + 阿克曼转换          │    │  HTTP→PC VLM       │
│                 │    │  静态 TF               │    │                    │
│                 │    │  EKF                  │    │                    │
│                 │    │  AMCL + Map Server    │    │                    │
│                 │    │  激光雷达驱动           │    │                    │
│                 │    │  Aurora930 摄像头      │    │                    │
│                 │    │  QR BPU 检测器         │    │                    │
│                 │    │  YOLO 锥桶检测         │    │                    │
│                 │    │  播报管理器            │    │                    │
└─────────────────┘    └──────────────────────┘    └───────────────────┘
```

便捷统一单终端启动：
```bash
ros2 launch origincar_bringup competition.launch.py
```

### 3.4 TF 坐标树

```
map ──→ odom_combined ──→ base_footprint ──→ base_link ──→ laser
(AMCL)    (EKF, 30Hz)     (TF static, 0,0,0)  (0.092,0,0)  (x=laser_x, z=0.102)
                                                             yaw=laser_yaw
```

- `map → odom_combined`: 由 AMCL 发布 (全局定位校正)
- `odom_combined → base_footprint`: 由 EKF 节点发布 (融合里程计 + IMU, 30 Hz)
- `base_footprint → base_link`: 静态 TF (后轴居中 X 偏移, 0.092m)
- `base_link → laser`: 静态 TF，可通过 `laser_yaw` 参数运行时调整偏航角 (激光雷达标定)
- **无 URDF, 无 robot_state_publisher** — 所有变换为静态 TF 节点，内存占用最小
- EKF 在底盘串口启动 4 秒后启动，避免 DDS 初始化内存峰值

### 3.5 核心话题

| 话题 | 类型 | 发布者 | QoS | 描述 |
|:---|:---|:---|:---|:---|
| `/odom` | `nav_msgs/Odometry` | `origincar_base` | Default | 来自 STM32 串口的轮式里程计 |
| `/imu/data` | `sensor_msgs/Imu` | `origincar_base` | Default | 来自 MPU6050 的原始 IMU (通过 STM32) |
| `/odom_combined` | `nav_msgs/Odometry` | `ekf_node` | Default | EKF 融合里程计 (30 Hz) |
| `/scan` | `sensor_msgs/LaserScan` | `lslidar_driver` | Sensor Data | 2D 激光扫描用于 AMCL + 避障 |
| `/aurora/rgb/image_raw` | `sensor_msgs/Image` | Aurora930 摄像头 | Sensor Data | 原始 RGB 图像用于 QR 检测和 VLM |
| `/qr_direction` | `std_msgs/String` | `qr_roi_decoder_node` | Default | QR 内容 → 方向 (奇数→顺时针, 偶数→逆时针) |
| `/racing_obstacle_detection` | `ai_msgs/PerceptionTargets` | YOLO 锥桶节点 | Sensor Data | 锥桶检测结果 (边界框 + 置信度) |
| `/image_description` | `std_msgs/String` | `connect_to_pc` | Default | 来自 PC 的 VLM 标记牌识别结果 |
| `/announcement` | `std_msgs/String` | TaskMaster | Default | 通过 I2C TTS 播报的文本 |
| `/capture_trigger` | `std_msgs/Empty` | TaskMaster | Default | 标记牌拍照触发信号发送至 PC 桥接 |
| `/cmd_vel` | `geometry_msgs/Twist` | TaskMaster | Default | 速度指令 (20 Hz) 发送至底盘 |

---

## 4. 关键任务实现策略

### 4.1 TaskMaster 任务状态机

TaskMaster 是竞赛系统的中央大脑，以单个 C++ 节点实现，包含两个定时循环：

```
                   ┌──────────┐
                   │ WAIT_TF  │  等待 map→base_footprint TF 稳定
                   │ (启动)    │  (5 帧连续稳定 + 超时保护)
                   └────┬─────┘
                        │ TF 稳定 ≥5 帧
                   ┌────▼─────┐
                   │   IDLE   │  等待 2s, 播报 "任务开始"
                   └────┬─────┘
                        │
                   ┌────▼──────────┐
                   │ NAV_P_QR_B    │  巡线: P → QR 扫描区 → B 区通道入口
                   │ (合并路径)     │  QR: 基于奇偶的方向锁定 (奇数→顺时针, 偶数→逆时针)
                   │               │  QR 截止: 若截止点前未检测到 QR 则自动锁定默认方向
                   └────┬──────────┘
                        │ 路径完成 (QR 已锁定), 无缝过渡
                   ┌────▼─────┐
                   │ RING_NAV │  C 区环道绕行 (顺时针或逆时针取决于 QR 结果)
                   │           │  在指定路径点触发标记牌拍照
                   │           │  LiDAR + YOLO 锥桶融合避障
                   └────┬─────┘
                        │ 环道完成
                   ┌────▼─────┐
                   │ NAV_TO_P │  从 B 区通道出口返回 → P 点
                   └────┬─────┘
                        │ 到达 P 点
                   ┌────▼─────┐
                   │   PARK   │  停车 3s, 播报 "任务完成"
                   └────┬─────┘
                        │
                   ┌────▼─────┐
                   │   DONE   │  终止状态
                   └──────────┘
```

**每阶段 90s 超时**: 任意阶段超过 90 秒强制 FAILED → PARK，防止无限循环。

### 4.2 自定义 RPP 控制器

TaskMaster 不使用 Nav2 进行运动控制，而是在内部实现了 20 Hz 的自定义 Pure Pursuit 控制器，具有以下特性：

| 特性 | 描述 |
|:---|:---|
| **自适应前视** | 前方曲率高时缩短前视距离 (累计偏航 ≥0.5 rad → 0.78×; ≥1.0 rad → 0.55×) |
| **重捕获** | 机器人偏离路径时，跳转到 20 点窗口内最近的剩余路径点 |
| **前视进度提交** | 前视目标前进时提交已通过的路径点，实现平滑连续运动 |
| **精密路径点** | 首/尾 2 点、运动转换点、QR 扫描/拍照/暂停点使用紧密通过半径 |
| **逐点增益** | 每个路径点具有 `speed_gain` 和 `angular_gain` 用于精细速度/转向控制 |
| **QR 区减速** | QR 扫描路径点附近应用 `qr_scan_speed_gain=0.85` (仅 QR 未决状态) |
| **拍照区减速** | 拍照路径点附近应用 `capture_speed_gain=0.60` |
| **转弯增益** | `angular_gain > 1.0` 的路径点应用 `turn_speed_gain=0.50`, `turn_angular_gain=1.50` |
| **路径修剪** | 状态切换时，将新路径修剪为从机器人当前位置最近点开始 |

```
控制循环 (20 Hz):
  1. 查询机器人位姿 (map→base_footprint TF)
  2. 处理暂停 (若有暂停路径点)
  3. 重捕获进度 (若偏离)
  4. 推进已通过的路径点
  5. 选择自适应前视目标
  6. 计算速度 (基准 × 路径点增益 × 区域增益)
  7. 计算 Pure Pursuit 曲率 → 角速度
  8. 应用 LiDAR + YOLO 锥桶融合避障
  9. 发布 /cmd_vel
```

### 4.3 LiDAR + YOLO 锥桶融合避障

避障系统融合 LiDAR 距离数据与 YOLO 锥桶检测结果，实现可靠的障碍物处理：

```
传感器输入                    状态机                         控制输出
─────────────                 ─────────────                  ──────────────
LiDAR /scan ─────┐
                 ├──→ 扇区分析 ──→ 避障状态:
YOLO 锥桶 ───────┘    前/左/右        ├── CLEAR: 正常 RPP 控制
                      最小距离         ├── AVOIDING: 速度 × slow_scale
                                      │             + 角速度偏置
                                      ├── BACKUP_ONCE: 倒车
                                      │   backup_distance 或超时
                                      └── WAIT_CLEAR: 零速度,
                                          等待锥桶超时 + 保持

融合规则:
  • 需锥桶检测 (置信度 ≥ cone_confidence_threshold) 才能进入 AVOIDING
  • 仅 LiDAR 检测到紧急距离内障碍物 → 仅 SAFETY STOP (不避障)
  • 超过 cone_detection_timeout_sec + obstacle_clear_hold_sec 无有效锥桶 → REACQUIRE_PATH → CLEAR
  • 每次遭遇仅倒车一次; 第二次禁止倒车 → WAIT_CLEAR
  • 避障方向: 基于左/右距离差 + 锥桶图像位置
```

| 参数 | 默认值 | 描述 |
|:---|:---|:---|
| `cone_confidence_threshold` | 0.35 | 视为确认锥桶的最低 YOLO 置信度 |
| `obstacle_backup_distance_threshold` | 0.20 m | 触发倒车的距离 |
| `obstacle_avoid_distance` | 0.45 m | 开始避障转向的距离 |
| `obstacle_slow_distance` | 0.45 m | 开始减速的距离 |
| `obstacle_backup_distance` | 0.40 m | 倒车距离 |
| `obstacle_backup_timeout_sec` | 1.50 s | 最大倒车时长 |
| `cone_detection_timeout_sec` | 0.25 s | 锥桶数据最大有效时长 |
| `obstacle_clear_hold_sec` | 0.40 s | 最后一次检测到锥桶后清除状态的保持时间 |
| `rear_clearance_distance` | 0.25 m | 倒车所需的最小后方间隙 |
| `avoid_proximity_max_angular_z` | 1.10 rad/s | 避障期间最大角速度 |

### 4.4 QR 码检测

两级 BPU + CPU 流水线：

```
Aurora930 摄像头 (/aurora/rgb/image_raw)
    │
    ▼
qr_bpu_detector_node
    │  BPU 推理 (TensorRT .bin 模型)
    │  14~15 FPS, 平均推理 ~5.7~6.4 ms
    │  输出: QR 码周围的检测边界框
    ▼
qr_roi_decoder_node
    │  利用 BPU 边界框裁剪 ROI
    │  ZBar 持续解码
    │  仅发布有效数字结果 (0-9999)
    ▼
/qr_direction (String)
    │
    ▼
TaskMaster
    │  解析数字 → 奇数=顺时针, 偶数=逆时针
    │  锁定方向 (QrDecisionState: PENDING→CW_LOCKED/CCW_LOCKED)
    │  仅在 NAV_P_QR_B 状态下接受 QR
    │  QR 截止路径点强制锁定默认方向
```

### 4.5 云端 VLM 标记牌识别

```
TaskMaster 到达拍照路径点
    │
    ▼
/capture_trigger (Empty) → /person_trigger (Int32)
    │
    ▼
connect_to_pc (car_pc_bridge_node)
    │  捕获最新 Aurora930 帧
    │  HTTP POST → PC VLM 服务器 (192.168.3.12:9999/predict)
    │  通过 HTTP 回调接收文字识别结果
    ▼
/image_description (String)
    │
    ▼
origincar_broadcast → I2C → 亚博 TTS 模块 → 语音输出
```

**设计思路**: VLM 推理完全卸载到独立 PC，保留 RDK X5 计算资源用于实时导航和避障。

### 4.6 语音播报

```
/qr_direction ──────────┐
                        ├──→ origincar_broadcast
/image_description ─────┘    broadcast_manager_node
/announcement ──────────────→  • 多话题订阅
                               • 基于队列的排序 (先到先播)
                               • 去重
                               • 串行 I2C 访问
                                    │
                                    ▼
                              speech_client.py → I2C → 亚博 TTS 模块
```

### 4.7 路径结构

路径点文件 `waypoints_flowpath_custom_rpp.yaml` 定义四条路线：

| 路径 | 点数 | 描述 |
|:---|:---|:---|
| `p_to_qr_to_b_rpp` | ~33 | P → QR 扫描区 → B 区通道入口 (合并, 仅前进) |
| `ring_cw` | ~88 | C 区环道顺时针绕行 |
| `ring_ccw` | ~88 | C 区环道逆时针绕行 |
| `return_to_p_rpp` | ~16 | B 区通道出口 → P 点返回路径 |

每个路径点支持：
- `motion`: `forward` / `reverse`
- `pass_radius`: 路径点通过半径 (精密点用紧密值)
- `speed_gain` / `angular_gain`: 逐点控制增益
- `pause`: 暂停时长 (秒)
- `qr_scan`: QR 扫描区域标记
- `qr_deadline`: 若未决则强制锁定 QR 方向
- `capture`: 触发 VLM 标记牌拍照

---

## 5. 竞赛任务与规则适配

### 5.1 竞赛场地

```
┌───────────────────────────┐
│                           │   5m × 5m 场地
│    A 区 (蓝色)             │   仅外围围栏
│                           │   无物理内部隔墙
│    B 区 (黄色)             │   ABC 三区: 仅地面颜色
│                           │
│    ┌──────────┐           │
│    │ C 区     │           │
│    │ (绿色 +  │           │
│    │  黄色)   │           │
│    └──────────┘           │
│                           │
└───────────────────────────┘

LiDAR 可见:   仅外围围栏
LiDAR 不可见: B 区通道边界、C 区边界、黄色环道、所有地面颜色
AMCL 质量:   角落好 (L 形特征) / 墙中段差 (对称) / 中心差
```

### 5.2 物理约束与语义约束分离

AMCL 定位地图 (`race_modify`) 只包含 LiDAR 真实可见的外围围栏。B 区通道边界、C 区绿色禁行区、黄色环道等 LiDAR 看不到的地面语义信息不写入定位地图。

语义约束通过以下方式实施：
- **路径点引导**: 路径点引导机器人穿过 B 区通道
- **Keepout 滤波遮罩** (`race_keepout`): 禁行区虚拟障碍物
- **速度分区**: C 区环道弯道减速 (`speed_gain`)

### 5.3 规则适配

| 规则 | 实现 |
|:---|:---|
| 180s 限时 | 优化路径密度 + `cruise_speed=0.85 m/s` + 每阶段 90s 超时回退 |
| 碰撞障碍物 +10s/次 | LiDAR + YOLO 锥桶融合避障: 0.20m 紧急停车, 0.45m 减速, 倒车+等待恢复 |
| QR 扫描确定方向 | BPU 检测 + ZBar 解码 → 基于奇偶的方向锁定 (奇数→顺时针, 偶数→逆时针) |
| QR 截止回退 | 可配置 `default_qr_direction` (cw/ccw)，截止路径点前未检测到 QR 则自动锁定 |
| B 区通道穿行 | 路径点引导穿过 B 区通道 + keepout 虚拟边界墙 |
| C 区环道 + 标记牌 | 完整环道路径 (88 点) 含 `capture: true` 标记 → VLM 识别 |
| 返回 P 点并停车 | `return_to_p_rpp` 路径终点 P，`goal_tolerance=0.35m` |
| 语音播报 | I2C 亚博 TTS 模块，队列播报；无需显示屏 |
| 共享网络 DDS 干扰 | `ROS_DOMAIN_ID=42` + FastDDS Discovery Server 已就绪 |

---

## 6. 系统启动与部署

### 6.1 构建

```bash
cd dev_ws
source /opt/ros/humble/setup.bash

# x86 构建 (跳过 BPU 包)
colcon build --symlink-install \
  --packages-skip qr_bpu_detector racing_obstacle_detection_yolo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# RDK X5 完整构建
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 6.2 三终端启动（比赛）

```bash
# 推荐: 单脚本启动
ros2 run origincar_bringup start_competition_3term.sh

# 等效手动命令:
# 终端1 — TaskMaster
ros2 launch origincar_bringup task.launch.py

# 终端2 — 车辆栈 (底盘, TF, EKF, AMCL, 激光雷达, 摄像头, QR, YOLO, 播报)
ros2 launch origincar_bringup vehicle_stack.launch.py

# 终端3 — PC VLM 桥接
ros2 launch connect_to_pc car_pc_bridge.launch.py
```

### 6.3 一键启动（调试）

```bash
# 所有子系统在一个终端中，完整参数控制
ros2 launch origincar_bringup competition.launch.py

# 覆盖速度
ros2 launch origincar_bringup competition.launch.py cruise_speed:=0.90

# 覆盖 QR 默认方向
ros2 launch origincar_bringup competition.launch.py default_qr_direction:=ccw

# 启动部分子系统
ros2 launch origincar_bringup competition.launch.py \
  start_camera:=false start_qr:=false start_cone_detection:=false \
  start_task:=false start_pc_bridge:=false start_broadcast:=false
```

### 6.4 关键运行参数

```
TaskMaster:
  cruise_speed: 0.85 m/s         (基础前进速度)
  qr_scan_speed_gain: 0.85       (QR 扫描区速度缩放)
  turn_speed_gain: 0.50          (急转弯速度缩放)
  turn_angular_gain: 1.50        (急转弯角速度增益)
  capture_speed_gain: 0.60       (VLM 拍照区速度缩放)
  max_angular_z: 1.80 rad/s      (转向速率限制)
  control_frequency: 20.0 Hz     (RPP 控制器频率)
  lookahead_dist: 0.22 m         (基础前视距离)
  pass_radius: 0.35 m            (默认路径点通过半径)
  reverse_pass_radius: 0.30 m    (倒车路径点通过半径)
  goal_tolerance: 0.35 m         (终点路径点容差)

LiDAR 外参 (base→laser):
  laser_x: -0.10 m               (base_link 到激光雷达 X 偏移)
  laser_yaw: 0.05 rad            (偏航角标定偏移)
```

### 6.5 DDS 竞赛网络隔离

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 daemon stop
```

---

## 7. 路径点标定工具

基于 HTML 的手动路径点编辑器用于高效场地标定：

- **位置**: `origin_car/tools/manual-waypoint-editor-origin.html`
- **功能**:
  - 在 SLAM 地图上拖拽放置路径点 (`race_modify` / `race_keepout`)
  - 可视化叠加层：区域、禁行区、QR 扫描区
  - 多路线支持 (顺时针 / 逆时针)
  - 带可拖拽手柄的偏航角可视化
  - 逐点通过半径配置
  - 撤销/重做历史
  - YAML 导出兼容 `waypoints_flowpath_custom_rpp.yaml` 格式
- **配套地图**: `origin_car/tools/map/race_modify.png`, `race_keepout.png`

```
tools/
├── manual-waypoint-editor-origin.html  # 路径点标定工具
└── map/
    ├── race_modify.png                 # AMCL 定位地图叠加层
    └── race_keepout.png                # Keepout 遮罩叠加层
```

---

## 8. 技术亮点与创新

### 8.1 统一单工作空间架构

14 个运行时包统一在一个 `dev_ws` 工作空间中，所有参数通过顶层 launch 文件暴露，支持单命令覆盖任意运行时参数。

### 8.2 三终端进程隔离

TaskMaster、车辆基础设施和 PC 桥接分别运行在独立终端进程中，防止单点故障级联，并通过错峰启动缓解 RDK X5 内存压力。

### 8.3 物理-语义约束分离

AMCL 定位地图只包含真实 LiDAR 可见的物理结构，语义约束通过路径点、keepout mask 和速度分区实施，避免 AMCL 匹配虚假墙体导致定位发散。

### 8.4 激光 + 视觉融合避障

YOLO 锥桶检测提供语义确认（避免 LiDAR 噪点误触发），LiDAR 提供精确距离（保证安全紧急停车）。

### 8.5 云端-本地混合架构

VLM 推理完全不消耗车端算力，所有 BPU/CPU 资源用于 20 Hz 实时运动控制和感知。

### 8.6 自适应 Pure Pursuit + 路径重捕获

控制器具有曲率自适应前视距离、逐点增益控制和自动路径重捕获——即使在避障导致偏离后也能自动回到路径。

---

## 构建状态

| 平台 | 状态 | 备注 |
|:---|:---|:---|
| x86 (开发) | 待验证 | 跳过 BPU 包 (`qr_bpu_detector`, `racing_obstacle_detection_yolo`) |
| RDK X5 (部署) | 待验证 | 完整构建所有包 |

---

## 许可

Proprietary — 第21届全国大学生智能汽车竞赛（地瓜机器人赛项）
