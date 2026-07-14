# OriginCar Project Framework

## Current Unified Runtime

正式比赛使用 `origincar_place/dev_ws` 单工作空间和三终端启动：

1. `origincar_bringup/task.launch.py`：只启动 TaskMaster。
2. `origincar_bringup/vehicle_stack.launch.py`：启动底盘、静态 TF、EKF、Nav2 localization、LiDAR、相机、语音、QR 和原版锥桶检测。
3. `connect_to_pc/car_pc_bridge.launch.py`：只启动 PC 大模型桥接。

推荐入口为 `ros2 run origincar_bringup start_competition_3term.sh`。旧目录结构说明仅作为迁移来源记录，不代表当前运行布局。

`racing_obstacle_detection_yolo` 必须保持与旧工程原版逐文件一致，任何适配只能放在 `origincar_bringup` 或 TaskMaster。当前运行不需要 URDF，因此新工作空间不包含 `origincar_description`，也不启动 `robot_state_publisher`。

## Project Overview

OriginCar 是一个面向**第21届全国大学生智能汽车竞赛（地瓜机器人赛项）**的自主驾驶机器人项目。基于 OriginCar 阿克曼底盘（STM32 下位机 + RDK X5 上位机），使用 ROS2 Humble 构建全栈自动驾驶系统。

竞赛任务：车辆从 P 点出发，在 5m×5m 场地内自主完成避障行驶、QR 码扫描、B 区通道穿越、C 区环道行驶、标记牌识别与大模型图生文播报，最后返回 P 点停车。限时 180 秒。

---

## Directory Structure

```
origin_car/
├── dev_ws/                          # 主工作空间 (核心功能)
│   ├── config/                      # 全局模型配置 (YOLO/ResNet/EfficientNet 等)
│   └── src/origincar/               # 核心 ROS2 元包
│       ├── 3rdparty/                # 第三方依赖
│       │   ├── ackermann_msgs-ros2/ # 阿克曼底盘消息定义
│       │   ├── aurora930/           # Aurora930 深度相机 .deb 驱动包
│       │   └── serial_ros2/         # 跨平台串口通信库
│       ├── origincar_base/          # 底盘驱动与传感器节点
│       ├── origincar_bringup/       # 相机与可视化 launch 文件
│       ├── origincar_description/   # URDF/XACRO 机器人模型
│       ├── origincar_msg/           # 自定义 ROS2 消息接口
│       ├── origincar_task/          # 竞赛任务状态机 (TaskMaster)
│       └── utils/                   # 图像传输工具节点
│
├── qr_detect_ws/                    # QR 码检测工作空间 (BPU 推理)
│   └── src/qr_bpu_detector/
│       ├── config/                  # BPU 模型 (.bin) + 类别列表
│       ├── launch/                  # 检测/ROI解码 launch 文件
│       └── src/
│           ├── qr_bpu_detector_node.cpp   # BPU 推理主节点
│           └── qr_roi_decoder_node.cpp    # ROI 区域解码节点
│
├── speaker_ws/                      # 语音/多模态工作空间
│   ├── image_to_text/               # VLM 图像识别模块 (Qwen2-VL)
│   │   └── image_to_text/
│   │       ├── qwen_api_server.py   # Qwen2-VL API 服务端
│   │       └── car_sender.py        # 车端图像发送节点
│   └── text_to_voice/               # TTS 文字转语音模块
│       ├── qwen_api_server.py       # TTS 服务端
│       └── car_sender.py            # 车端语音发送节点
│
├── broadcast_ws/                    # 语音播报管理工作空间
│   └── src/origincar_broadcast/
│       ├── launch/broadcast.launch.py
│       └── origincar_broadcast/
│           ├── broadcast_manager_node.py  # 播报队列管理节点
│           └── speech_client.py           # 语音客户端
│
├── connect_to_pc_ws/                # 车端→PC 桥接工作空间
│   └── src/connect_to_pc/
│       ├── launch/car_pc_bridge.launch.py
│       └── connect_to_pc/
│           └── car_pc_bridge_node.py      # HTTP 图像帧桥接节点
│
├── collect_qr_image_ws/             # QR 图像采集工具工作空间
│   └── src/collect_qr_image/
│       └── collect_qr_images.py           # Aurora930 RGB 手动采集工具
│
├── lidar_ws/                        # LiDAR 工作空间
│   ├── wheeltec_lidar.launch.py     # LiDAR 启动快捷入口
│   └── src/
│       ├── lslidar_driver/          # 镭神 LS 系列 LiDAR 驱动 (ROS2)
│       └── lslidar_msgs/            # LiDAR 自定义消息
│
├── ros2_ws/                         # 辅助功能工作空间
│   └── src/
│       ├── image_to_text/           # 图生文 HTTP 客户端完整 ROS2 包
│       └── republish_node/          # 图像压缩重发布节点 (C++)
│
├── deptrum-ros-driver-aurora930/    # Aurora930 深度相机独立驱动
│   └── launch/                      # aurora930_launch.py / viewer930_launch.py
├── connect_to_pc_ws/                 # 车-PC 大模型 HTTP 桥接工作空间
│   └── src/
│       └── connect_to_pc/            # 相机图片发送、PC 回调接收、/image_description 发布
│
├── broadcast_ws/                    # 语音播报工作空间
│   └── src/
│       └── origincar_broadcast/     # 多话题播报管理与 I2C 语音驱动
│
├── docs/                            # 项目文档
│   ├── origincar_plan.md            # 竞赛开发计划
│   ├── PROJECT_FRAMEWORK.md         # 本文档
│   └── Program_of_the_21st...       # 竞赛官方方案说明
│
├── CLAUDE.md                        # AI 助手工作指引
└── .gitignore                       # Git 忽略规则
```

---

## System Architecture

### Overall Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                      TaskMaster (FSM)                            │
│   IDLE→NAV_TO_QR→SCAN_QR→NAV_TO_B→CROSS_B                       │
│   →RING_NAV(→CAPTURE_SIGN)→EXIT_C→NAV_TO_P→PARK→DONE            │
│   C++ Node, enum-class FSM, 10Hz Timer                           │
└──────┬────────────────┬────────────────┬────────────────────────┘
       │ NavigateToPose │ /capture_trigger│ /announcement
       │ FollowPath     │                │
       ▼                ▼                ▼
┌──────────────┐  ┌─────────────┐  ┌──────────────────────────────┐
│  Nav2 Stack  │  │  感知子系统  │  │       语音播报子系统            │
│  AMCL+SMAC   │  │             │  │ broadcast_ws                  │
│  MPPI Ctrl   │  │ qr_detect_ws│  │ broadcast_manager_node        │
│  BT Navigator│  │ ├ BPU推理   │  │   ↑ /announcement topic       │
└──────┬───────┘  │ └ ROI解码   │  │                              │
       │/cmd_vel  │   ↓/qr_result│  │ speaker_ws (VLM→TTS)         │
       ▼          │             │  │ connect_to_pc → PC VLM服务    │
┌──────────────┐  │ Aurora930   │  │   ↓ /image_description        │
│origincar_base│  │ /image_raw  │  │ text_to_voice TTS合成         │
│ STM32通信     │  │ → VLM识别   │  └──────────────────────────────┘
│ /imu/data_raw│  └─────────────┘
│ /odom        │
│ EKF→/odom_combined
└──────────────┘
```

### Sensor & Data Flow

```
┌──────────┐  /scan       ┌──────────┐  costmap    ┌──────────┐
│  LiDAR   │─────────────▶│  Nav2    │────────────▶│  MPPI    │
│  LSN10   │              │  AMCL    │             │Controller│
└──────────┘              └──────────┘             └────┬─────┘
                                                        │/ackermann_cmd
┌──────────┐  /image_raw  ┌──────────────┐              │
│ Aurora930│─────────────▶│qr_bpu_detect │              ▼
│ RGB相机  │  /image_raw  │ BPU推理+ROI  │──/qr_result→TaskMaster
│          │─────────────▶│connect_to_pc │                │
└──────────┘              │→ PC VLM服务  │──/image_desc   │
                          └──────────────┘             ┌──┴───────┐
┌──────────┐  /imu/data_raw ┌──────────┐               │  STM32   │
│  MPU6050 │───────────────▶│  EKF     │  /odom_combined│ Motor/   │
│ (STM32)  │               │  Fusion  │──────────────▶│ Servo    │
└──────────┘               └──────────┘               └──────────┘
         /odom ──────────────────────────────────────────────────▲
```

---

## Package Details

### 1. origincar_base — 底盘驱动核心

| 项目 | 内容 |
|------|------|
| 语言 | C++ |
| 功能 | STM32 串口通信、IMU 数据解析、里程计计算、舵机/电机指令下发 |
| 发布 | `/imu/data_raw`, `/odom` |
| 订阅 | `/ackermann_cmd` |
| 关键文件 | `src/origincar_base.cpp`, `src/Quaternion_Solution.cpp` |
| 已知问题 | IMU 双重滤波、零偏未校准、里程计系数为经验值 |

### 2. origincar_bringup — 启动管理

| 项目 | 内容 |
|------|------|
| 语言 | Python (launch) |
| 功能 | USB相机、深度相机、WebSocket 显示器启动配置 |
| launch | `camera.launch.py`, `deepcamera.launch.py`, `usb_websocket_display.launch.py` |

### 3. origincar_description — 机器人模型

| 项目 | 内容 |
|------|------|
| 格式 | URDF/XACRO |
| 内容 | 阿克曼底盘 3D 模型、五连杆悬挂 STL 网格 |
| 关键文件 | `urdf/origincar.xacro`, `meshes/*.STL` |

### 4. origincar_msg — 消息接口

| 项目 | 内容 |
|------|------|
| 消息 | `Data.msg` (感知数据), `Roi.msg` (ROI), `Sign.msg` (标志) |
| 位置 | `dev_ws/src/origincar/origincar_msg/` 和 `racing_ws/src/origincar_msg/` |

### 5. origincar_task — 竞赛任务状态机 (新建)

| 项目 | 内容 |
|------|------|
| 语言 | C++ |
| 功能 | FSM 编排全流程任务，调 Nav2 Action 实现导航 |
| 状态 | IDLE → NAV_TO_QR → SCAN_QR → NAV_TO_B → CROSS_B → RING_NAV(→CAPTURE_SIGN) → EXIT_C → NAV_TO_P → PARK → DONE |
| 关键文件 | `src/task_master.cpp`, `include/origincar_task/task_master.hpp` |
| 依赖 | Nav2 (NavigateToPose, FollowPath), YAML-cpp |

### 6. racing_control — 视觉赛车控制

| 项目 | 内容 |
|------|------|
| 语言 | C++ |
| 功能 | 视觉线跟踪 + 动态 PID 控制 + 锥桶避障 + QR 码检测 |
| 关键文件 | `src/racing_control.cpp`, `include/racing_control/racing_control.h` |
| 来源 | 地瓜机器人官方开源，针对 X5 优化 |

### 7. racing_obstacle_detection_yolo — YOLO 障碍物检测

| 项目 | 内容 |
|------|------|
| 语言 | C++ |
| 模型 | YOLOv5s / YOLOv8s (TensorRT .bin) |
| 功能 | 锥桶检测，发布感知结果到 `/racing_obstacle_detection` |
| 关键文件 | `src/sample.cpp`, `src/image_utils.cpp`, `src/parser.cpp` |

### 8. racing_track_detection_resnet — 赛道检测

| 项目 | 内容 |
|------|------|
| 语言 | C++ |
| 模型 | ResNet (race_track_detection_7.15.bin) |
| 功能 | 赛道线分割，输出左右线位置用于 PID 控制 |

### 9. lslidar_driver — LiDAR 驱动

| 项目 | 内容 |
|------|------|
| 语言 | C++ |
| 支持型号 | LSM10, LSM10P, LSN10, LSN10P |
| 通信 | 串口(UART) 或 网络(UDP) |
| 发布 | `/scan` (LaserScan) |
| 状态 | 代码已编译，实车未验证 |

### 10. qr_bpu_detector — QR 码 BPU 推理检测

| 项目 | 内容 |
|------|------|
| 语言 | C++ |
| 推理 | RDK X5 BPU（.bin 模型，零 CPU 占用） |
| 功能 | 图像中 QR 码目标检测（BPU推理）+ ROI 区域解码 |
| 发布 | `/qr_result` (String: CW/CCW 方向) |
| 订阅 | `/image_raw` |
| 关键文件 | `src/qr_bpu_detector_node.cpp`, `src/qr_roi_decoder_node.cpp` |

### 11. speaker_ws — 视觉语言模型与 TTS

| 子模块 | 功能 |
|--------|------|
| `image_to_text/qwen_api_server.py` | Qwen2-VL API 服务端，对外暴露图生文接口 |
| `image_to_text/car_sender.py` | 车端图像发送，触发 VLM 识别 |
| `text_to_voice/qwen_api_server.py` | TTS 服务端，将文字合成语音 |
| `text_to_voice/car_sender.py` | 车端语音发送节点 |

> PC 端运行 Qwen2-VL 服务，车端通过 HTTP 发图、接收识别结果，不消耗车端算力。

### 12. broadcast_ws — 语音播报管理

| 项目 | 内容 |
|------|------|
| 语言 | Python |
| 功能 | 订阅 `/announcement` 话题，管理播报队列，调用 TTS 服务合成语音 |
| 关键文件 | `origincar_broadcast/broadcast_manager_node.py`, `speech_client.py` |
| launch | `broadcast.launch.py` |

### 13. connect_to_pc_ws — 车端→PC 图像桥接

| 项目 | 内容 |
|------|------|
| 语言 | Python |
| 功能 | 将 Aurora930 相机帧通过 HTTP 转发到 PC 端 VLM 服务 |
| 关键文件 | `connect_to_pc/car_pc_bridge_node.py` |
| launch | `car_pc_bridge.launch.py` |

### 14. 辅助包

| 包名 | 语言 | 功能 |
|------|------|------|
| `collect_qr_image` | Python | Aurora930 RGB 手动 QR 图像采集工具 |
| `image_to_text` (ros2_ws) | Python | 完整 ROS2 版图生文 HTTP 客户端 |
| `connect_to_pc` | Python | 车-PC HTTP 图生文桥接，发布 `/image_description` |
| `origincar_broadcast` | Python | 多话题播报管理，按队列串行驱动 I2C 语音模块 |
| `republish_node` | C++ | 图像压缩与话题重发布 |
| `utils` | C++ | 图像传输协议转换 |
| `qrtest` (racing_ws) | C++ | 旧版 QR 码检测（YOLO ROI+ZBar，已暂停） |

---

## Technology Stack

| 层级 | 技术 |
|------|------|
| 操作系统 | Ubuntu 22.04 (RDK X5 arm64) |
| 中间件 | ROS2 Jazzy (colcon build) |
| AI 推理 | TensorRT 10.1, YOLOv5s/v8s, ResNet |
| 视觉 | OpenCV CUDA |
| 导航 | Nav2 (AMCL + SMAC Planner + MPPI Controller) |
| 传感器融合 | robot_localization (EKF) |
| 下位机 | STM32 (HAL 库) + 串口协议 |
| 底盘 | 阿克曼转向 (Ackermann Drive) |
| 大模型集成 | HTTP REST → 远程 LLM 推理 |
| 仿真 | Gazebo (URDF), RViz 可视化 |
| 版本控制 | Git (dev/main 分支模型) |

---

## Build System

### 构建命令

```bash
# 主工作空间
cd dev_ws/
colcon build --symlink-install

# LiDAR 工作空间
cd lidar_ws/
colcon build --symlink-install

# 辅助工作空间
cd ros2_ws/
colcon build --symlink-install

# 车-PC 大模型桥接工作空间
cd connect_to_pc_ws/
colcon build --symlink-install

# 语音播报工作空间
cd broadcast_ws/
colcon build --symlink-install
```

### 跨平台策略

- **开发机**: x86 Ubuntu 22.04 (开发、调试、测试)
- **部署机**: RDK X5 arm64 (实车运行)
- `.gitignore` 忽略所有 `build/`, `install/`, `log/` 目录

---

## Key Design Decisions

1. **Nav2 全栈导航 > 纯视觉跟踪**: 新方案引入 Nav2 实现 A/B 区自主导航，C 区备选视觉线跟踪
2. **MPPI 控制器**: 已内置阿克曼约束 (min_turn_r=0.35m)，无需额外开发轨迹跟踪器
3. **不采用 Ego-Planner**: 阿克曼约束标注"不稳定"，场景不匹配 (详见 origincar_plan.md §2.3)
4. **IMU 双重滤波待修复**: 移除 Quaternion_Solution 四元数输出，保留 imu_filter_madgwick
5. **pc 端大模型**: 图生文采用 HTTP→PC 方案，不消耗车端算力；车-PC 之间使用普通 HTTP 协议，车端仅在本地把文本桥接到 `/image_description`
6. **无屏幕模块**: 纯语音播报，不购买屏幕硬件
7. **播报采用话题入口**: `origincar_broadcast` 订阅 `/qr_direction` 与 `/image_description`，两路文本结果按实际到达时间进入队列，统一去重、排队和串行访问 I2C，避免重复播报或多方同时占用语音模块
8. **竞赛环境 DDS 隔离**: 默认使用 `ROS_DOMAIN_ID=42`，必要时使用 FastDDS Discovery Server，降低多队同网段 multicast discovery 对 RDK X5 内存和 CLI 响应的影响
9. **低负载建图 fallback**: 正式导航优先使用 EKF + AMCL；当 RDK X5 内存峰值不可接受时，建图阶段可临时使用 `odom_tf_node` + `slam_mapping_no_ekf.launch.py` 绕开 EKF
10. **轻量 RViz 策略**: 远程调试 RViz 默认不开 `/scan`，只显示 Map/TF/Odom 等低带宽信息，需要看 scan 时手动打开
11. **QR 比赛最小链路**: `qr_bpu_minimal.launch.py` 只启动 BPU 检测和 ROI 解码，默认订阅 `/aurora/rgb/image_raw` raw 图像，不启动 websocket/debug image

---

## Current Development Status

参考 `docs/origincar_plan.md` 中的执行阶段:

| 阶段 | 状态 |
|------|------|
| 阶段1: 物理验证 (LiDAR/IMU/里程计/TF) | 🟡 LiDAR/IMU/odom 基础链路已跑通，仍需长时间稳定性验证 |
| 阶段2: 传感器融合验证 (EKF/AMCL) | 🟡 EKF 有输出但受 RDK X5 内存峰值影响，AMCL 待裁剪地图复测 |
| 阶段3: 地图完善 | 🟡 已生成并裁剪真实外围围栏 SLAM 地图，路径点仍需实车复核 |
| 阶段4: TaskMaster 状态机 | 🟡 基础框架完成，已修复重复 Nav2 goal preemption 问题 |
| 阶段5: 路径点配置 | 🟡 已按裁剪地图换算，仍需 RViz/AMCL 复测 |
| 阶段6: 子模块 (QR/图生文/播报) | 🟡 QR BPU 最小链路实测 14~15 FPS，图生文/播报待联调 |
| 阶段7: 系统集成联调 | 🔴 待进行 |
