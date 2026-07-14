# OriginCar 21届智能车竞赛开发计划

> 创建日期: 2026-06-12 | 最后更新: 2026-07-08
> 开发环境: x86 Ubuntu 22.04 → RDK X5 (arm64) 部署
> 车辆平台: OriginCar (阿克曼底盘, STM32下位机, RDK X5)

---

## 一、物理环境与硬件现状

### 1.1 比赛场地物理特征

```
┌───────────────────────────┐
│                           │   5m×5m 场地
│    A区 (蓝)                │   一整圈围栏板包围
│                           │   内部无任何物理隔离
│    B区 (黄)               │   ABC三区仅靠地面颜色区分
│                           │
│    ┌──────────┐           │
│    │ C区(绿+黄)│           │
│    └──────────┘           │
│                           │
└───────────────────────────┘

LiDAR 能看到:  只有外围四面墙（围栏板）
LiDAR 看不到:  B区通道边界、C区诊疗室边界、黄色环道、所有地面颜色标记
AMCL 定位:     墙角处好(有L形特征) / 墙面中段差(对称) / 场地中心差(四面墙都远)
```

### 1.2 比赛任务流程

```
P点出发 → [子任务1] 避障行驶到任务发布点，扫二维码获取方向(顺/逆时针)
       → [子任务2] 通过B区通道 → C区黄色环道行驶一周 + 识别图生文标记牌
       → [子任务3] 通过B区返回A区 → 回到P点停稳
限时 180秒，碰障碍物 +10s/次
```

### 1.3 硬件验证状态

| 模块 | 状态 | 说明 |
|------|------|------|
| 底盘驱动 | ✅ 已验证 | 去年跑过，STM32串口通信/IMU/里程计正常 |
| USB相机 | ✅ 已验证 | 去年纯视觉方案用过 |
| Aurora930 深度相机 | ❓ 未确认 | 去年可能没用 |
| LiDAR N10 | ❌ 未验证 | 驱动已编译(lidar_ws)，实车从未跑过 |
| IMU 校准 | ✅ 已校准 | v0.1.1: 92.3s静置标定，陀螺仪/加速度计零偏已补偿 |
| EKF 融合 | ❌ 未验证 | 配置文件存在，从未实车跑过 |
| Nav2 全栈 | ❌ 未使用 | 去年纯视觉寻线避障，今年全新引入 |
| TF 树 | ⚠️ 部分完成 | 尺子粗测 + base_footprint移至后轮中轴 |

### 1.4 已知代码问题

1. **IMU双重滤波**: `Quaternion_Solution.cpp`（Mahony）跑了一遍，`imu_filter_madgwick` 又跑一遍
2. ~~**IMU参数为手册值**~~: ✅ 已修复(v0.1.1)，添加零偏补偿常量
3. **里程计系数为经验值**: X方向乘1.03，Y方向乘1.125
4. **gyro_link与base_footprint重合**: 官方设计如此，base_footprint已手动移至后轮中轴
5. **IMU协方差矩阵不合理**: angular_velocity X/Y=1e6(不可信)、Z=1e-6(极可信，但实际三轴精度相同)；linear_acceleration 全为0(未设置)。会导致 EKF 对 IMU 数据权重分配异常

### 1.5 地图

当前策略：**使用实测 SLAM 外围围栏地图，不再直接使用带手绘内墙/区域线的静态地图作为 AMCL 定位地图**。

原因：
- 赛题物理场地为 5m×5m，只有外围围栏板是真实可被 LiDAR 观测到的几何结构。
- B区通道边界、C区诊疗室边界、黄色环道、地面颜色标记等均为地面语义/颜色信息，LiDAR 看不到。
- 如果在用于 AMCL/SLAM 定位的 occupancy map 中手动补充“物理环境里不存在的墙”，AMCL 会尝试用真实 scan 去匹配虚假墙体，容易导致定位失败或跳变。

保留的历史方案（仅作语义/任务约束参考，不直接作为 LiDAR 定位地图）：
- 手画 300×300 pgm
- 分辨率: 0.02 m/pixel → 覆盖 6m×6m
- 25px黑色边框 = 0.5m 边界
- 内部 250×250px = 5m×5m 比赛区域
- 白色=可行走，黑色=不可行走(B区通道边界、C区绿色禁行区)

2026-07-07 实测 SLAM 地图处理记录：
- 原始保存图：`dev_ws/src/origincar/origincar_base/map/map_uncropped_462x439.pgm`
- 原始尺寸：462×439 px，resolution=0.02m/px，覆盖约 9.24m×8.78m
- 原始图中黑色主框以外存在零散漂点，导致地图尺寸被异常撑大。
- 主黑框自动识别 bbox：左上 `(147,31)`，右下 `(417,299)`（原始图像素坐标，含边界）。
- 裁剪后导航图：`dev_ws/src/origincar/origincar_base/map/map.pgm`
- 裁剪后尺寸：271×269 px，resolution=0.02m/px，覆盖约 5.42m×5.38m
- 裁剪后 `origin`: `[-0.481, -0.463, 0]`
- 裁剪后地图覆盖范围：x≈[-0.481, 4.939]m，y≈[-0.463, 4.917]m，基本对应 5m×5m 场地加约 0.45~0.5m 围栏/边界余量。

后续建议：
- AMCL/SLAM 定位地图只保留真实 LiDAR 可见的外围围栏/障碍物。
- B/C 区通道、黄色环道、禁行区等赛题语义约束应在任务层、waypoints、costmap filter 或自定义语义层中处理，不应作为虚假墙体写入 AMCL 定位地图。

---

## 二、技术方案

### 2.1 整体架构

```
┌──────────────────────────────────────────────┐
│         TaskMaster (C++ Node, 自定义FSM)       │
│                                               │
│  A区: Nav2 NavigateToPose                    │
│       (AMCL + SMAC规划 + MPPI控制)            │
│       + LiDAR costmap 动态避锥桶               │
│                                               │
│  B区: Nav2 NavigateToPose                    │
│       (静态代价地图约束通道边界)                 │
│                                               │
│  C区: Nav2 FollowPath (预定义环道路径点)        │
│       备选: HSV视觉线跟踪 + PID                │
│       + YOLO 视觉避锥桶                        │
│       + 标记牌处暂停拍照 → 大模型图生文 → 语音播报 │
│                                               │
│  返回: Nav2 NavigateToPose (回P点停车)          │
└──────────────────────────────────────────────┘
          │
          │ Nav2 Action Client 调用
          ▼
┌──────────────────────────────────────────────┐
│            Nav2 全栈 (不改动)                  │
│  AMCL + SMAC Hybrid(Reeds-Shepp) + MPPI(Ack) │
│  Global/Local Costmap + BT Navigator          │
│  参数文件: param_mini_akm.yaml                │
└──────────────────────────────────────────────┘
          │
          │ /cmd_vel
          ▼
┌──────────────────────────────────────────────┐
│         origincar_base (不改动)                │
│  串口→STM32→电机/舵机                          │
│  发布 /odom, /imu/data_raw                    │
└──────────────────────────────────────────────┘
```

### 2.2 各模块选型

| 模块 | 选型 | 状态 |
|------|------|------|
| 状态机 | C++ 自定义 enum-class FSM + rclcpp Timer | 新建 |
| A/B区导航 | Nav2 NavigateToPose Action | 已有配置 |
| C区导航(首选) | Nav2 FollowPath + 预定义环道路径点 | 新建 |
| C区导航(备选) | OpenCV HSV黄色检测 + PID线跟踪 | 新建 |
| 全局规划器 | SMAC Hybrid (Reeds-Shepp) — **不改** | 已有 |
| 局部控制器 | MPPI (Ackermann, min_turn_r=0.35m) — **不改** | 已有 |
| 定位 | AMCL + EKF(odom+imu) | 已有配置(待实车验证) |
| QR扫描 | **复用去年方案**: YOLO检测ROI → ZBar解码 → 直接半场扫码 | 复用 racing_ws |
| 避障(A/B区) | LiDAR → Nav2 costmap → MPPI自动避障 | 已有 |
| 避障(C区备选) | YOLOv8s 锥桶检测 | 复用 racing_ws |
| 图生文 | HTTP POST 图片 → **PC端部署的大模型** → 文字返回 | 复用 ros2_ws |
| 播报 | **语音播报**(已有驱动，待完成) | 新建 |
| 屏幕 | **不购买**，纯语音播报 | - |
| Ego-Planner | **暂不使用**（分析见2.3节） | 备用 |

### 2.3 Ego-Planner-2D-ROS2 深度评估

**项目地址**: [JackJu-HIT/Ego-Planner-2D-ROS2](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2)
**上游来源**: [ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner) (浙大高飞团队, RA-L 2021)
**原始论文**: *EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors*

#### 2.3.1 算法核心

EGO-Planner 是一个**无需构建ESDF（欧几里得符号距离场）的梯度局部规划器**，原始设计面向四旋翼飞行器，规划时间约 1ms。

**2D改造版 (JackJu-HIT) 的核心管线**:

```
全局路径点 ──▶ B样条参数化 ──▶ 碰撞检测 ──▶ 梯度优化 ──▶ 时间重分配 ──▶ 轨迹精修 ──▶ 光滑轨迹
                  │               │              │              │              │
                  │           A*绕障搜索     L-BFGS优化器    可行性检查      二次优化
                  │               │          (≤200迭代)    (速度/加速度)   (fitness代价)
                  │               │              │
                  └───────────────┴──────────────┘
                          控制点分段调整
```

**代价函数（Rebound 阶段）**:

| 代价项 | 权重 | 含义 |
|--------|------|------|
| `f_smoothness` | λ1=12.0 | B样条 jerk（三阶导）最小化，保证轨迹光滑 |
| `f_distance` | λ2=1.0 | 碰撞距离代价，推离障碍物至 clearance 距离外 |
| `f_feasibility` | λ3=3.0 | 速度/加速度超限惩罚（默认 max_vel=1.0, max_acc=0.5） |

**代价函数（Refine 阶段）**:

| 代价项 | 权重 | 含义 |
|--------|------|------|
| `f_smoothness` | λ1 | 同上 |
| `f_fitness` | λ4=1.0 | 跟踪原始参考路径的偏差代价 |
| `f_feasibility` | λ3 | 同上 |

**代码中已注释掉的功能**:
- `calKappaCost` (曲率代价, λ5=0.1, k_max=2): 限制轨迹曲率 — **注释标注"约束还不稳定"**
- `calTurnCost` (转向角代价, w_max=1.0): 限制转向角速度 — **注释标注"约束还不稳定"**

#### 2.3.2 ROS2 节点结构

`TrajectoryAndObstaclesPublisher` (继承 `rclcpp::Node`):

**订阅**:
| 话题 | 消息类型 | 用途 |
|------|----------|------|
| `/goal_pose` | PoseStamped | **被改用作添加障碍物**（非导航目标） |
| `/clicked_point` | PointStamped | Rviz Publish Point 工具逐点添加全局路径 |
| `/initialpose` | PoseWithCovarianceStamped | 设置机器人初始位姿 |
| `/trigger_plan` | Bool | 手动触发/停止规划 |

**发布**:
| 话题 | 消息类型 | 用途 |
|------|----------|------|
| `visual_local_trajectory` | Path | 优化后的局部轨迹（路径点序列） |
| `visual_global_path` | Path | 原始全局路径 |
| `visual_obstacles` | PointCloud2 | 原始障碍物点云 |
| `visual_local_obstacles` | PointCloud2 | 膨胀后障碍物点云 |
| `trajectories` | MarkerArray | A* 搜索路径可视化 |

**关键参数（硬编码在头文件中）**:
```cpp
double max_vel_ = 2.0;           // 最大速度
double max_acc_ = 3.0;           // 最大加速度
double max_jerk_ = 4.0;          // 最大加加速度
double map_resolution_ = 0.1;    // GridMap分辨率
double map_x_size_ = 50.0;       // 地图X尺寸
double map_y_size_ = 50.0;       // 地图Y尺寸
double map_inflate_value_ = 1.0; // 障碍物膨胀半径
```

**定时器**: 200ms (5Hz) 规划循环

#### 2.3.3 核心模块源码分析

**GridMap2D** (`planner/GridMap2D/`):
- 2D 栅格地图，分辨率默认 0.1m
- 支持障碍物设置 (`setObstacle`)、膨胀 (`inflate`)、碰撞查询 (`getInflateOccupancy`)
- 世界坐标 ↔ 网格坐标转换
- **独立于 Nav2 costmap，完全不互通**

**A* 搜索** (`planner/path_searching/src/dyn_a_star.cpp`):
- 当 B样条控制点落入膨胀障碍物区域时触发
- 在障碍物段两端搜索绕障路径
- 返回 A* 路径用于引导控制点调整方向

**B样条优化器** (`planner/bspline_opt/src/bspline_optimizer.cpp`):
- 使用 L-BFGS 求解器（`lbfgs.hpp`，内嵌的轻量实现）
- `rebound_optimize()`: 碰撞段重优化，最多重试 3 次
- `refine_optimize()`: 时间重分配后精修
- `check_collision_and_rebound()`: 持续碰撞检测，检测到新碰撞时重新触发优化
- 2D 梯度计算（jerk/acc/vel/碰撞距离 的解析梯度，2行×N列矩阵）

**UniformBspline** (`planner/bspline_opt/src/uniform_bspline.cpp`):
- 3 阶 B样条轨迹表示
- 支持评估位置/速度/加速度 (`evaluateDeBoorT`)
- 可行性检查 (`checkFeasibility`)
- 时间拉伸 (`lengthenTime`)

#### 2.3.4 与 OriginCar 集成需要的改造

如果要将 Ego-Planner 集成到 OriginCar，需要做以下工作：

| 改造项 | 工作量 | 说明 |
|--------|--------|------|
| 障碍物输入 | 中 | 订阅 `/scan` → 转换为 GridMap 障碍点 → 调用 `setObstacles()` |
| 位姿输入 | 低 | 订阅 `/odom_combined` 或 `/tf` → 调用 `setCurrentVehiclePos()` |
| 全局路径输入 | 低 | 从 SMAC Planner 或预定义路径点传入 |
| cmd_vel 输出 | **高** | 需要写一个轨迹跟踪控制器（Pure Pursuit / Stanley / MPC），将路径点序列转为 `/cmd_vel` |
| 阿克曼约束 | **高** | `calKappaCost` 和 `calTurnCost` 被注释标注为不稳定，要实现阿克曼最小转弯半径约束需要自己调试这两个代价项 |
| Nav2 集成 | 中 | 作为 Nav2 的 planner plugin 或 controller plugin 注册，需实现对应接口 |
| 参数调优 | 中 | λ1/λ2/λ3/λ4、max_vel/max_acc、膨胀半径等需要针对 OriginCar 调参 |

**总预估工作量**: 3-5 天（如果阿克曼约束能调通），否则无法用于阿克曼底盘。

#### 2.3.5 结论

**不采纳**。理由按优先级排列：

1. **阿克曼约束缺失是最致命的**。B样条可能生成阿克曼底盘无法执行的急转弯。代码中曲率/转向约束明确标注"不稳定"，作者自己也承认没调通
2. **输出是路径点不是 cmd_vel**。需要额外开发轨迹跟踪控制器，引入新的调参变量
3. **集成成本高**。自建 GridMap 与 Nav2 costmap 隔离，需要写 LiDAR→GridMap 的桥接代码
4. **场景不匹配**。EGO-Planner 为无人机高速避障设计（穿树林、1ms重规划），OriginCar 以 0.35m/s 在 5m×5m 场地低速行驶，优势无法体现
5. **MPPI 已足够**。现有 MPPI 控制器已配置 Ackermann 约束、Nav2 costmap 集成、直接输出 cmd_vel，对当前场景完全胜任

**保留为后续备选**: 如果 Nav2 全流程跑通后，实车测试发现 MPPI 轨迹质量不足以满足比赛需求，可重新评估。届时优先关注 `calKappaCost`/`calTurnCost` 是否已被上游修复。

---

## 三、执行阶段

### 阶段1: 物理验证 🟡 部分完成

#### 1.1 LiDAR 实车验证 ✅
- ✅ 启动 `lslidar_driver`，验证 `/scan` 话题有数据（点云正常）

#### 1.2 IMU 校准 ✅ (v0.1.1)
- ✅ 静置录制92.3s `/imu/data_raw` bag (1847采样点)
- ✅ 陀螺仪/加速度计三轴零偏已补偿到 `origincar_base.cpp`
- ✅ IMU 协方差矩阵修复：angular_velocity 三轴均 1e-6，linear_acceleration 三轴均 1e-3

#### 1.3 里程计校准 🟡 部分完成
- ✅ 线性校准：自走0.905m，odom=0.9864m，修正后 SCALE_X=0.944997, SCALE_Y=1.032157
- ❌ 角速度校准：待完成（Ackermann 需画圆，暂跳过）

#### 1.4 TF 树说明
- `base_footprint → base_link`：静态TF发布（0.092m X方向），**不通过URDF**
- `base_link → laser`：静态TF发布（默认 x=0.083m, z=0.102m），**不通过URDF**；`slim_bringup.launch.py` 暴露 `laser_yaw` 等外参参数，用于在 RViz 中校正雷达安装偏角
- URDF 仅发布车体装饰链路（board_link/camera/wheels），对导航无用
- Nav2 不需要 robot_state_publisher，slim_bringup 的静态TF已足够

```bash
# 验证命令
ros2 run tf2_tools view_frames
# 期望链: odom_combined → base_footprint → base_link → laser

# 雷达扫描整体相对地图有小角度偏差时，优先调 base_link→laser yaw，而不是旋转 AMCL 地图
ros2 launch origincar_base slim_bringup.launch.py laser_yaw:=0.035
```

---

### 阶段2: 传感器融合与导航验证 🟡 进行中

#### 2.1 Nav2 配置修复 ✅
- ✅ 删除 param_mini_akm.yaml 中不存在的 scan2（local/global costmap 均已修复）
- ✅ start_all.launch.py 改用 slim_bringup（避免 OOM，省去 URDF + imu_filter ~684MB）

#### 2.2 EKF 验证
```bash
# 环境准备
killall node 2>/dev/null; kill -9 $(pgrep openclaw-gateway) 2>/dev/null
free -h   # 确认可用 > 3GB

# 启动
ros2 launch origincar_base slim_bringup.launch.py
ros2 launch lslidar_driver lsn10_launch.py

# 验证
ros2 topic hz /odom_combined     # 应 30Hz
ros2 topic echo /odom_combined --once
ros2 run tf2_tools view_frames
```
**通过标准**: 推动机器人 x/y 有变化；静止时 yaw 漂移 < 0.01 rad/min

2026-07-06/07 联调记录：
- `slim_bringup.launch.py` 已使用默认 `ekf_filter_node` 参数命名空间，避免 `ekf.yaml` 不生效导致 `/odom_combined` 无输出。
- 在 RDK X5 上观察到 `ekf_node` 曾被 OOM Killer 杀死，峰值 RSS 约 1.3GB。当前已通过合并静态 TF、降低 EKF/SLAM/Nav2 负载、关闭 RViz `/scan` 默认订阅等方式降低峰值。
- 保留 `base_no_ekf.launch.py` + `slam_mapping_no_ekf.launch.py` 作为低内存建图 fallback；正式导航仍优先验证 EKF + AMCL。

#### 2.3 AMCL 定位验证
```bash
# 一键启动底盘+Nav2（使用手画地图）
ros2 launch origincar_base start_all.launch.py
ros2 launch lslidar_driver lsn10_launch.py

# 给初始位姿
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{ header: {frame_id: map}, pose: {pose: {position: {x: 0.5, y: 0.3}}}}" --once
```
**通过标准**: 粒子收敛（不发散），手推车跟踪位置误差 < 10cm

#### 2.4 基础 Nav2 单点导航测试
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{ pose: { header: { frame_id: 'map' }, pose: { position: { x: 1.0, y: 0.5 }, orientation: { w: 1.0 } } } }"
```
**通过标准**: 机器人到达目标附近 < 25cm，路径合理

---

### 阶段3: 地图与路径点标定 🟡 进行中

**建图策略**（场地内部无物理墙壁，仅外围有围栏）：
- Step 1: SLAM 建外围轮廓地图（slam_toolbox）
- Step 2: 对实测 SLAM 地图裁剪，去掉主场地外漂点，保留真实 LiDAR 可见外围围栏
- Step 3: B 区通道、C 区禁行区、黄色环道等语义约束放到任务层/路径点/costmap filter，不写入 AMCL 定位地图

**路径点实测标定**（AMCL 收敛后读取 `/amcl_pose`）：
```bash
ros2 topic echo /amcl_pose --once
```
将机器人放到关键位置，记录实测坐标更新 `origincar_task/config/waypoints.yaml`：
- `point_P`：发车/停车点
- `qr_point`：QR 码任务发布区
- `zone_b_a_side`：B区A侧入口
- `zone_b_c_side`：B区C侧出口（C区入口）
- `ring_ccw/cw`：C区环道逐点测量（目前仅估算）

2026-07-07 已完成：
- 保存原始 SLAM 图到 `origincar_base/map/map_uncropped_462x439.*`。
- 将导航地图裁剪为 `271x269`，覆盖约 `5.42m x 5.38m`，`origin=[-0.481, -0.463, 0]`。
- `waypoints.yaml` 已按裁剪后 map frame 换算；`waypoints_field_origin_5x5.yaml` 保留原 5m x 5m 场地坐标，便于后续复核。

---

### 阶段4: TaskMaster 状态机 🟡 框架完成，正在联调

**当前进展**：框架已写，导航 stub（超时触发）。需完善：

```
IDLE → NAV_TO_QR → SCAN_QR → NAV_TO_B → CROSS_B
→ RING_NAV → CAPTURE_SIGN(停顿2-3s) → EXIT_C → NAV_TO_P → PARK → DONE
```

需改造：
- `handle_nav_*()`: 等待 Nav2 action 完成回调，替换超时触发
- `handle_scan_qr()`: 订阅 `/qr_result`（qr_bpu_detector 节点）
- `handle_capture_sign()`: 无 PC 时停顿 2~3s 再继续，VLM 待后续接入
- 已修复重复发送 NavigateToPose goal 导致 Nav2 preemption/abort 的问题，`NAV_GOAL_SEND_COUNT=1`。

**调试启动顺序**：
```bash
ros2 launch origincar_base slim_bringup.launch.py
ros2 launch lslidar_driver lsn10_launch.py
ros2 launch origincar_task task_nav.launch.py task_delay:=45
ros2 launch qr_bpu_detector qr_bpu_minimal.launch.py
```

---

### 阶段5: 子模块集成 🟡 代码就绪，待联调

| 模块 | 工作空间 | 状态 |
|------|---------|------|
| QR BPU 推理 | `qr_detect_ws` | 代码完整，待实车测试 |
| 语音播报管理 | `broadcast_ws` | 代码完整，待联调 |
| VLM 图像识别 | `speaker_ws` | 需 PC 连接，暂跳过 |
| 车→PC 桥接 | `connect_to_pc_ws` | 需 PC 连接，暂跳过 |

2026-07-07 实车结果：
- QR BPU 检测默认输入已切到 `/aurora/rgb/image_raw` + `raw`。
- 最小比赛 launch：`qr_bpu_minimal.launch.py`。
- 实测 `qr_bpu_detector_node` 约 14~15 FPS，BPU `avg_infer` 约 5.7~6.4ms，检测框稳定输出。

---

### 阶段6: 系统集成联调 🔴 待进行

- QR扫描: 复用 racing_ws/qrtest
- QR解码: 2026-07-05 新增 `qr_bpu_detector/qr_roi_decoder_node`，基于 BPU 检测框裁剪 ROI，持续 ZBar 解码并发布方向文本
- 图生文: 2026-07-05 新增 `connect_to_pc_ws/src/connect_to_pc`，车-PC 之间采用普通 HTTP 传输图片和结果，不使用 ROS2 跨机 topic/message；车端将 PC 返回或回调的文本统一发布到 `/image_description`
- 语音播报: 2026-07-05 新增 `broadcast_ws/src/origincar_broadcast`，采用话题方式订阅 `/qr_direction` 与 `/image_description` 两路文本结果，按到达顺序进入队列并串行驱动 I2C 语音模块
- 无屏幕模块
- 一键 task_launch.py 启动全栈
- 实车全流程跑通（IDLE→DONE）
- 监控关键话题：`/rosout | grep task_master`、`/qr_result`、`/odom_combined`

---

## 四、待解决问题

| 问题 | 优先级 | 状态 |
|------|--------|------|
| EKF 实车验证 | 🔴 高 | 待做（阶段2.2） |
| AMCL 定位验证 | 🔴 高 | 待做（阶段2.3，使用裁剪后实测地图） |
| Nav2 单点导航测试 | 🔴 高 | 待做（阶段2.4，使用 `task_nav.launch.py`） |
| SLAM 建真实地图 | 🟡 高 | 已获得裁剪版外围地图，仍需复测稳定性 |
| 路径点实测标定 | 🟡 高 | 已按裁剪地图换算，仍需 RViz/AMCL 复核 |
| TaskMaster Nav2 回调完善 | 🟡 中 | 重复 goal 问题已修，仍需全流程联调 |
| 标记牌实际坐标 | 🟡 中 | 待做（阶段5） |
| 大模型服务部署 | 🟡 中 | 待做（阶段6，车端桥接包已新增，PC 端服务仍需实机部署和压力测试） |
| 里程计角速度校准 | 🟡 中 | 暂跳过 |
| IMU 双重滤波修复 | 🟡 中 | 待做 |
| VLM / TTS 接入 | 🟡 中 | 需 PC 连接 |
| C区首选/备选方案定稿 | 🟢 低 | AMCL跑通后评估 |
| Ego-Planner备用 | 🟢 低 | Nav2全流程跑通后 |
| DDS 竞赛环境干扰 | 🔴 高 | 已加入 ROS_DOMAIN_ID + FastDDS Discovery Server 流程，需赛场压测 |
| RDK X5 OOM | 🔴 高 | 已降低 EKF/SLAM/Nav2/RViz 负载，需长时间运行压测 |



