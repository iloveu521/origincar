# OriginCar Broadcast

## Overview

`origincar_broadcast` 是比赛流程中的统一语音播报包。节点订阅文本话题，将请求放入优先级队列，再由后台 worker 串行调用 I2C 语音合成模块，避免多个模块同时访问语音硬件。

默认订阅：

- `/announcement`: `TaskMaster` 发布的比赛流程事件，例如开始、QR 扫码完成、任务完成。
- `/image_description`: `connect_to_pc` 发布的图生文结果，由播报节点直接消费。

QR 方向不再由播报节点直接订阅，避免 `/qr_direction` 和 `/announcement` 对同一次扫码结果重复播报。图生文结果不再经过 `TaskMaster` 转发，避免 PC 通信或语音播报影响导航状态机。

## Build

```bash
cd /userdata/origin_car/broadcast_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch origincar_broadcast broadcast.launch.py
```

开发机无 I2C 硬件时可关闭真实播报，仅观察队列和去重逻辑：

```bash
ros2 launch origincar_broadcast broadcast.launch.py speech_enabled:=false
```

## Parameters

- `source_topics/source_names/source_priorities/source_cooldowns_sec`: 多话题管理配置。
- `queue_max_size`: 队列最大长度，满队列时高优先级请求可替换低优先级请求。
- `duplicate_window_sec`: 全局相同文本去重窗口。
- `i2c_bus`: RDK X5 I2C 总线，默认 `5`。
- `i2c_addr`: 语音模块地址，默认 `48` (`0x30`)。
- `reader/volume/speed/intonation/style/language`: 语音芯片合成参数。
