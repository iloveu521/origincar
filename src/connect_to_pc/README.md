# connect_to_pc

## Overview

`connect_to_pc` 是车端运行的 ROS2 Python 功能包，通过 HTTP 连接 PC 端大模型服务，并把返回文本发布为 `/image_description`。

任务链路：

1. 订阅车端相机话题 `/aurora/rgb/image_raw`，缓存最新帧。
2. 收到 `/capture_trigger` 后，仅发送下一帧到 PC。
3. 后台线程将图像缩放为 `320x320` 并编码为 JPEG。
4. HTTP `POST /predict?car_ip=<car_ip>` 发送到 PC 端 `qwen_api_server.py`。
5. 兼容 PC HTTP 响应体和 `POST /receive_result` 主动回调两种返回方式。
6. 车端把最终文本发布到 `/image_description`，由 `origincar_broadcast` 直接播报。

在当前 TaskMaster 集成策略下，TaskMaster 只发布 `/capture_trigger`，不订阅 `/image_description`，也不等待 PC 或语音播报反馈。

## Build

```bash
cd /userdata/origin_car/connect_to_pc_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch connect_to_pc car_pc_bridge.launch.py \
  pc_server_url:=http://192.168.3.12:9999/predict \
  ip_probe_host:=192.168.3.12 \
  trigger_required:=true \
  trigger_topic:=/capture_trigger
```

如果自动探测车端 IP 不符合 PC 回调可达地址，可显式指定：

```bash
ros2 launch connect_to_pc car_pc_bridge.launch.py \
  pc_server_url:=http://192.168.3.12:9999/predict \
  car_ip:=192.168.3.20
```

## Parameters

| Parameter | Default | Description |
|---|---:|---|
| `image_topic` | `/aurora/rgb/image_raw` | 车端相机图像输入 |
| `description_topic` | `/image_description` | 图生文结果本地发布话题 |
| `trigger_required` | `true` | 是否等待触发话题后才发送下一帧 |
| `trigger_topic` | `/capture_trigger` | 触发一次图像发送的话题 |
| `pc_server_url` | `http://192.168.3.12:9999/predict` | PC 端大模型 HTTP 接口 |
| `min_send_period_sec` | `0.3` | 发送限频；同一时间只允许一个请求在途 |
| `retry_backoff_sec` | `5.0` | 请求失败后的退避时间 |
| `resize_width` / `resize_height` | `320` / `320` | 发送前缩放尺寸 |
| `jpeg_quality` | `70` | JPEG 压缩质量 |
| `stop_after_first_result` | `true` | 发布一次有效结果后进入 `SUCCEEDED` |
| `publish_state` | `true` | 是否发布状态机状态 |
| `state_topic` | `/image_description_state` | 状态输出话题 |
| `enable_callback_receiver` | `true` | 是否开启车端 `/receive_result` HTTP 回调服务 |
| `callback_port` | `8888` | 车端 HTTP 回调监听端口 |
| `car_ip` | empty | 为空时通过 UDP probe 自动获取车端局域网 IP |
| `ip_probe_host` | `192.168.3.12` | 自动探测车端 IP 时使用的 PC 侧地址 |

如果需要多个采图点都触发 PC 请求，可在 launch 中设置 `stop_after_first_result:=false`。

## Troubleshooting

如果启动时报 `Port 8888 is in use`，说明车端已有程序占用了 PC 回调用端口。节点会自动关闭回调接收器，并继续使用 `/predict` 的 HTTP 响应体发布 `/image_description`。

也可以显式关闭回调接收：

```bash
ros2 launch connect_to_pc car_pc_bridge.launch.py \
  pc_server_url:=http://192.168.3.12:9999/predict \
  ip_probe_host:=192.168.3.12 \
  enable_callback_receiver:=false
```
