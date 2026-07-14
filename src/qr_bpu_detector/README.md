# QR BPU Detector

## Overview

`qr_bpu_detector` 面向 RDK X5 和 ROS2 Humble，包含两个节点：

- `qr_bpu_detector_node`：使用 BPU 运行 640×640 NV12 QR 检测模型。
- `qr_roi_decoder_node`：将检测结果与原始相机帧按 `header.stamp` 精确同步，在原分辨率图像上裁剪并使用 ZBar 解码。

检测器每个已处理帧都会发布 `/qr_detection`，包括无目标帧，因此 `ExactTime` 可以匹配同一张原图，不再组合不同时间的检测框和图像。

## Long-range Strategy

远距离链路采用以下策略：

1. 按检测框尺寸计算相对 padding，避免固定 48/96 px 背景抑制二维码放大。
2. 按检测框短边计算缩放，将二维码目标尺寸放大至约 280 px，最大 8×。
3. 依次尝试原灰度、轻度锐化、CLAHE、直方图均衡、Otsu 和自适应阈值。
4. 无检测框时，检测器低频重试中央 70% 区域，提高小目标在模型输入中的像素占比。
5. 无框或有框但连续解码失败时，解码器低频扫描中央区域和全图。
6. 回退放大受最大像素数限制，避免在 RDK X5 上产生过大的临时图像。

默认参数统一保存在 `config/qr_runtime.yaml`。关键值如下：

```text
score_threshold = 0.15
center_retry_interval_frames = 5
center_crop_ratio = 0.70
dynamic_padding_ratio = 0.15
target_qr_pixels = 280
max_upscale = 8
fallback_frame_threshold = 10
fallback_interval_frames = 10
fallback_center_ratio = 0.75
fallback_upscale = 1.5
fallback_max_pixels = 1000000
sync_queue_size = 10
```

旧参数 `roi_padding` 和 `decode_period_ms` 已移除。

## Build

在 RDK X5 上执行：

```bash
cd ~/qr_detect_ws
source /opt/tros/humble/setup.bash

rm -rf build/qr_bpu_detector install/qr_bpu_detector

colcon build \
  --symlink-install \
  --packages-select qr_bpu_detector \
  --event-handlers console_direct+

source install/setup.bash
```

必须确认 `src/` 下只有一个 `qr_bpu_detector/package.xml`，不能再出现嵌套同名包。

## Run

原始图像话题示例：

```bash
ros2 launch qr_bpu_detector qr_bpu_minimal.launch.py \
  image_topic:=/aurora/rgb/image_raw \
  image_msg_type:=raw
```

USB MJPEG 压缩话题示例：

```bash
ros2 launch qr_bpu_detector qr_bpu_minimal.launch.py \
  image_topic:=/image \
  image_msg_type:=compressed
```

如需修改高级参数，复制 `config/qr_runtime.yaml` 后通过以下方式加载：

```bash
ros2 launch qr_bpu_detector qr_bpu_minimal.launch.py \
  config_file:=/absolute/path/to/qr_runtime.yaml \
  image_topic:=/image \
  image_msg_type:=compressed
```

## Interfaces

输入：

- 相机图像：`sensor_msgs/msg/Image` 或 `sensor_msgs/msg/CompressedImage`
- `/qr_detection`：`ai_msgs/msg/PerceptionTargets`

输出：

- `/qr_direction`：奇数为 `顺时针`，偶数为 `逆时针`
- `/qr_number`：有效数字字符串
- `/qr_decode_state`：`WAITING_FOR_TARGET`、`DECODING` 或 `SUCCEEDED`
- `/qr_detection/image/compressed`：可选检测调试图

## Verification

启动后应出现：

```text
QR BPU detector started ... center_retry=on/5 frames, center_ratio=0.70
QR ROI decoder started: exact_sync=true ... target_qr=280 px,
max_upscale=8.0x ... fallback_upscale=1.5x, sync_queue=10
```

检查参数：

```bash
ros2 param get /qr_bpu_detector enable_center_retry
ros2 param get /qr_bpu_detector center_crop_ratio
ros2 param get /qr_roi_decoder dynamic_padding_ratio
ros2 param get /qr_roi_decoder target_qr_pixels
ros2 param get /qr_roi_decoder max_upscale
ros2 param get /qr_roi_decoder sync_queue_size
```

检查时间同步：

```bash
ros2 topic echo /aurora/rgb/image_raw --once
ros2 topic echo /qr_detection --once
```

同一处理帧的两个 `header.stamp` 必须完全一致。

## Distance Test

分别在 1 m、2 m、3 m、4 m 静止测试，每个距离至少观察 30 个检测帧，并记录：

- `/qr_detection` 是否有框及置信度。
- 日志中的二维码框宽高。
- `/qr_number` 是否成功输出及首次成功耗时。
- 原始图像是否清晰、曝光是否稳定。

判断方法：

- 有框但不能解码：检查原图二维码像素、对焦和 ROI 日志。
- 无框但中央重试能检测：保持中央重试配置。
- 中央重试仍无框：提高相机分辨率、缩小镜头视场或补充远距离小目标训练数据。
- 二维码主体低于约 2 px/module 时，插值无法恢复缺失信息，必须从相机分辨率、焦距或物理二维码尺寸解决。

比赛运行时应在 `SCAN_QR` 前停车，并等待曝光和对焦稳定后再判定失败。
