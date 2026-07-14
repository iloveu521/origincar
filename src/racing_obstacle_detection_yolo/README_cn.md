# 功能介绍

基于深度学习的方法识别赛道中的障碍物，使用模型为YOLOv5s

# 使用方法

## 准备工作

具备真实的机器人或机器人仿真模块，包含运动底盘、相机及RDK套件，并且能够正常运行。

## 编译与运行

**1.编译**

启动机器人后，通过终端SSH或者VNC连接机器人，打开终端拉取相应代码并编译安装

```bash
# 拉取目标检测代码代码
mkdir -p ~/racing_ws/src && cd ~/racing_ws/src
git clone https://github.com/wunuo1/racing_obstacle_detection_yolo.git -b feature-x5

# 编译
cd ..
source /opt/tros/humble/setup.bash
colcon build
```

**2.运行障碍物检测功能**

```shell
source ~/racing_ws/install/setup.bash
cp -r ~/racing_ws/install/racing_obstacle_detection_yolo/lib/racing_obstacle_detection_yolo/config/ .

# web端可视化障碍物（启动功能后在浏览器打开 ip:8000）
export WEB_SHOW=TRUE

# 仿真（使用仿真模型）
ros2 launch racing_obstacle_detection_yolo racing_obstacle_detection_yolo_simulation.launch.py

# 实际场景（使用实际场景中的模型）
ros2 launch racing_obstacle_detection_yolo racing_obstacle_detection_yolo.launch.py
```


# 原理简介

地平线RDK通过摄像头获取小车前方环境数据，图像数据通过训练好的YOLO模型进行推理得到障碍物的图像坐标值并发布。

# 接口说明

## 话题

### Pub话题

| 名称                          | 消息类型                                                     | 说明                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /racing_obstacle_detection    | ai_msgs/msg/PerceptionTargets             | 发布障碍物的图像坐标                 |

### Sub话题
| 名称                          | 消息类型                                                     | 说明                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /hbmem_img或/image_raw       | hbm_img_msgs/msg/HbmMsg1080P或sensor_msgs/msg/Image        | 接收相机发布的图片消息（640x480）                   |

## 参数

| 参数名                | 类型        | 说明                                                                                                                                 |
| --------------------- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| sub_img_topic       | string |     接收的图片话题名称，请根据实际接收到的话题名称配置，默认值为/hbmem_img |
| is_shared_mem_sub   | bool | 是否使用共享内存，请根据实际情况配置，默认值为True |
| config_file | string | 配置文件读取路径，请根据识别情况配置，默认值为config/yolov5sconfig_simulation.json |

# 注意
该功能包提供gazebo仿真环境中识别障碍物的模型以及特定的实际场景中识别障碍物的模型，若自行采集数据集进行训练，请注意替换。
