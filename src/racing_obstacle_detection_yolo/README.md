# Function Introduction

Using deep learning methods to identify obstacles in the track, using the YOLOv5s model

# Usage

## Preparations

Equipped with a real robot or robot simulation module, including a motion chassis, camera, and RDK kit, and able to operate normally.

## Compile and Run
**1.Compile**

After starting the robot, connect to it via SSH or VNC on the terminal, open the terminal, pull the corresponding code, and compile and install it

```bash
# Pull the target detection code code
mkdir -p ~/racing_ws/src && cd ~/racing_ws/src
git clone https://github.com/wunuo1/racing_obstacle_detection_yolo.git -b feature-x5

# Compile
cd ..
source /opt/tros/setup.bash
colcon build
```

**2.Run object detection function**

```shell
source ~/racing_ws/install/setup.bash
cp -r ~/racing_ws/install/racing_obstacle_detection_yolo/lib/racing_obstacle_detection_yolo/config/ .

# Visualize obstacles on the web (open IP: 8000 in the browser after activating the feature)
export WEB_SHOW=TRUE

# Simulation (using simulation models)
ros2 launch racing_obstacle_detection_yolo racing_obstacle_detection_yolo_simulation.launch.py

# Actual scenario (using models from actual scenarios)
ros2 launch racing_obstacle_detection_yolo racing_obstacle_detection_yolo.launch.py
```


# Principle Overview

RDK obtains environmental data in front of the car through a camera, and the image data is inferred using a trained YOLO model to obtain the image coordinate values of obstacles and published.

# Interface Description

## Topics

### Published Topics

| Name                          | Type                                                     | Description                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /racing_obstacle_detection    | ai_msgs/msg/PerceptionTargets             | Publishes information about obstacles                 |

### Subscribed Topics
| Name                          | Type                                                     | Description                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /hbmem_img或/image_raw       | hbm_img_msgs/msg/HbmMsg1080P或sensor_msgs/msg/Image        | Receive image messages posted by the camera (640x480)     |

## Parameters

| Parameter Name       | Type        | Description    |
| --------------------- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| sub_img_topic       | string | Please configure the received image topic name based on the actual received topic name. The default value is/hbmem_img |
| is_shared_mem_sub   | bool | Whether to use shared memory, please configure according to the actual situation, default value is True |
| config_file | string | Please configure the path for reading the configuration file based on the recognition situation. The default value isconfig/yolov5sconfig_simulation.json |

# Note
This feature pack provides models for identifying obstacles in the Gazebo simulation environment as well as models for identifying obstacles in specific real-world scenarios. If you collect your own dataset for training, please note to replace them.