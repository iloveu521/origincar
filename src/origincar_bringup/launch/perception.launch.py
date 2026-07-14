#!/usr/bin/env python3
"""
perception.launch.py — 感知子系统启动

启动内容:
  - Aurora930 深度相机驱动
  - QR BPU 检测 (BPU 推理 + ROI 解码)
  - 锥桶 YOLO 检测

用法:
  ros2 launch origincar_bringup perception.launch.py
  ros2 launch origincar_bringup perception.launch.py start_qr:=false start_cone_detection:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ── Switches ──
    start_camera = LaunchConfiguration('start_camera', default='true')
    start_qr = LaunchConfiguration('start_qr', default='true')
    start_cone_detection = LaunchConfiguration('start_cone_detection', default='true')

    # ── Camera params ──
    camera_format = LaunchConfiguration('camera_format', default='raw')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('start_camera', default_value='true'))
    ld.add_action(DeclareLaunchArgument('start_qr', default_value='true'))
    ld.add_action(DeclareLaunchArgument('start_cone_detection', default_value='true'))
    ld.add_action(DeclareLaunchArgument(
        'camera_format', default_value='raw',
        description='Camera image format: raw, compressed'))

    # ── Aurora930 深度相机 ──
    aurora930_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('deptrum-ros-driver-aurora930'),
                'launch', 'aurora930_launch.py')),
        condition=IfCondition(start_camera),
    )

    # ── QR BPU 检测 (最小链路: BPU infer + ROI ZBar decode) ──
    qr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('qr_bpu_detector'),
                'launch', 'qr_bpu_minimal.launch.py')),
        condition=IfCondition(start_qr),
    )

    # ── 锥桶 YOLO 检测 ──
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('racing_obstacle_detection_yolo'),
                'launch', 'racing_obstacle_detection_yolo.launch.py')),
        condition=IfCondition(start_cone_detection),
    )

    ld.add_action(aurora930_launch)
    ld.add_action(qr_launch)
    ld.add_action(yolo_launch)

    return ld
