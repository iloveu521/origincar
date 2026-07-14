#!/usr/bin/env python3
"""
base.launch.py — 底盘串口、Ackermann 转换、IMU、里程计、EKF、TF、激光、定位

启动内容:
  - origincar_base (串口驱动 + 可选 Ackermann 转换)
  - 静态 TF (base_footprint → base_link → laser)
  - EKF (odom + imu → odom_combined)

用法:
  ros2 launch origincar_bringup base.launch.py
  ros2 launch origincar_bringup base.launch.py laser_x:=-0.10 laser_yaw:=0.05
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    bringup_dir = get_package_share_directory('origincar_base')

    # ── Launch arguments ──
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    akmcar = LaunchConfiguration('akmcar', default='true')

    # LiDAR 外参
    laser_x = LaunchConfiguration('laser_x', default='-0.10')
    laser_y = LaunchConfiguration('laser_y', default='0.0')
    laser_z = LaunchConfiguration('laser_z', default='0.102')
    laser_roll = LaunchConfiguration('laser_roll', default='0.0')
    laser_pitch = LaunchConfiguration('laser_pitch', default='0.0')
    laser_yaw = LaunchConfiguration('laser_yaw', default='0.05')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument(
        'akmcar', default_value='true',
        description='Start Ackermann cmd_vel converter with base serial driver.'))
    ld.add_action(DeclareLaunchArgument('laser_x', default_value='-0.10'))
    ld.add_action(DeclareLaunchArgument('laser_y', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('laser_z', default_value='0.102'))
    ld.add_action(DeclareLaunchArgument('laser_roll', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('laser_pitch', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument(
        'laser_yaw', default_value='0.05',
        description='Yaw offset from base_link to laser (rad).'))

    # ── 静态 TF ──
    static_tf = launch_ros.actions.Node(
        package='origincar_base',
        executable='origincar_static_tf_node',
        name='origincar_static_tf_node',
        output='screen',
        parameters=[{
            'laser_x': laser_x,
            'laser_y': laser_y,
            'laser_z': laser_z,
            'laser_roll': laser_roll,
            'laser_pitch': laser_pitch,
            'laser_yaw': laser_yaw,
        }],
    )

    # ── 底盘串口驱动 ──
    origincar_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'base_serial.launch.py')),
        launch_arguments={'akmcar': akmcar}.items(),
    )

    # ── EKF ──
    ekf_config = Path(bringup_dir, 'config', 'ekf.yaml')
    robot_ekf = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odom_combined')],
    )
    # EKF 延迟 4 秒启动，避免 DDS 初始化内存峰值叠加
    ekf_delayed = TimerAction(period=4.0, actions=[robot_ekf])

    ld.add_action(static_tf)
    ld.add_action(origincar_base)
    ld.add_action(ekf_delayed)

    return ld
