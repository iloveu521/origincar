#!/usr/bin/env python3
"""
competition.launch.py — 比赛总启动文件

启动全部子系统:
  - base.launch.py      (底盘、TF、EKF)
  - perception.launch.py (相机、QR、锥桶)
  - mission.launch.py    (TaskMaster、PC桥接、播报)
  - LiDAR (lslidar_driver)

所有参数均暴露为 launch argument，方便调试。

用法:
  # 全量启动
  ros2 launch origincar_bringup competition.launch.py

  # 只启动底盘和 LiDAR
  ros2 launch origincar_bringup competition.launch.py \
    start_camera:=false start_qr:=false start_cone_detection:=false \
    start_task:=false start_pc_bridge:=false start_broadcast:=false

  # 覆盖速度参数
  ros2 launch origincar_bringup competition.launch.py cruise_speed:=0.90

  # 覆盖 QR 方向
  ros2 launch origincar_bringup competition.launch.py default_qr_direction:=ccw
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ── 总开关 ──
    start_base = LaunchConfiguration('start_base', default='true')
    start_lidar = LaunchConfiguration('start_lidar', default='true')
    start_camera = LaunchConfiguration('start_camera', default='true')
    start_qr = LaunchConfiguration('start_qr', default='true')
    start_cone_detection = LaunchConfiguration('start_cone_detection', default='true')
    start_task = LaunchConfiguration('start_task', default='true')
    start_pc_bridge = LaunchConfiguration('start_pc_bridge', default='true')
    start_broadcast = LaunchConfiguration('start_broadcast', default='true')

    # ── 底盘参数 ──
    laser_x = LaunchConfiguration('laser_x', default='-0.10')
    laser_yaw = LaunchConfiguration('laser_yaw', default='0.05')
    akmcar = LaunchConfiguration('akmcar', default='true')

    # ── 速度参数 ──
    cruise_speed = LaunchConfiguration('cruise_speed', default='0.85')
    qr_scan_speed_gain = LaunchConfiguration('qr_scan_speed_gain', default='0.85')
    turn_speed_gain = LaunchConfiguration('turn_speed_gain', default='0.50')
    turn_angular_gain = LaunchConfiguration('turn_angular_gain', default='1.50')
    capture_speed_gain = LaunchConfiguration('capture_speed_gain', default='0.60')
    max_angular_z = LaunchConfiguration('max_angular_z', default='1.80')

    # ── 避障参数 ──
    cone_confidence = LaunchConfiguration('cone_confidence', default='0.35')
    obstacle_slow_distance = LaunchConfiguration('obstacle_slow_distance', default='0.45')
    obstacle_avoid_distance = LaunchConfiguration('obstacle_avoid_distance', default='0.45')
    obstacle_backup_distance_threshold = LaunchConfiguration(
        'obstacle_backup_distance_threshold', default='0.20')
    obstacle_backup_speed_gain = LaunchConfiguration(
        'obstacle_backup_speed_gain', default='0.70')
    obstacle_backup_distance = LaunchConfiguration(
        'obstacle_backup_distance', default='0.40')
    obstacle_backup_timeout_sec = LaunchConfiguration(
        'obstacle_backup_timeout_sec', default='1.50')
    rear_clearance_distance = LaunchConfiguration(
        'rear_clearance_distance', default='0.25')

    # ── 控制参数 ──
    control_frequency = LaunchConfiguration('control_frequency', default='20.0')
    capture_radius = LaunchConfiguration('capture_radius', default='0.25')
    scan_timeout_sec = LaunchConfiguration('scan_timeout_sec', default='0.30')
    cone_detection_timeout_sec = LaunchConfiguration(
        'cone_detection_timeout_sec', default='0.25')
    obstacle_clear_hold_sec = LaunchConfiguration(
        'obstacle_clear_hold_sec', default='0.40')
    default_qr_direction = LaunchConfiguration('default_qr_direction', default='cw')
    lookahead_dist = LaunchConfiguration('lookahead_dist', default='0.22')
    pass_radius = LaunchConfiguration('pass_radius', default='0.35')
    reverse_pass_radius = LaunchConfiguration('reverse_pass_radius', default='0.30')
    goal_tolerance = LaunchConfiguration('goal_tolerance', default='0.35')
    enable_obstacle_avoidance = LaunchConfiguration(
        'enable_obstacle_avoidance', default='true')

    # ── PC Bridge ──
    pc_server_url = LaunchConfiguration('pc_server_url',
                                        default='http://192.168.3.12:9999/predict')
    enable_callback_receiver = LaunchConfiguration(
        'enable_callback_receiver', default='false')

    # ── Other ──
    camera_format = LaunchConfiguration('camera_format', default='raw')
    waypoints_file = LaunchConfiguration('waypoints_file', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ld = LaunchDescription()

    # ═══════════════════════════════════════════
    # Big switches
    # ═══════════════════════════════════════════
    ld.add_action(DeclareLaunchArgument('start_base', default_value='true'))
    ld.add_action(DeclareLaunchArgument('start_lidar', default_value='true'))
    ld.add_action(DeclareLaunchArgument('start_camera', default_value='true'))
    ld.add_action(DeclareLaunchArgument('start_qr', default_value='true'))
    ld.add_action(DeclareLaunchArgument('start_cone_detection', default_value='true'))
    ld.add_action(DeclareLaunchArgument('start_task', default_value='true'))
    ld.add_action(DeclareLaunchArgument('start_pc_bridge', default_value='true'))
    ld.add_action(DeclareLaunchArgument('start_broadcast', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))

    # Base
    ld.add_action(DeclareLaunchArgument('laser_x', default_value='-0.10'))
    ld.add_action(DeclareLaunchArgument(
        'laser_yaw', default_value='0.05',
        description='Yaw offset from base_link to laser (rad).'))
    ld.add_action(DeclareLaunchArgument('akmcar', default_value='true'))

    # Speed
    ld.add_action(DeclareLaunchArgument('cruise_speed', default_value='0.85'))
    ld.add_action(DeclareLaunchArgument('qr_scan_speed_gain', default_value='0.85'))
    ld.add_action(DeclareLaunchArgument('turn_speed_gain', default_value='0.50'))
    ld.add_action(DeclareLaunchArgument('turn_angular_gain', default_value='1.50'))
    ld.add_action(DeclareLaunchArgument('capture_speed_gain', default_value='0.60'))
    ld.add_action(DeclareLaunchArgument('max_angular_z', default_value='1.80'))

    # Obstacle
    ld.add_action(DeclareLaunchArgument('cone_confidence', default_value='0.35'))
    ld.add_action(DeclareLaunchArgument('obstacle_slow_distance', default_value='0.45'))
    ld.add_action(DeclareLaunchArgument('obstacle_avoid_distance', default_value='0.45'))
    ld.add_action(DeclareLaunchArgument('obstacle_backup_distance_threshold', default_value='0.20'))
    ld.add_action(DeclareLaunchArgument('obstacle_backup_speed_gain', default_value='0.70'))
    ld.add_action(DeclareLaunchArgument('obstacle_backup_distance', default_value='0.40'))
    ld.add_action(DeclareLaunchArgument('obstacle_backup_timeout_sec', default_value='1.50'))
    ld.add_action(DeclareLaunchArgument('rear_clearance_distance', default_value='0.25'))

    # Control
    ld.add_action(DeclareLaunchArgument('control_frequency', default_value='20.0'))
    ld.add_action(DeclareLaunchArgument('capture_radius', default_value='0.25'))
    ld.add_action(DeclareLaunchArgument('scan_timeout_sec', default_value='0.30'))
    ld.add_action(DeclareLaunchArgument('cone_detection_timeout_sec', default_value='0.25'))
    ld.add_action(DeclareLaunchArgument('obstacle_clear_hold_sec', default_value='0.40'))
    ld.add_action(DeclareLaunchArgument('default_qr_direction', default_value='cw'))
    ld.add_action(DeclareLaunchArgument('lookahead_dist', default_value='0.22'))
    ld.add_action(DeclareLaunchArgument('pass_radius', default_value='0.35'))
    ld.add_action(DeclareLaunchArgument('reverse_pass_radius', default_value='0.30'))
    ld.add_action(DeclareLaunchArgument('goal_tolerance', default_value='0.35'))
    ld.add_action(DeclareLaunchArgument('enable_obstacle_avoidance', default_value='true'))

    # PC Bridge
    ld.add_action(DeclareLaunchArgument(
        'pc_server_url', default_value='http://192.168.3.12:9999/predict'))
    ld.add_action(DeclareLaunchArgument('enable_callback_receiver', default_value='false'))

    # Other
    ld.add_action(DeclareLaunchArgument('camera_format', default_value='raw'))
    ld.add_action(DeclareLaunchArgument('waypoints_file', default_value='',
        description='Override path to waypoints YAML file.'))

    # ═══════════════════════════════════════════
    # Base (chassis, TF, EKF)
    # ═══════════════════════════════════════════
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('origincar_bringup'),
                'launch', 'base.launch.py')),
        launch_arguments={
            'laser_x': laser_x,
            'laser_yaw': laser_yaw,
            'akmcar': akmcar,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(start_base),
    )

    # ═══════════════════════════════════════════
    # LiDAR
    # ═══════════════════════════════════════════
    lidar_share = get_package_share_directory('lslidar_driver')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_share, 'launch', 'lsn10_launch.py')),
        condition=IfCondition(start_lidar),
    )

    # ═══════════════════════════════════════════
    # Perception (camera, QR, cone YOLO)
    # ═══════════════════════════════════════════
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('origincar_bringup'),
                'launch', 'perception.launch.py')),
        launch_arguments={
            'start_camera': start_camera,
            'start_qr': start_qr,
            'start_cone_detection': start_cone_detection,
            'camera_format': camera_format,
        }.items(),
    )

    # ═══════════════════════════════════════════
    # Mission (TaskMaster, PC bridge, broadcast)
    # ═══════════════════════════════════════════
    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('origincar_bringup'),
                'launch', 'mission.launch.py')),
        launch_arguments={
            'start_task': start_task,
            'start_pc_bridge': start_pc_bridge,
            'start_broadcast': start_broadcast,
            'use_sim_time': use_sim_time,
            'cruise_speed': cruise_speed,
            'qr_scan_speed_gain': qr_scan_speed_gain,
            'turn_speed_gain': turn_speed_gain,
            'turn_angular_gain': turn_angular_gain,
            'capture_speed_gain': capture_speed_gain,
            'max_angular_z': max_angular_z,
            'cone_confidence_threshold': cone_confidence,
            'obstacle_slow_distance': obstacle_slow_distance,
            'obstacle_avoid_distance': obstacle_avoid_distance,
            'obstacle_backup_distance_threshold': obstacle_backup_distance_threshold,
            'obstacle_backup_speed_gain': obstacle_backup_speed_gain,
            'obstacle_backup_distance': obstacle_backup_distance,
            'obstacle_backup_timeout_sec': obstacle_backup_timeout_sec,
            'rear_clearance_distance': rear_clearance_distance,
            'control_frequency': control_frequency,
            'capture_radius': capture_radius,
            'scan_timeout_sec': scan_timeout_sec,
            'cone_detection_timeout_sec': cone_detection_timeout_sec,
            'obstacle_clear_hold_sec': obstacle_clear_hold_sec,
            'default_qr_direction': default_qr_direction,
            'lookahead_dist': lookahead_dist,
            'pass_radius': pass_radius,
            'reverse_pass_radius': reverse_pass_radius,
            'goal_tolerance': goal_tolerance,
            'enable_obstacle_avoidance': enable_obstacle_avoidance,
            'waypoints_file': waypoints_file,
            'pc_server_url': pc_server_url,
            'enable_callback_receiver': enable_callback_receiver,
        }.items(),
    )

    ld.add_action(base_launch)
    ld.add_action(lidar_launch)
    ld.add_action(perception_launch)
    ld.add_action(mission_launch)

    return ld
