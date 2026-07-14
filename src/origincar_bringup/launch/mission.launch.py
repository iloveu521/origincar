#!/usr/bin/env python3
"""Launch the competition mission subsystem with one parameter contract."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _bool(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    bringup_share = get_package_share_directory('origincar_bringup')
    task_share = get_package_share_directory('origincar_task')
    params_file = os.path.join(bringup_share, 'config', 'competition.yaml')
    waypoints_file = os.path.join(
        task_share, 'config', 'waypoints_flowpath_custom_rpp.yaml')

    declarations = [
        DeclareLaunchArgument('start_task', default_value='true'),
        DeclareLaunchArgument('start_pc_bridge', default_value='true'),
        DeclareLaunchArgument('start_broadcast', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('task_params_file', default_value=params_file),
        DeclareLaunchArgument('waypoints_file', default_value=waypoints_file),
        DeclareLaunchArgument('cruise_speed', default_value='0.85'),
        DeclareLaunchArgument('qr_scan_speed_gain', default_value='0.85'),
        DeclareLaunchArgument('turn_speed_gain', default_value='0.50'),
        DeclareLaunchArgument('turn_angular_gain', default_value='1.50'),
        DeclareLaunchArgument('capture_speed_gain', default_value='0.60'),
        DeclareLaunchArgument('max_angular_z', default_value='1.80'),
        DeclareLaunchArgument('cone_confidence_threshold', default_value='0.35'),
        DeclareLaunchArgument('obstacle_slow_distance', default_value='0.45'),
        DeclareLaunchArgument('obstacle_avoid_distance', default_value='0.45'),
        DeclareLaunchArgument('obstacle_backup_distance_threshold', default_value='0.20'),
        DeclareLaunchArgument('obstacle_backup_speed_gain', default_value='0.70'),
        DeclareLaunchArgument('obstacle_backup_distance', default_value='0.40'),
        DeclareLaunchArgument('obstacle_backup_timeout_sec', default_value='1.50'),
        DeclareLaunchArgument('rear_clearance_distance', default_value='0.25'),
        DeclareLaunchArgument('control_frequency', default_value='20.0'),
        DeclareLaunchArgument('capture_radius', default_value='0.25'),
        DeclareLaunchArgument('scan_timeout_sec', default_value='0.30'),
        DeclareLaunchArgument('cone_detection_timeout_sec', default_value='0.25'),
        DeclareLaunchArgument('obstacle_clear_hold_sec', default_value='0.40'),
        DeclareLaunchArgument('default_qr_direction', default_value='cw'),
        DeclareLaunchArgument('lookahead_dist', default_value='0.22'),
        DeclareLaunchArgument('pass_radius', default_value='0.35'),
        DeclareLaunchArgument('reverse_pass_radius', default_value='0.30'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.35'),
        DeclareLaunchArgument('enable_obstacle_avoidance', default_value='true'),
        DeclareLaunchArgument(
            'pc_server_url', default_value='http://192.168.3.12:9999/predict'),
        DeclareLaunchArgument('ip_probe_host', default_value='192.168.3.12'),
        DeclareLaunchArgument('enable_callback_receiver', default_value='false'),
    ]

    task_master = Node(
        package='origincar_task',
        executable='task_master_node',
        name='task_master',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_task')),
        parameters=[
            LaunchConfiguration('task_params_file'),
            {
                'use_sim_time': _bool('use_sim_time'),
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'cruise_speed': _float('cruise_speed'),
                'qr_scan_speed_gain': _float('qr_scan_speed_gain'),
                'turn_speed_gain': _float('turn_speed_gain'),
                'turn_angular_gain': _float('turn_angular_gain'),
                'capture_speed_gain': _float('capture_speed_gain'),
                'max_angular_z': _float('max_angular_z'),
                'cone_confidence_threshold': _float('cone_confidence_threshold'),
                'obstacle_slow_distance': _float('obstacle_slow_distance'),
                'obstacle_avoid_distance': _float('obstacle_avoid_distance'),
                'obstacle_backup_distance_threshold': _float(
                    'obstacle_backup_distance_threshold'),
                'obstacle_backup_speed_gain': _float(
                    'obstacle_backup_speed_gain'),
                'obstacle_backup_distance': _float('obstacle_backup_distance'),
                'obstacle_backup_timeout_sec': _float(
                    'obstacle_backup_timeout_sec'),
                'rear_clearance_distance': _float('rear_clearance_distance'),
                'control_frequency': _float('control_frequency'),
                'capture_radius': _float('capture_radius'),
                'scan_timeout_sec': _float('scan_timeout_sec'),
                'cone_detection_timeout_sec': _float(
                    'cone_detection_timeout_sec'),
                'obstacle_clear_hold_sec': _float('obstacle_clear_hold_sec'),
                'default_qr_direction': LaunchConfiguration(
                    'default_qr_direction'),
                'lookahead_dist': _float('lookahead_dist'),
                'pass_radius': _float('pass_radius'),
                'reverse_pass_radius': _float('reverse_pass_radius'),
                'goal_tolerance': _float('goal_tolerance'),
                'enable_obstacle_avoidance': _bool(
                    'enable_obstacle_avoidance'),
            },
        ],
    )

    pc_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('connect_to_pc'),
            'launch', 'car_pc_bridge.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_pc_bridge')),
        launch_arguments={
            'pc_server_url': LaunchConfiguration('pc_server_url'),
            'ip_probe_host': LaunchConfiguration('ip_probe_host'),
            'enable_callback_receiver': LaunchConfiguration(
                'enable_callback_receiver'),
        }.items(),
    )

    broadcast = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('origincar_broadcast'),
            'launch', 'broadcast.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_broadcast')),
    )

    return LaunchDescription(declarations + [task_master, pc_bridge, broadcast])
