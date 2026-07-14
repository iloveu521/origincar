#!/usr/bin/env python3
"""Terminal 2: vehicle, localization, perception and speech stack.

Starts chassis, static TF, EKF, Nav2 localization, LiDAR, Aurora930 camera,
QR detector/decoder, the unmodified cone detector, and speech broadcasting.
It intentionally does not start TaskMaster or connect_to_pc.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    laser_x = LaunchConfiguration('laser_x')
    laser_yaw = LaunchConfiguration('laser_yaw')

    base_share = get_package_share_directory('origincar_base')
    bringup_share = get_package_share_directory('origincar_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')
    lidar_share = get_package_share_directory('lslidar_driver')
    broadcast_share = get_package_share_directory('origincar_broadcast')

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'base.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'laser_x': laser_x,
            'laser_yaw': laser_yaw,
        }.items(),
    )

    localization = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'localization_launch.py')),
            launch_arguments={
                'namespace': '',
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true',
                'use_composition': 'False',
                'use_respawn': 'False',
                'log_level': 'info',
            }.items(),
        )],
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_share, 'launch', 'lsn10_launch.py')),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'perception.launch.py')),
    )

    broadcast = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(broadcast_share, 'launch', 'broadcast.launch.py')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(base_share, 'map', 'race_modify.yaml'),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(base_share, 'param', 'param_mini_akm.yaml'),
        ),
        DeclareLaunchArgument('laser_x', default_value='-0.10'),
        DeclareLaunchArgument('laser_yaw', default_value='0.05'),
        base,
        localization,
        lidar,
        perception,
        broadcast,
    ])
