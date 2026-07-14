#!/usr/bin/env python3
"""Launch the OriginCar broadcast manager."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('origincar_broadcast')
    default_config = os.path.join(package_share, 'config', 'broadcast.yaml')

    config_file = LaunchConfiguration('config_file')
    speech_enabled = LaunchConfiguration('speech_enabled')
    i2c_bus = LaunchConfiguration('i2c_bus')
    i2c_addr = LaunchConfiguration('i2c_addr')
    volume = LaunchConfiguration('volume')
    speed = LaunchConfiguration('speed')
    configure_on_startup = LaunchConfiguration('configure_on_startup')
    wait_after_speak = LaunchConfiguration('wait_after_speak')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Broadcast manager parameter file.'),
        DeclareLaunchArgument(
            'speech_enabled',
            default_value='true',
            description='Enable I2C speech output.'),
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='5',
            description='I2C bus id for the speech module.'),
        DeclareLaunchArgument(
            'i2c_addr',
            default_value='48',
            description='I2C address for the speech module.'),
        DeclareLaunchArgument(
            'volume',
            default_value='10',
            description='Speech module volume, 0-10.'),
        DeclareLaunchArgument(
            'speed',
            default_value='5',
            description='Speech module speed, 0-10.'),
        DeclareLaunchArgument(
            'configure_on_startup',
            default_value='false',
            description='Send speech module control frames during startup.'),
        DeclareLaunchArgument(
            'wait_after_speak',
            default_value='false',
            description='Poll speech chip status after each text frame.'),
        Node(
            package='origincar_broadcast',
            executable='broadcast_manager',
            name='broadcast_manager',
            output='screen',
            parameters=[
                config_file,
                {
                    'speech_enabled': speech_enabled,
                    'i2c_bus': i2c_bus,
                    'i2c_addr': i2c_addr,
                    'volume': volume,
                    'speed': speed,
                    'configure_on_startup': configure_on_startup,
                    'wait_after_speak': wait_after_speak,
                },
            ],
        ),
    ])
