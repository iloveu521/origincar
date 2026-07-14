#!/usr/bin/env python3
"""Launch the car-to-PC HTTP bridge node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('connect_to_pc')
    default_config = os.path.join(package_share, 'config', 'car_pc_bridge.yaml')

    config_file = LaunchConfiguration('config_file')
    pc_server_url = LaunchConfiguration('pc_server_url')
    image_topic = LaunchConfiguration('image_topic')
    image_msg_type = LaunchConfiguration('image_msg_type')
    ip_probe_host = LaunchConfiguration('ip_probe_host')
    enable_callback_receiver = LaunchConfiguration('enable_callback_receiver')
    callback_port = LaunchConfiguration('callback_port')
    stop_after_first_result = LaunchConfiguration('stop_after_first_result')
    trigger_required = LaunchConfiguration('trigger_required')
    trigger_topic = LaunchConfiguration('trigger_topic')
    car_ip = LaunchConfiguration('car_ip')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Car-PC bridge parameter file.'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/image',
            description='Camera topic to send to the PC service.'),
        DeclareLaunchArgument(
            'image_msg_type',
            default_value='compressed',
            description='Camera message type: compressed or raw.'),
        DeclareLaunchArgument(
            'pc_server_url',
            default_value='http://192.168.3.12:9999/predict',
            description='PC-side /predict HTTP endpoint.'),
        DeclareLaunchArgument(
            'ip_probe_host',
            default_value='192.168.3.12',
            description='LAN host used to detect the car outbound IP.'),
        DeclareLaunchArgument(
            'enable_callback_receiver',
            default_value='true',
            description='Enable car-side HTTP callback receiver.'),
        DeclareLaunchArgument(
            'callback_port',
            default_value='8888',
            description='Car-side callback receiver port.'),
        DeclareLaunchArgument(
            'stop_after_first_result',
            default_value='true',
            description='Stop sending and receiving after one valid result.'),
        DeclareLaunchArgument(
            'trigger_required',
            default_value='true',
            description='Only send a camera frame after trigger_topic is received.'),
        DeclareLaunchArgument(
            'trigger_topic',
            default_value='/capture_trigger',
            description='Topic used to trigger one image transfer.'),
        DeclareLaunchArgument(
            'car_ip',
            default_value='',
            description='Car LAN IP advertised to the PC callback.'),
        Node(
            package='connect_to_pc',
            executable='car_pc_bridge',
            name='car_pc_bridge',
            output='screen',
            parameters=[
                config_file,
                {
                    'pc_server_url': pc_server_url,
                    'image_topic': image_topic,
                    'image_msg_type': image_msg_type,
                    'ip_probe_host': ip_probe_host,
                    'enable_callback_receiver': ParameterValue(
                        enable_callback_receiver,
                        value_type=bool,
                    ),
                    'callback_port': ParameterValue(
                        callback_port,
                        value_type=int,
                    ),
                    'stop_after_first_result': ParameterValue(
                        stop_after_first_result,
                        value_type=bool,
                    ),
                    'trigger_required': ParameterValue(
                        trigger_required,
                        value_type=bool,
                    ),
                    'trigger_topic': trigger_topic,
                    'car_ip': car_ip,
                },
            ],
        ),
    ])
