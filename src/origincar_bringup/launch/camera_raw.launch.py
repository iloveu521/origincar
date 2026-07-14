import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory, get_package_prefix

def generate_launch_description():

    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument('device', default_value='/dev/video0', description='usb/mipi camera device'),
        DeclareLaunchArgument('width', default_value='640', description='camera image width'),
        DeclareLaunchArgument('height', default_value='480', description='camera image height'),
    ]

    # Include launch descriptions
    usb_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_usb_cam') + '/launch/hobot_usb_cam.launch.py'),
                                       launch_arguments={'usb_image_width': LaunchConfiguration('width'),
                                                         'usb_image_height': LaunchConfiguration('height'),
                                                         'usb_video_device': LaunchConfiguration('device')}.items())

    return LaunchDescription(launch_args + [
        usb_node,
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            output='screen',
            parameters=[
                {"channel": 1},
                {"in_mode": "ros"},
                {"in_format": "jpeg"},
                {"out_mode": "ros"},
                {"out_format": "bgr8"},
                {"sub_topic": "/image"},
                {"pub_topic": "/image_raw"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
    ])
