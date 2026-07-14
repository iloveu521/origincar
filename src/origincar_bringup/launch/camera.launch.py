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

    nv12_codec_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_decode.launch.py'),
                                        launch_arguments={'codec_in_mode': 'ros', 'codec_out_mode': 'shared_mem',
                                                                 'codec_sub_topic': '/image', 'codec_pub_topic': '/hbmem_img',
                                                                 'codec_in_format': 'jpeg', 'codec_out_format': 'nv12'}.items())

    # web_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('websocket') + '/launch/websocket.launch.py'),
    #                                     launch_arguments={'websocket_image_topic': '/image', 'websocket_image_type': 'mjpeg',
    #                                                       'websocket_smart_topic': LaunchConfiguration("dnn_example_msg_pub_topic_name")}.items())


    return LaunchDescription(launch_args + [
        usb_node,
        nv12_codec_node,
        # web_node
    ])
