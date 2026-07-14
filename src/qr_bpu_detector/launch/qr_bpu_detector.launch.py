import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("qr_bpu_detector")
    default_model = os.path.join(
        pkg_share, "config", "qr_detect_best_bayese_640x640_nv12.bin")
    default_config = os.path.join(pkg_share, "config", "qr_runtime.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=TextSubstitution(text=default_config)),
        DeclareLaunchArgument(
            "model_file",
            default_value=TextSubstitution(text=default_model)),
        DeclareLaunchArgument(
            "sub_img_topic",
            default_value=TextSubstitution(text="/aurora/rgb/image_raw")),
        DeclareLaunchArgument(
            "image_msg_type",
            default_value=TextSubstitution(text="raw")),
        DeclareLaunchArgument(
            "detection_topic",
            default_value=TextSubstitution(text="/qr_detection")),
        DeclareLaunchArgument(
            "debug_image_topic",
            default_value=TextSubstitution(text="/qr_detection/image/compressed")),
        DeclareLaunchArgument(
            "score_threshold",
            default_value=TextSubstitution(text="0.15")),
        DeclareLaunchArgument(
            "nms_threshold",
            default_value=TextSubstitution(text="0.50")),
        DeclareLaunchArgument(
            "publish_debug_image",
            default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument(
            "log_fps",
            default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument(
            "fps_log_interval_sec",
            default_value=TextSubstitution(text="1.0")),
        Node(
            package="qr_bpu_detector",
            executable="qr_bpu_detector_node",
            name="qr_bpu_detector",
            output="screen",
            parameters=[LaunchConfiguration("config_file"), {
                "model_file": LaunchConfiguration("model_file"),
                "sub_img_topic": LaunchConfiguration("sub_img_topic"),
                "image_msg_type": LaunchConfiguration("image_msg_type"),
                "detection_topic": LaunchConfiguration("detection_topic"),
                "debug_image_topic": LaunchConfiguration("debug_image_topic"),
                "score_threshold": LaunchConfiguration("score_threshold"),
                "nms_threshold": LaunchConfiguration("nms_threshold"),
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
                "log_fps": LaunchConfiguration("log_fps"),
                "fps_log_interval_sec": LaunchConfiguration("fps_log_interval_sec"),
                "class_names": ["qr_code"],
            }],
        ),
    ])
