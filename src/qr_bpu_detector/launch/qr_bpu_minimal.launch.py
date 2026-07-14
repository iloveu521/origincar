#!/usr/bin/env python3
"""Competition launch: BPU QR detection plus exact-frame ROI decoding."""

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

    config_file = LaunchConfiguration("config_file")
    image_topic = LaunchConfiguration("image_topic")
    image_msg_type = LaunchConfiguration("image_msg_type")
    score_threshold = LaunchConfiguration("score_threshold")
    detection_topic = "/qr_detection"

    detector_node = Node(
        package="qr_bpu_detector",
        executable="qr_bpu_detector_node",
        name="qr_bpu_detector",
        output="screen",
        parameters=[config_file, {
            "model_file": default_model,
            "sub_img_topic": image_topic,
            "image_msg_type": image_msg_type,
            "detection_topic": detection_topic,
            "score_threshold": score_threshold,
            "nms_threshold": 0.50,
            "publish_debug_image": False,
            "log_fps": True,
            "fps_log_interval_sec": 1.0,
            "class_names": ["qr_code"],
        }],
    )

    decoder_node = Node(
        package="qr_bpu_detector",
        executable="qr_roi_decoder_node",
        name="qr_roi_decoder",
        output="screen",
        parameters=[config_file, {
            "image_topic": image_topic,
            "image_msg_type": image_msg_type,
            "detection_topic": detection_topic,
            "direction_topic": "/qr_direction",
            "raw_number_topic": "/qr_number",
            "state_topic": "/qr_decode_state",
            "target_type": "qr_code",
            "min_confidence": score_threshold,
            "dynamic_padding_ratio": LaunchConfiguration(
                "dynamic_padding_ratio"),
            "target_qr_pixels": LaunchConfiguration("target_qr_pixels"),
            "max_upscale": LaunchConfiguration("max_upscale"),
            "fallback_frame_threshold": LaunchConfiguration(
                "fallback_frame_threshold"),
            "fallback_interval_frames": LaunchConfiguration(
                "fallback_interval_frames"),
            "fallback_center_ratio": LaunchConfiguration(
                "fallback_center_ratio"),
            "fallback_upscale": LaunchConfiguration("fallback_upscale"),
            "fallback_max_pixels": LaunchConfiguration(
                "fallback_max_pixels"),
            "max_roi_candidates": LaunchConfiguration("max_roi_candidates"),
            "sync_queue_size": LaunchConfiguration("sync_queue_size"),
            "scan_full_image_on_roi_failure": LaunchConfiguration(
                "scan_full_image_on_roi_failure"),
            "publish_state": False,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=TextSubstitution(text=default_config),
            description="Runtime parameter YAML."),
        DeclareLaunchArgument(
            "image_topic",
            default_value=TextSubstitution(text="/image_raw"),
            description="Input camera image topic."),
        DeclareLaunchArgument(
            "image_msg_type",
            default_value=TextSubstitution(text="raw"),
            description="Input image type: raw or compressed."),
        DeclareLaunchArgument(
            "score_threshold",
            default_value=TextSubstitution(text="0.15"),
            description="QR detector and decoder confidence threshold."),
        DeclareLaunchArgument(
            "dynamic_padding_ratio",
            default_value=TextSubstitution(text="0.15"),
            description="Padding on each side as a ratio of detector box size."),
        DeclareLaunchArgument(
            "target_qr_pixels",
            default_value=TextSubstitution(text="280.0"),
            description="Target QR box size used to calculate ROI upscaling."),
        DeclareLaunchArgument(
            "max_upscale",
            default_value=TextSubstitution(text="8.0"),
            description="Maximum ROI enlargement factor."),
        DeclareLaunchArgument(
            "fallback_frame_threshold",
            default_value=TextSubstitution(text="10"),
            description="Failures before the first center/full-image fallback."),
        DeclareLaunchArgument(
            "fallback_interval_frames",
            default_value=TextSubstitution(text="10"),
            description="Frames between subsequent fallback scans."),
        DeclareLaunchArgument(
            "fallback_center_ratio",
            default_value=TextSubstitution(text="0.75"),
            description="Centered fallback ROI width/height ratio."),
        DeclareLaunchArgument(
            "fallback_upscale",
            default_value=TextSubstitution(text="1.5"),
            description="Fallback image enlargement factor."),
        DeclareLaunchArgument(
            "fallback_max_pixels",
            default_value=TextSubstitution(text="1000000"),
            description="Maximum enlarged fallback image area."),
        DeclareLaunchArgument(
            "max_roi_candidates",
            default_value=TextSubstitution(text="3")),
        DeclareLaunchArgument(
            "sync_queue_size",
            default_value=TextSubstitution(text="10"),
            description="ExactTime image/detection synchronization queue size."),
        DeclareLaunchArgument(
            "scan_full_image_on_roi_failure",
            default_value=TextSubstitution(text="true"),
            description="Enable low-frequency center and full-image fallback."),
        detector_node,
        decoder_node,
    ])
