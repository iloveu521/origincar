import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def _launch_nodes(context, *args, **kwargs):
    del args, kwargs

    legacy_sub_img_topic = LaunchConfiguration("sub_img_topic").perform(context).strip()
    image_topic = legacy_sub_img_topic or LaunchConfiguration("image_topic").perform(context)
    min_confidence = LaunchConfiguration("min_confidence").perform(context).strip()
    decoder_min_confidence = float(
        min_confidence or LaunchConfiguration("score_threshold").perform(context)
    )
    config_file = LaunchConfiguration("config_file")

    detector_node = Node(
        package="qr_bpu_detector",
        executable="qr_bpu_detector_node",
        name="qr_bpu_detector",
        output="screen",
        parameters=[config_file, {
            "model_file": LaunchConfiguration("model_file"),
            "sub_img_topic": image_topic,
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
    )

    decoder_node = Node(
        package="qr_bpu_detector",
        executable="qr_roi_decoder_node",
        name="qr_roi_decoder",
        output="screen",
        parameters=[config_file, {
            "image_topic": image_topic,
            "image_msg_type": LaunchConfiguration("image_msg_type"),
            "detection_topic": LaunchConfiguration("detection_topic"),
            "direction_topic": LaunchConfiguration("direction_topic"),
            "raw_number_topic": LaunchConfiguration("raw_number_topic"),
            "state_topic": LaunchConfiguration("state_topic"),
            "target_type": LaunchConfiguration("target_type"),
            "min_confidence": decoder_min_confidence,
            "dynamic_padding_ratio": LaunchConfiguration("dynamic_padding_ratio"),
            "target_qr_pixels": LaunchConfiguration("target_qr_pixels"),
            "max_upscale": LaunchConfiguration("max_upscale"),
            "fallback_frame_threshold": LaunchConfiguration(
                "fallback_frame_threshold"),
            "fallback_interval_frames": LaunchConfiguration(
                "fallback_interval_frames"),
            "fallback_center_ratio": LaunchConfiguration("fallback_center_ratio"),
            "fallback_upscale": LaunchConfiguration("fallback_upscale"),
            "fallback_max_pixels": LaunchConfiguration("fallback_max_pixels"),
            "max_roi_candidates": LaunchConfiguration("max_roi_candidates"),
            "sync_queue_size": LaunchConfiguration("sync_queue_size"),
            "scan_full_image_on_roi_failure": LaunchConfiguration(
                "scan_full_image_on_roi_failure"),
            "publish_state": LaunchConfiguration("publish_state"),
        }],
    )

    return [
        RegisterEventHandler(
            OnProcessStart(
                target_action=detector_node,
                on_start=[decoder_node],
            )
        ),
        detector_node,
    ]


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
            "image_topic",
            default_value=TextSubstitution(text="/aurora/rgb/image_raw")),
        DeclareLaunchArgument(
            "sub_img_topic",
            default_value=TextSubstitution(text="")),
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
        DeclareLaunchArgument(
            "direction_topic",
            default_value=TextSubstitution(text="/qr_direction")),
        DeclareLaunchArgument(
            "raw_number_topic",
            default_value=TextSubstitution(text="/qr_number")),
        DeclareLaunchArgument(
            "state_topic",
            default_value=TextSubstitution(text="/qr_decode_state")),
        DeclareLaunchArgument(
            "target_type",
            default_value=TextSubstitution(text="qr_code")),
        DeclareLaunchArgument(
            "min_confidence",
            default_value=TextSubstitution(text="")),
        DeclareLaunchArgument(
            "dynamic_padding_ratio",
            default_value=TextSubstitution(text="0.15")),
        DeclareLaunchArgument(
            "target_qr_pixels",
            default_value=TextSubstitution(text="280.0")),
        DeclareLaunchArgument(
            "max_upscale",
            default_value=TextSubstitution(text="8.0")),
        DeclareLaunchArgument(
            "fallback_frame_threshold",
            default_value=TextSubstitution(text="10")),
        DeclareLaunchArgument(
            "fallback_interval_frames",
            default_value=TextSubstitution(text="10")),
        DeclareLaunchArgument(
            "fallback_center_ratio",
            default_value=TextSubstitution(text="0.75")),
        DeclareLaunchArgument(
            "fallback_upscale",
            default_value=TextSubstitution(text="1.5")),
        DeclareLaunchArgument(
            "fallback_max_pixels",
            default_value=TextSubstitution(text="1000000")),
        DeclareLaunchArgument(
            "max_roi_candidates",
            default_value=TextSubstitution(text="3")),
        DeclareLaunchArgument(
            "sync_queue_size",
            default_value=TextSubstitution(text="10")),
        DeclareLaunchArgument(
            "scan_full_image_on_roi_failure",
            default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument(
            "publish_state",
            default_value=TextSubstitution(text="true")),
        OpaqueFunction(function=_launch_nodes),
    ])
