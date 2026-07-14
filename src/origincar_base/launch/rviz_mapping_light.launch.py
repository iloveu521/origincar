import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = LaunchConfiguration('rviz_config')
    default_config = os.path.join(
        get_package_share_directory('origincar_base'),
        'rviz',
        'rviz_mapping_light.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=default_config),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_mapping_light',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
