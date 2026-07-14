import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_base = LaunchConfiguration('start_base')
    slam_params_file = LaunchConfiguration('slam_params_file')

    origincar_base_share = get_package_share_directory('origincar_base')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    base_launch = os.path.join(origincar_base_share, 'launch', 'base_no_ekf.launch.py')
    slam_launch = os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
    default_slam_params = os.path.join(
        origincar_base_share, 'config', 'slam_mapping_no_ekf.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam_params),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(start_base),
        ),

        TimerAction(period=6.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': slam_params_file,
                }.items(),
            ),
        ]),
    ])
