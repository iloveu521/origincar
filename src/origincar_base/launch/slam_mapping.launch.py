import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    start_base = LaunchConfiguration('start_base')

    origincar_base_share = get_package_share_directory('origincar_base')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    slim_bringup_launch = os.path.join(
        origincar_base_share, 'launch', 'slim_bringup.launch.py')
    online_async_launch = os.path.join(
        slam_toolbox_share, 'launch', 'online_async_launch.py')
    default_slam_params = os.path.join(
        origincar_base_share, 'config', 'slam_mapping.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='slam_toolbox mapping parameter file'),
        DeclareLaunchArgument(
            'start_base',
            default_value='true',
            description='Start slim_bringup before slam_toolbox. Set false if base/EKF/TF are already running.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slim_bringup_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(start_base),
        ),

        # Wait for base, static TF and EKF (EKF starts at t=4s) before SLAM subscribes.
        TimerAction(period=8.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(online_async_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': slam_params_file,
                }.items(),
            ),
        ]),
    ])
