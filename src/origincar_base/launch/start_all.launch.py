import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 一键启动：底盘/TF/EKF（origincar_bringup） + Nav2 算法层（nav2_bringup bringup_launch）

    use_sim_time = LaunchConfiguration('use_sim_time')
    akmcar = LaunchConfiguration('akmcar')

    # 地图与 Nav2 参数文件（建议在真机上用绝对路径覆盖）
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart')

    
    this_pkg_share = get_package_share_directory('origincar_base')
    default_map = os.path.join(this_pkg_share, 'map', 'map.yaml')
    default_params = os.path.join(this_pkg_share, 'param', 'param_mini_akm.yaml')

    # 车体 bringup（底盘、静态TF、EKF）— 使用 slim_bringup 避免 OOM
    # 说明：base_footprint→base_link 和 base_link→laser 均由静态TF发布，不需要 URDF。
    # robot_state_publisher(URDF) 和 imu_filter_madgwick 各占 ~342MB，对 Nav2 无用，不启动。
    origincar_base_share = get_package_share_directory('origincar_base')
    origincar_bringup_launch = os.path.join(origincar_base_share, 'launch', 'slim_bringup.launch.py')

    # Nav2 bringup（官方）
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch = os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('akmcar', default_value='true', description='Ackermann car mode'),
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace for Nav2'),
        DeclareLaunchArgument('autostart', default_value='true', description='Autostart Nav2 lifecycle nodes'),
        DeclareLaunchArgument('map', default_value=default_map, description='Full path to map yaml file'),
        DeclareLaunchArgument('params_file', default_value=default_params, description='Full path to Nav2 params yaml'),

        # 1) 启动底盘/TF/EKF（slim，无 URDF/imu_filter）
        # EKF 内部已延迟 4 秒启动，避免与 origincar_base 的 DDS 峰值叠加
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(origincar_bringup_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        # 2) Nav2 延迟 10 秒启动，等待 EKF 稳定（EKF 在第 4 秒启动，需约 2 秒稳定）
        TimerAction(period=10.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_launch),
                launch_arguments={
                    'namespace': namespace,
                    'use_namespace': 'false',
                    'slam': 'false',
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': autostart,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                    'log_level': 'info',
                }.items(),
            ),
        ]),
    ])
