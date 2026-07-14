import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 这个 launch 只做一件事：启动 Nav2（算法层），并把地图与参数文件传给 nav2_bringup。

    # 是否使用仿真时间（真机通常为 false）
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 支持传入命名空间（可选；单车可留空）
    namespace = LaunchConfiguration('namespace')

    # 自动启动生命周期节点（通常 true）
    autostart = LaunchConfiguration('autostart')

    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # ====== 默认地图与参数文件 ======
    # 注意：这里默认复用当前工程 wheeltec_nav2 的 map/param 目录
    # 如果你做了自己的 bringup 包（例如 rdkx5_nav2_bringup），把下面 package 名改成你的包名即可。
    this_pkg_share = get_package_share_directory('origincar_base')

    default_map = os.path.join(this_pkg_share, 'map', 'map.yaml')

    # 阿克曼小车可先用 param_mini_akm.yaml 起步，后续再按你的车体尺寸/速度/转弯半径调参。
    default_params = os.path.join(this_pkg_share, 'param', 'param_mini_akm.yaml')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # Nav2 官方 bringup 的入口
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace for the navigation stack'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes (在节点发生崩溃意外关闭时是否尝试自动重启该节点)"),
        
        # 直接包含 nav2_bringup 的 bringup_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': 'false',
                'slam': 'False',
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                'use_composition': 'False',
                'use_respawn': 'False',
                'log_level': 'info',
            }.items(),
        ),
    ])
