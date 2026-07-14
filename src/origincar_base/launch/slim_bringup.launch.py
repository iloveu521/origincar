import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Minimal launch for EKF verification — no URDF, saves ~400MB memory."""

    bringup_dir = get_package_share_directory('origincar_base')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_sim_time_dec = DeclareLaunchArgument('use_sim_time', default_value='false')
    akmcar = LaunchConfiguration('akmcar', default='true')
    akmcar_dec = DeclareLaunchArgument(
        'akmcar',
        default_value='true',
        description='Start Ackermann cmd_vel converter with the base serial driver.',
    )
    laser_x = LaunchConfiguration('laser_x', default='0.083')
    laser_y = LaunchConfiguration('laser_y', default='0.0')
    laser_z = LaunchConfiguration('laser_z', default='0.102')
    laser_roll = LaunchConfiguration('laser_roll', default='0.0')
    laser_pitch = LaunchConfiguration('laser_pitch', default='0.0')
    laser_yaw = LaunchConfiguration('laser_yaw', default='0.05')
    laser_x_dec = DeclareLaunchArgument('laser_x', default_value='0.083')
    laser_y_dec = DeclareLaunchArgument('laser_y', default_value='0.0')
    laser_z_dec = DeclareLaunchArgument('laser_z', default_value='0.102')
    laser_roll_dec = DeclareLaunchArgument('laser_roll', default_value='0.0')
    laser_pitch_dec = DeclareLaunchArgument('laser_pitch', default_value='0.0')
    laser_yaw_dec = DeclareLaunchArgument(
        'laser_yaw',
        default_value='0.0',
        description='Yaw offset from base_link to laser in radians for LiDAR extrinsic calibration.',
    )

    # ====== Static TF (required for EKF) ======
    # Use one process for all static transforms to reduce DDS participants and memory.
    static_tf = launch_ros.actions.Node(
        package='origincar_base',
        executable='origincar_static_tf_node',
        name='origincar_static_tf_node',
        output='screen',
        parameters=[{
            'laser_x': laser_x,
            'laser_y': laser_y,
            'laser_z': laser_z,
            'laser_roll': laser_roll,
            'laser_pitch': laser_pitch,
            'laser_yaw': laser_yaw,
        }],
    )

    # ====== Origincar Base (serial driver + optional Ackermann converter) ======
    origincar_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'base_serial.launch.py')),
        launch_arguments={'akmcar': akmcar}.items(),
    )

    # ====== EKF (fuse /odom + /imu/data_raw → /odom_combined) ======
    # NOTE: ekf.yaml uses imu0: /imu/data_raw directly, no imu_filter_madgwick needed
    ekf_config = Path(bringup_dir, 'config', 'ekf.yaml')
    # 不指定 name，使用默认节点名 ekf_filter_node，与 ekf.yaml 的命名空间一致
    robot_ekf = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odom_combined')],
    )

    # EKF 延迟 4 秒启动，错开 DDS 初始化内存峰值
    # origincar_base 启动时 DDS 峰值约 880MB，稳定后降到 23MB
    # 4 秒后再启动 EKF，两个峰值不叠加，避免 OOM
    ekf_delayed = TimerAction(period=4.0, actions=[robot_ekf])

    ld = LaunchDescription()
    ld.add_action(use_sim_time_dec)
    ld.add_action(akmcar_dec)
    ld.add_action(laser_x_dec)
    ld.add_action(laser_y_dec)
    ld.add_action(laser_z_dec)
    ld.add_action(laser_roll_dec)
    ld.add_action(laser_pitch_dec)
    ld.add_action(laser_yaw_dec)
    ld.add_action(static_tf)
    ld.add_action(origincar_base)
    ld.add_action(ekf_delayed)

    return ld
