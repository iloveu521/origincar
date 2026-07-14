from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    static_tf = launch_ros.actions.Node(
        package='origincar_base',
        executable='origincar_static_tf_node',
        name='origincar_static_tf_node',
        output='screen',
    )

    origincar_base = launch_ros.actions.Node(
        package='origincar_base',
        executable='origincar_base_node',
        name='origincar_base_node',
        output='screen',
        parameters=[{'usart_port_name': '/dev/ttyACM0', 'use_sim_time': use_sim_time}],
    )

    odom_tf = launch_ros.actions.Node(
        package='origincar_base',
        executable='odom_tf_node',
        name='odom_tf_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        static_tf,
        origincar_base,
        odom_tf,
    ])
