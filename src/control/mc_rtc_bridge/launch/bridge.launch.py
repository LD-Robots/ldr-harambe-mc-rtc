from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('mc_rtc_bridge'), 'config', 'bridge_config.yaml'
    ])

    bridge_node = Node(
        package='mc_rtc_bridge',
        executable='mc_rtc_bridge',
        name='mc_rtc_bridge',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([bridge_node])
