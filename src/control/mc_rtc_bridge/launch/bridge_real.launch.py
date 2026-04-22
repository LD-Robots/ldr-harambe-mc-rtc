from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge_node = Node(
        package='mc_rtc_bridge',
        executable='mc_rtc_bridge_real',
        name='mc_rtc_bridge_real',
        parameters=[{
            'control_rate': 100.0,
            'joint_state_topic': '/joint_states',
            'imu_topic': '/pelvis/imu',
            'startup_duration': 3.0,
            'trajectory_dt': 0.05,
        }],
        output='screen',
    )

    return LaunchDescription([bridge_node])
