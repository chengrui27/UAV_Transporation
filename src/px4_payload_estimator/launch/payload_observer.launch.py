from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_payload_estimator',
            executable='payload_observer',
            name='payload_observer',
            output='screen',
            parameters=[
                {'m_uav': 2.059},
                {'l_cable': 1.0},
                {'max_thrust': 40.6673}
            ]
        )
    ])
