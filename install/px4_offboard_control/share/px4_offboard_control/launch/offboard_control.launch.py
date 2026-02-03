from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
import os


def generate_launch_description():
    """Launch file for PX4 offboard control with MAVROS"""

    # Start MAVROS using ExecuteProcess instead of Node
    # This more closely mimics running 'ros2 run' in terminal
    mavros_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mavros', 'mavros_node',
            '--ros-args',
            '--param', 'fcu_url:=udp://:14540@localhost:14557'
        ],
        output='screen',
        shell=False
    )

    # Offboard control node - delayed start
    offboard_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='px4_offboard_control',
                executable='offboard_control',
                name='offboard_control',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        mavros_process,
        offboard_node
    ])
