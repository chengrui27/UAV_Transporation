from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch multi-UAV offboard takeoff controller."""

    multi_offboard_node = Node(
        package='multi_payload',
        executable='multi_offboard',
        name='multi_offboard',
        output='screen'
    )

    return LaunchDescription([
        multi_offboard_node
    ])

