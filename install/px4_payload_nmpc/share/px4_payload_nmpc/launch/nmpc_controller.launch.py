#!/usr/bin/env python3
"""
启动NMPC控制器节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成启动描述"""

    # 声明launch参数
    cable_length_arg = DeclareLaunchArgument(
        'cable_length',
        default_value='1.0',
        description='Cable length in meters'
    )

    quad_mass_arg = DeclareLaunchArgument(
        'quad_mass',
        default_value='2.095',
        description='Quadrotor mass in kg'
    )

    payload_mass_arg = DeclareLaunchArgument(
        'payload_mass',
        default_value='0.5',
        description='Payload mass in kg'
    )

    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='100.0',
        description='Control frequency in Hz'
    )

    offboard_delay_arg = DeclareLaunchArgument(
        'offboard_delay',
        default_value='2.0',
        description='Delay before enabling offboard mode in seconds'
    )

    # NMPC控制器节点
    nmpc_controller_node = Node(
        package='px4_payload_nmpc',
        executable='nmpc_controller',
        name='nmpc_controller',
        output='screen',
        parameters=[{
            'cable_length': LaunchConfiguration('cable_length'),
            'quad_mass': LaunchConfiguration('quad_mass'),
            'payload_mass': LaunchConfiguration('payload_mass'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'offboard_delay': LaunchConfiguration('offboard_delay'),
        }],
        emulate_tty=True
    )

    return LaunchDescription([
        cable_length_arg,
        quad_mass_arg,
        payload_mass_arg,
        control_frequency_arg,
        offboard_delay_arg,
        nmpc_controller_node
    ])
