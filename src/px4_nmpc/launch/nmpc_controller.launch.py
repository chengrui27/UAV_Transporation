#!/usr/bin/env python3
"""
启动PX4 NMPC控制器节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成启动描述"""

    # 声明launch参数
    quad_mass_arg = DeclareLaunchArgument(
        'quad_mass',
        default_value='1.535',
        description='Quadrotor mass in kg'
    )

    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='10.0',
        description='Control frequency in Hz'
    )

    offboard_delay_arg = DeclareLaunchArgument(
        'offboard_delay',
        default_value='2.0',
        description='Delay before enabling offboard mode in seconds'
    )

    # NMPC控制器节点
    nmpc_controller_node = Node(
        package='px4_nmpc',
        executable='nmpc_controller',
        name='nmpc_controller',
        output='screen',
        parameters=[{
            'quad_mass': LaunchConfiguration('quad_mass'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'offboard_delay': LaunchConfiguration('offboard_delay'),
        }],
        emulate_tty=True
    )

    return LaunchDescription([
        quad_mass_arg,
        control_frequency_arg,
        offboard_delay_arg,
        nmpc_controller_node
    ])
