#!/usr/bin/env python3
"""
Launch multi-payload NMPC node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='100.0',
        description='Control frequency in Hz'
    )

    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height',
        default_value='2.0',
        description='Takeoff height delta in meters'
    )

    takeoff_tolerance_arg = DeclareLaunchArgument(
        'takeoff_tolerance',
        default_value='0.2',
        description='Takeoff position tolerance in meters'
    )

    offboard_setpoint_threshold_arg = DeclareLaunchArgument(
        'offboard_setpoint_threshold',
        default_value='20',
        description='Number of setpoints before switching to OFFBOARD'
    )

    cable_length_arg = DeclareLaunchArgument(
        'cable_length',
        default_value='1.5',
        description='Cable length in meters'
    )

    quad_mass_arg = DeclareLaunchArgument(
        'quad_mass',
        default_value='1.535',
        description='Quadrotor mass in kg'
    )

    payload_mass_arg = DeclareLaunchArgument(
        'payload_mass',
        default_value='1.0',
        description='Payload mass in kg'
    )

    max_thrust_arg = DeclareLaunchArgument(
        'max_thrust',
        default_value='23.59',
        description='Maximum thrust in N for normalization'
    )

    omega_filter_alpha_arg = DeclareLaunchArgument(
        'omega_filter_alpha',
        default_value='1.0',
        description='Low-pass filter alpha for cable angular velocity'
    )

    # Edit these three vectors to change takeoff targets for uav1/uav2/uav3.
    takeoff_target_x = [0.7, -0.3, -0.3]
    takeoff_target_y = [0.0, 0.6, -0.6]
    takeoff_target_z = [2.0, 2.0, 2.0]

    nmpc_node = Node(
        package='multi_payload',
        executable='multi_payload_nmpc',
        name='multi_payload_nmpc',
        output='screen',
        parameters=[{
            'control_frequency': LaunchConfiguration('control_frequency'),
            'takeoff_height': LaunchConfiguration('takeoff_height'),
            'takeoff_tolerance': LaunchConfiguration('takeoff_tolerance'),
            'offboard_setpoint_threshold': LaunchConfiguration('offboard_setpoint_threshold'),
            'cable_length': LaunchConfiguration('cable_length'),
            'quad_mass': LaunchConfiguration('quad_mass'),
            'payload_mass': LaunchConfiguration('payload_mass'),
            'max_thrust': LaunchConfiguration('max_thrust'),
            'omega_filter_alpha': LaunchConfiguration('omega_filter_alpha'),
            'takeoff_target_x': takeoff_target_x,
            'takeoff_target_y': takeoff_target_y,
            'takeoff_target_z': takeoff_target_z,
        }],
        emulate_tty=True
    )

    return LaunchDescription([
        control_frequency_arg,
        takeoff_height_arg,
        takeoff_tolerance_arg,
        offboard_setpoint_threshold_arg,
        cable_length_arg,
        quad_mass_arg,
        payload_mass_arg,
        max_thrust_arg,
        omega_filter_alpha_arg,
        nmpc_node,
    ])
