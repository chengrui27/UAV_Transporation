# PX4 Offboard Control with MAVROS

This ROS2 package demonstrates how to control a PX4 drone in offboard mode using MAVROS.

## Requirements

- ROS2 Humble
- PX4 v1.13.3
- MAVROS for ROS2 Humble

## Build Instructions

```bash
cd ~/px4_ws
colcon build --packages-select px4_offboard_control
source install/setup.bash
```

## Usage

### 1. Start PX4 SITL Simulation

In terminal 1:
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

### 2. Launch MAVROS and Offboard Control

In terminal 2:
```bash
cd ~/px4_ws
source install/setup.bash
ros2 launch px4_offboard_control offboard_control.launch.py
```

## What This Does

The offboard control node will:

1. Connect to PX4 via MAVROS
2. Send setpoint commands at 100 Hz (required for offboard mode)
3. Automatically switch to OFFBOARD mode
4. Arm the vehicle
5. Command the drone to hover at position (0, 0, 2) - 2 meters altitude

## Modifying Target Position

You can modify the target position in the source code:

Edit [src/offboard_control.cpp:51-53](src/offboard_control.cpp#L51-L53):
```cpp
pose_.pose.position.x = 0.0;  // X position (meters)
pose_.pose.position.y = 0.0;  // Y position (meters)
pose_.pose.position.z = 2.0;  // Z position (meters, altitude)
```

Then rebuild:
```bash
colcon build --packages-select px4_offboard_control
```

## Troubleshooting

### MAVROS not connecting to PX4

Check that PX4 SITL is running and the UDP ports are correct:
- PX4 SITL uses UDP port 14540 for MAVLink
- MAVROS connects to `udp://:14540@127.0.0.1:14557`

### Offboard mode rejected

Make sure:
- Setpoints are being published at >2 Hz (this node publishes at 100 Hz)
- The drone is receiving position setpoints before switching to offboard mode
- No RC failsafe is active (disable with `param set COM_RCL_EXCEPT 4` in PX4)

## Topics

- `/mavros/state` - Current vehicle state
- `/mavros/setpoint_position/local` - Position setpoints
- `/mavros/local_position/pose` - Current position

## Services

- `/mavros/cmd/arming` - Arm/disarm vehicle
- `/mavros/set_mode` - Change flight mode
