# status_estimate

Bridge Gazebo model states to MAVROS vision/odometry inputs for PX4 SITL.

## Build

colcon build --packages-select status_estimate
source install/setup.bash

## Run

ros2 launch status_estimate gazebo_vision_bridge.launch.py

## Parameters

- pose_source: `gz` (Gazebo transport) or `ros` (ROS ModelStates). Default: `gz`
- gz_pose_topic: Gazebo transport pose topic (auto-detect if empty)
- gz_world_name: World name to build `/gazebo/<world>/pose/info` if topic is not provided
- mavros_ns: Extra namespace inserted before the MAVROS plugin topics (default empty). Use `mavros` if your topics are `/<ns>/mavros/vision_pose/pose`.
- model_states_topic: ROS ModelStates topic when `pose_source=ros` (default: /model_states)
- vehicle_mappings: Comma list of model:namespace (default: iris_0:uav1,iris_1:uav2,iris_2:uav3)
- publish_vision_pose: Publish to /<ns>/mavros/vision_pose/pose (default: true)
- publish_odometry: Publish to /<ns>/mavros/odometry/in (default: false)
- world_frame_id: Frame id for outputs (default: world)
- child_frame_id: Odometry child frame id template (default: base_link, supports {ns})

PX4 must be configured to accept vision/odometry inputs (EKF2_AID_MASK, etc.).
If using `pose_source=gz`, verify the Gazebo transport topic with
`gz topic -l | grep pose/info` and pass `gz_pose_topic` if auto-detect fails.
