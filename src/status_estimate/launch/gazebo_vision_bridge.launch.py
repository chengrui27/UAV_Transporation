from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pose_source',
            default_value='gz',
        ),
        DeclareLaunchArgument(
            'model_states_topic',
            default_value='/model_states',
        ),
        DeclareLaunchArgument(
            'gz_pose_topic',
            default_value='',
        ),
        DeclareLaunchArgument(
            'gz_world_name',
            default_value='',
        ),
        DeclareLaunchArgument(
            'mavros_ns',
            default_value='',
        ),
        DeclareLaunchArgument(
            'vehicle_mappings',
            default_value='iris_0:uav1,iris_1:uav2,iris_2:uav3',
        ),
        DeclareLaunchArgument(
            'publish_vision_pose',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'publish_odometry',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'world_frame_id',
            default_value='world',
        ),
        DeclareLaunchArgument(
            'child_frame_id',
            default_value='base_link',
        ),
        Node(
            package='status_estimate',
            executable='gazebo_vision_bridge',
            name='gazebo_vision_bridge',
            output='screen',
            parameters=[{
                'pose_source': LaunchConfiguration('pose_source'),
                'model_states_topic': LaunchConfiguration('model_states_topic'),
                'gz_pose_topic': LaunchConfiguration('gz_pose_topic'),
                'gz_world_name': LaunchConfiguration('gz_world_name'),
                'mavros_ns': LaunchConfiguration('mavros_ns'),
                'vehicle_mappings': LaunchConfiguration('vehicle_mappings'),
                'publish_vision_pose': ParameterValue(
                    LaunchConfiguration('publish_vision_pose'), value_type=bool
                ),
                'publish_odometry': ParameterValue(
                    LaunchConfiguration('publish_odometry'), value_type=bool
                ),
                'world_frame_id': LaunchConfiguration('world_frame_id'),
                'child_frame_id': LaunchConfiguration('child_frame_id'),
            }],
        ),
    ])
