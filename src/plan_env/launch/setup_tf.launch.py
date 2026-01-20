from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    一体化启动文件：TF树设置 + 点云转换

    功能：
    1. 发布 map -> base_link TF (从MAVROS odom)
    2. 发布 base_link -> lidar_link 静态TF
    3. 将点云从 lidar_link 转换到 map 坐标系

    输出：
    - /velodyne_points_map (PointCloud2, frame_id: map)
    """
    return LaunchDescription([
        # 参数定义
        DeclareLaunchArgument(
            'target_frame',
            default_value='map',
            description='目标坐标系'
        ),
        DeclareLaunchArgument(
            'input_cloud',
            default_value='/velodyne_points',
            description='输入点云话题'
        ),
        DeclareLaunchArgument(
            'output_cloud',
            default_value='/velodyne_points_map',
            description='输出点云话题'
        ),

        # 1. 从 MAVROS odom 发布 map -> base_link TF
        Node(
            package='plan_env',
            executable='odom_to_tf.py',
            name='odom_to_tf',
            output='screen',
            parameters=[{
                'odom_topic': '/mavros/local_position/odom',
                'publish_tf': True,
            }]
        ),

        # 2. 静态TF: base_link -> lidar_link
        # 雷达安装在机体上方8cm
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_tf',
            arguments=['0', '0', '0.08', '0', '0', '0', 'base_link', 'lidar_link'],
            output='screen'
        ),

        # 3. 点云坐标转换: lidar_link -> map
        Node(
            package='plan_env',
            executable='pointcloud_transformer.py',
            name='pointcloud_transformer',
            output='screen',
            parameters=[{
                'target_frame': LaunchConfiguration('target_frame'),
                'input_topic': LaunchConfiguration('input_cloud'),
                'output_topic': LaunchConfiguration('output_cloud'),
                'timeout': 0.1,
            }]
        ),
    ])
