from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'map_size_x',
            default_value='40.0',
            description='Map size in X direction (meters)'
        ),
        DeclareLaunchArgument(
            'map_size_y',
            default_value='40.0',
            description='Map size in Y direction (meters)'
        ),
        DeclareLaunchArgument(
            'map_size_z',
            default_value='5.0',
            description='Map size in Z direction (meters)'
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.1',
            description='Map resolution (meters/voxel)'
        ),

        # Grid Map Node
        Node(
            package='plan_env',
            executable='grid_map_node',
            name='grid_map_node',
            output='screen',
            parameters=[{
                # Map parameters
                'grid_map.resolution': LaunchConfiguration('resolution'),
                'grid_map.map_size_x': LaunchConfiguration('map_size_x'),
                'grid_map.map_size_y': LaunchConfiguration('map_size_y'),
                'grid_map.map_size_z': LaunchConfiguration('map_size_z'),
                'grid_map.local_update_range_x': 5.0,
                'grid_map.local_update_range_y': 5.0,
                'grid_map.local_update_range_z': 3.0,
                'grid_map.obstacles_inflation': 0.1,

                # Camera parameters (adjust based on your camera)
                'grid_map.fx': 387.229,
                'grid_map.fy': 387.229,
                'grid_map.cx': 321.04,
                'grid_map.cy': 243.56,

                # Depth filter
                'grid_map.use_depth_filter': True,
                'grid_map.depth_filter_tolerance': 0.15,
                'grid_map.depth_filter_maxdist': 5.0,
                'grid_map.depth_filter_mindist': 0.2,
                'grid_map.depth_filter_margin': 2,
                'grid_map.k_depth_scaling_factor': 1000.0,
                'grid_map.skip_pixel': 2,

                # Raycasting parameters
                'grid_map.p_hit': 0.65,
                'grid_map.p_miss': 0.35,
                'grid_map.p_min': 0.12,
                'grid_map.p_max': 0.90,
                'grid_map.p_occ': 0.80,
                'grid_map.min_ray_length': 0.1,
                'grid_map.max_ray_length': 4.5,

                # Visualization
                'grid_map.visualization_truncate_height': 3.0,
                'grid_map.visualization_truncate_low': -0.2,
                'grid_map.virtual_ceil_height': 3.0,
                'grid_map.virtual_ceil_low': -0.5,
                'grid_map.virtual_ceil_yp': 1000.0,
                'grid_map.virtual_ceil_yn': -1000.0,
                'grid_map.show_occ_time': False,

                # Frame and pose type
                'grid_map.frame_id': 'map',
                'grid_map.pose_type': 2,  # 1: POSE_STAMPED, 2: ODOMETRY
                'grid_map.local_map_margin': 1,
                'grid_map.ground_height': 0.0,

                # Timeouts
                'grid_map.odom_depth_timeout': 1.0,

                # ESDF parameters
                'grid_map.esdf_slice_height': 1.5,
                'grid_map.show_esdf_time': False,
                'grid_map.local_bound_inflate': 0.5,
            }],
            remappings=[
                ('grid_map/odom', '/mavros/local_position/odom'),
                ('grid_map/cloud', '/mid360/points'),
                ('grid_map/depth', '/camera/depth/image_raw'),
            ]
        ),
    ])
