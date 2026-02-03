# ========== NMPC控制器 ==========
# 完整3步启动流程（需要3个终端）

# 终端1: 启动PX4 SITL Gazebo
cd PX4-Autopilot && ./reset_gazebo.sh
make px4_sitl gazebo
# 需要修改模型时再用，能够解决gazebo卡死，但是会导致相机的ros插件出错
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins

# 终端2: 启动MAVROS
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@localhost:14557
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" config_yaml:="/opt/ros/humble/share/mavros/launch/px4_config.yaml"

# 终端3: 启动NMPC控制器（会自动Offboard和解锁）
cd px4_ws && source install/setup.bash
ros2 launch px4_payload_nmpc nmpc_controller.launch.py
ros2 launch px4_nmpc nmpc_controller.launch.py
ros2 launch px4_offboard_control offboard_control.launch.py

# 终端4: 启动建图模块（构建ESDF地图，用于避障）
cd px4_ws && source install/setup.bash
ros2 run plan_env grid_map_node
ros2 launch plan_env grid_map.launch.py

# 终端5: 可视化
rviz2
ros2 run rqt_image_view rqt_image_view

# 负载位置估计
ros2 run px4_payload_nmpc payload_odom_from_vrpn     --ros-args -p quad_name:=t1 -p payload_name:=h1


colcon build --packages-select px4_nmpc