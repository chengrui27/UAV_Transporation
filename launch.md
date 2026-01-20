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
./start_mavros_multi.sh

# 终端3: 启动NMPC控制器（会自动Offboard和解锁）
cd px4_ws && source install/setup.bash
ros2 launch px4_payload_nmpc nmpc_controller.launch.py
ros2 launch px4_nmpc nmpc_controller.launch.py
ros2 launch px4_offboard_control offboard_control.launch.py
ros2 run multi_payload multi_offboard
ros2 launch multi_payload multi_payload_nmpc.launch.py

# 终端4: 启动建图模块（构建ESDF地图，用于避障）
cd px4_ws && source install/setup.bash
ros2 run plan_env grid_map_node
ros2 launch plan_env grid_map.launch.py

# 终端5: 可视化
rviz2
ros2 run rqt_image_view rqt_image_view

# 多机仿真
cd PX4-Autopilot
./Tools/gazebo_sitl_multiple_run.sh -t px4_sitl_default -m iris -n 3 -w multi_trans
./Tools/gazebo_sitl_multiple_run.sh -t px4_sitl_default -w multi_trans -s "iris:1:2.1:0,iris:1:-0.3:1.9,iris:1:-0.3:-1.9"

ros2 launch status_estimate gazebo_vision_bridge.launch.py     pose_source:=gz     gz_world_name:=multi_trans_world

# 选择功能包编译
colcon build --packages-select px4_nmpc

# 开启性能模式 & 单核心运行
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
taskset -c 0 python3 nmpc.py






# 无人机 1（px4_instance = 0, sysid = 1）
ros2 launch mavros px4.launch \
  namespace:=uav1 \
  fcu_url:="udp://:14540@127.0.0.1:14580" \
  tgt_system:=1

# 无人机 2（px4_instance = 1, sysid = 2）
ros2 launch mavros px4.launch \
  namespace:=uav2 \
  fcu_url:="udp://:14541@127.0.0.1:14581" \
  tgt_system:=2

# 无人机 3（px4_instance = 2, sysid = 3）
ros2 launch mavros px4.launch \
  namespace:=uav3 \
  fcu_url:="udp://:14542@127.0.0.1:14582" \
  tgt_system:=3

