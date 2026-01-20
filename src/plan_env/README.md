# plan_env - ESDF Mapping Package (ROS2 Humble)

本功能包是从 ROS1 Noetic 迁移到 ROS2 Humble 的 ESDF (Euclidean Signed Distance Field) 地图构建包。

## 功能

- 基于深度图像和点云构建占据栅格地图 (Occupancy Grid Map)
- 实时计算 ESDF 距离场用于路径规划
- 支持相机深度图 + 里程计融合
- 支持点云直接输入
- 提供可视化输出

## 编译

```bash
cd ~/px4_ws
colcon build --packages-select plan_env
source install/setup.bash
```

## 使用方式

### 方式 1: 作为库使用

在其他功能包的 CMakeLists.txt 中添加：

```cmake
find_package(plan_env REQUIRED)

ament_target_dependencies(your_node
  plan_env
  # ... other dependencies
)

target_link_libraries(your_node
  plan_env
)
```

在你的代码中：

```cpp
#include <plan_env/grid_map.h>

// 在节点中创建 GridMap 实例
GridMap::Ptr grid_map = std::make_shared<GridMap>();
grid_map->initMap(node);  // node 是 rclcpp::Node::SharedPtr
```

### 方式 2: 作为独立节点运行

```bash
# 直接运行节点（需要先配置参数）
ros2 run plan_env grid_map_node

# 或使用 launch 文件启动
ros2 launch plan_env grid_map.launch.py
```

## 话题订阅

- `/grid_map/odom` (nav_msgs/Odometry) - 里程计信息
- `/grid_map/cloud` (sensor_msgs/PointCloud2) - 点云数据（可选）
- `/grid_map/depth` (sensor_msgs/Image) - 深度图像（与 odom/pose 同步）

## 话题发布

- `/grid_map/occupancy` (sensor_msgs/PointCloud2) - 占据栅格地图可视化
- `/grid_map/occupancy_inflate` (sensor_msgs/PointCloud2) - 膨胀后的地图
- `/grid_map/esdf` (sensor_msgs/PointCloud2) - ESDF 距离场可视化

## 主要参数

在 [launch/grid_map.launch.py](launch/grid_map.launch.py) 中可以配置以下参数：

### 地图参数
- `grid_map.map_size_x/y/z`: 地图尺寸 (米)
- `grid_map.resolution`: 地图分辨率 (米/体素)
- `grid_map.obstacles_inflation`: 障碍物膨胀距离 (米)

### 相机参数
- `grid_map.fx/fy/cx/cy`: 相机内参

### 深度滤波
- `grid_map.use_depth_filter`: 是否启用深度滤波
- `grid_map.depth_filter_maxdist/mindist`: 深度有效范围

### Raycasting 参数
- `grid_map.p_hit/p_miss/p_occ`: 占据概率参数

## 示例配置

```bash
# 修改地图大小
ros2 launch plan_env grid_map.launch.py map_size_x:=50.0 map_size_y:=50.0 map_size_z:=3.0

# 修改分辨率
ros2 launch plan_env grid_map.launch.py resolution:=0.05
```

## 与 PX4 集成

本包已针对 PX4 无人机进行优化，默认话题映射：
- 里程计: `/mavros/local_position/odom`
- 点云/深度: `/camera/depth/points` 或 `/camera/depth/image_raw`

## API 参考

### GridMap 类主要方法

```cpp
// 初始化地图
void initMap(rclcpp::Node::SharedPtr node);

// 获取占据状态
int getOccupancy(Eigen::Vector3d pos);
int getInflateOccupancy(Eigen::Vector3d pos);

// 获取 ESDF 距离
double getDistance(const Eigen::Vector3d& pos);

// 查询地图属性
bool isInMap(const Eigen::Vector3d &pos);
bool isKnownFree(const Eigen::Vector3i &id);
bool isKnownOccupied(const Eigen::Vector3i &id);

// 获取地图信息
double getResolution();
Eigen::Vector3d getOrigin();
void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
```

## ROS2 迁移说明

从 ROS1 迁移的主要变更：
- ✅ 所有 ROS API 已更新为 ROS2 Humble
- ✅ 参数系统改用 `declare_parameter` / `get_parameter`
- ✅ 消息类型改用 `sensor_msgs::msg::*` 格式
- ✅ 订阅/发布使用 shared_ptr
- ✅ 定时器使用 `create_wall_timer`
- ✅ 日志使用 `RCLCPP_INFO/WARN/ERROR`
- ✅ 同时支持作为库和独立节点使用

## 依赖项

- ROS2 Humble
- Eigen3
- PCL (Point Cloud Library)
- OpenCV
- cv_bridge
- message_filters
- pcl_conversions

## 许可证

MIT License

## 作者

- 原作者: iszhouxin@zju.edu.cn
- ROS2 迁移: Claude Code Assistant
