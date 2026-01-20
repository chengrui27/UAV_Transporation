cmake_minimum_required(VERSION 3.8)
project(plan_env)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -march=native")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========== Find ROS2 dependencies ==========
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ========== Find third-party dependencies ==========
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

# ========== Include directories ==========
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

# ========== Generate messages ==========
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ESDFMap.msg"
  DEPENDENCIES std_msgs
)

# ========== Build plan_env library ==========
add_library(plan_env_lib SHARED
  src/grid_map.cpp
  src/raycast.cpp
)

ament_target_dependencies(plan_env_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  message_filters
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  Eigen3
)

target_link_libraries(plan_env_lib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(plan_env_lib "${cpp_typesupport_target}")

# Add dependency to ensure messages are generated before library
add_dependencies(plan_env_lib ${PROJECT_NAME})

# ========== Build grid_map_node executable ==========
add_executable(grid_map_node
  src/grid_map_node.cpp
)

ament_target_dependencies(grid_map_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  message_filters
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  Eigen3
)

target_link_libraries(grid_map_node
  plan_env_lib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# ========== Install ==========
# Install library
install(TARGETS plan_env_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executable
install(TARGETS grid_map_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

# Install Python scripts
install(PROGRAMS
  scripts/pointcloud_transformer.py
  scripts/odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

# ========== Export ==========
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  message_filters
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  Eigen3
  eigen3_cmake_module
)

ament_export_include_directories(include)
ament_export_libraries(plan_env_lib)

# ========== Testing ==========
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
