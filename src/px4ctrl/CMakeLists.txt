cmake_minimum_required(VERSION 3.8)
project(px4ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PositionCommand.msg"
  "msg/PositionCommandTrajectory.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(px4ctrl_node
    src/px4ctrl_node.cpp
    src/controller.cpp
    src/input.cpp
    src/params.cpp
)

target_include_directories(px4ctrl_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(px4ctrl_node
    rclcpp
    mavros_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs  
)

target_link_libraries(px4ctrl_node Eigen3::Eigen)
rosidl_target_interfaces(px4ctrl_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS px4ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
