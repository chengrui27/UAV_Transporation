cmake_minimum_required(VERSION 3.8)
project(px4_payload_nmpc)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_WITH_ACADOS "Build NMPC targets that depend on acados" ON)

# ========== Find dependencies ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

# Hint CMake where to find the local plan_env package if this workspace was already built
set(_local_plan_env_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../install/plan_env/share/plan_env/cmake")
if(EXISTS "${_local_plan_env_DIR}/plan_envConfig.cmake")
  set(plan_env_DIR "${_local_plan_env_DIR}")
endif()

find_package(plan_env REQUIRED)

# ========== acados Configuration ==========
# 设置acados安装路径，默认读取环境变量或~/acados
if(NOT ACADOS_INSTALL_DIR AND DEFINED ENV{ACADOS_INSTALL_DIR})
  set(ACADOS_INSTALL_DIR "$ENV{ACADOS_INSTALL_DIR}" CACHE PATH "Path to acados installation")
endif()
if(NOT ACADOS_INSTALL_DIR)
  set(ACADOS_INSTALL_DIR "$ENV{HOME}/acados" CACHE PATH "Path to acados installation")
endif()

set(ACADOS_INCLUDE_DIR "${ACADOS_INSTALL_DIR}/include")
set(ACADOS_LIB_DIR "${ACADOS_INSTALL_DIR}/lib")

# 添加acados生成的求解器路径 (由px4_payload_nmpc.py生成)
set(ACADOS_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../px4_payload/c_generated_code")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

if(BUILD_WITH_ACADOS)
  include_directories(
    ${ACADOS_INCLUDE_DIR}
    ${ACADOS_INCLUDE_DIR}/blasfeo/include
    ${ACADOS_INCLUDE_DIR}/hpipm/include
    ${ACADOS_GENERATED_DIR}
  )
  link_directories(
    ${ACADOS_LIB_DIR}
  )
endif()

# ========== Build ESDF Map Reader Library ==========
add_library(esdf_map_reader_payload SHARED
  src/esdf_map_reader.cpp
)

ament_target_dependencies(esdf_map_reader_payload
  rclcpp
  plan_env
  Eigen3
)

# ========== Build Payload Odometry Bridge (VRPN -> /payload/odom) ==========
add_executable(payload_odom_from_vrpn src/payload_odom_from_vrpn.cpp)

ament_target_dependencies(payload_odom_from_vrpn
  rclcpp
  nav_msgs
  geometry_msgs
  Eigen3
)

# ========== Build NMPC Controller Node ==========
if(BUILD_WITH_ACADOS)
  add_executable(nmpc_controller src/nmpc_controller.cpp)

  ament_target_dependencies(nmpc_controller
    rclcpp
    nav_msgs
    geometry_msgs
    std_msgs
    mavros_msgs
    Eigen3
    plan_env
  )

  # Link acados libraries
  target_link_libraries(nmpc_controller
    esdf_map_reader_payload
    ${ACADOS_LIB_DIR}/libacados.so
    ${ACADOS_LIB_DIR}/libblasfeo.so
    ${ACADOS_LIB_DIR}/libhpipm.so
  )

  # Link generated solver (需要先运行px4_payload_nmpc.py生成)
  if(EXISTS "${ACADOS_GENERATED_DIR}/libacados_ocp_solver_px4_payload_model.so")
    target_link_libraries(nmpc_controller
      ${ACADOS_GENERATED_DIR}/libacados_ocp_solver_px4_payload_model.so
    )
    message(STATUS "Found acados generated solver")
  else()
    message(WARNING "Acados generated solver not found! Run: cd ~/px4_ws/px4_payload && python3 px4_payload_nmpc.py")
  endif()
else()
  message(STATUS "Skipping nmpc_controller build (BUILD_WITH_ACADOS=OFF)")
endif()

# ========== Install ==========
# Libraries
install(TARGETS
  esdf_map_reader_payload
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Executables (ROS2 nodes)
install(TARGETS
  payload_odom_from_vrpn
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_WITH_ACADOS)
  install(TARGETS
    nmpc_controller
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

# Install acados generated solver library
if(BUILD_WITH_ACADOS AND EXISTS "${ACADOS_GENERATED_DIR}/libacados_ocp_solver_px4_payload_model.so")
  install(FILES
    ${ACADOS_GENERATED_DIR}/libacados_ocp_solver_px4_payload_model.so
    DESTINATION lib
  )
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ========== Testing ==========
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
