cmake_minimum_required(VERSION 3.8)
project(px4_nmpc)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========== Find dependencies ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(plan_env REQUIRED)

# ========== acados Configuration ==========
# 设置acados安装路径
set(ACADOS_INSTALL_DIR "/home/rei/acados" CACHE PATH "Path to acados installation")
set(ACADOS_INCLUDE_DIR "${ACADOS_INSTALL_DIR}/include")
set(ACADOS_LIB_DIR "${ACADOS_INSTALL_DIR}/lib")

# 检查acados是否存在
if(NOT EXISTS "${ACADOS_INCLUDE_DIR}")
  message(WARNING "acados not found at ${ACADOS_INSTALL_DIR}.
  Please set ACADOS_INSTALL_DIR or install acados.")
endif()

# 添加acados生成的求解器路径 (由px4_nmpc.py生成)
set(ACADOS_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../px4/c_generated_code")

include_directories(
  include
  ${ACADOS_INCLUDE_DIR}
  ${ACADOS_INCLUDE_DIR}/blasfeo/include
  ${ACADOS_INCLUDE_DIR}/hpipm/include
  ${ACADOS_GENERATED_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(
  ${ACADOS_LIB_DIR}
)

# ========== Build ESDF Map Reader Library ==========
add_library(esdf_map_reader SHARED
  src/esdf_map_reader.cpp
)

ament_target_dependencies(esdf_map_reader
  rclcpp
  plan_env
  Eigen3
)

# ========== Build NMPC Controller Node ==========
add_executable(nmpc_controller src/nmpc_controller.cpp)

ament_target_dependencies(nmpc_controller
  rclcpp
  geometry_msgs
  nav_msgs
  mavros_msgs
  plan_env
  Eigen3
)

# Link acados libraries
target_link_libraries(nmpc_controller
  esdf_map_reader
  ${ACADOS_LIB_DIR}/libacados.so
  ${ACADOS_LIB_DIR}/libblasfeo.so
  ${ACADOS_LIB_DIR}/libhpipm.so
)

# Link generated solver (需要先运行px4_nmpc.py生成)
if(EXISTS "${ACADOS_GENERATED_DIR}/libacados_ocp_solver_px4_model.so")
  target_link_libraries(nmpc_controller
    ${ACADOS_GENERATED_DIR}/libacados_ocp_solver_px4_model.so
  )
  message(STATUS "Found acados generated solver")
else()
  message(WARNING "Acados generated solver not found!
  Please run: cd ~/px4_ws/px4 && python3 px4_nmpc.py")
endif()

# ========== Install ==========
install(TARGETS
  esdf_map_reader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  nmpc_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install acados generated solver library
if(EXISTS "${ACADOS_GENERATED_DIR}/libacados_ocp_solver_px4_model.so")
  install(FILES
    ${ACADOS_GENERATED_DIR}/libacados_ocp_solver_px4_model.so
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
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
