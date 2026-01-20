cmake_minimum_required(VERSION 3.8)
project(multi_payload)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

# ========== acados Configuration ==========
set(ACADOS_INSTALL_DIR "/home/rei/acados" CACHE PATH "Path to acados installation")
set(ACADOS_INCLUDE_DIR "${ACADOS_INSTALL_DIR}/include")
set(ACADOS_LIB_DIR "${ACADOS_INSTALL_DIR}/lib")

if(NOT EXISTS "${ACADOS_INCLUDE_DIR}")
  message(WARNING "acados not found at ${ACADOS_INSTALL_DIR}.")
endif()

set(ACADOS_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../multi_payload/c_generated_code")

include_directories(
  ${ACADOS_INCLUDE_DIR}
  ${ACADOS_INCLUDE_DIR}/blasfeo/include
  ${ACADOS_INCLUDE_DIR}/hpipm/include
  ${ACADOS_GENERATED_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(
  ${ACADOS_LIB_DIR}
)

add_executable(multi_offboard src/multi_offboard_node.cpp)

ament_target_dependencies(multi_offboard
  rclcpp
  mavros_msgs
  geometry_msgs
)

add_executable(multi_payload_nmpc src/multi_payload_nmpc_node.cpp)

ament_target_dependencies(multi_payload_nmpc
  rclcpp
  mavros_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  Eigen3
)

target_link_libraries(multi_payload_nmpc
  ${ACADOS_LIB_DIR}/libacados.so
  ${ACADOS_LIB_DIR}/libblasfeo.so
  ${ACADOS_LIB_DIR}/libhpipm.so
)

if(EXISTS "${ACADOS_GENERATED_DIR}/libacados_ocp_solver_multi_payload.so")
  target_link_libraries(multi_payload_nmpc
    ${ACADOS_GENERATED_DIR}/libacados_ocp_solver_multi_payload.so
  )
  message(STATUS "Found acados generated solver for multi_payload")
else()
  message(WARNING "Acados generated solver not found at ${ACADOS_GENERATED_DIR}")
endif()

install(TARGETS
  multi_offboard
  multi_payload_nmpc
  DESTINATION lib/${PROJECT_NAME}
)

# Install acados generated solver library
if(EXISTS "${ACADOS_GENERATED_DIR}/libacados_ocp_solver_multi_payload.so")
  install(FILES
    ${ACADOS_GENERATED_DIR}/libacados_ocp_solver_multi_payload.so
    DESTINATION lib
  )
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
