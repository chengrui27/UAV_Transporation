#include <rclcpp/rclcpp.hpp>
#include "plan_env/grid_map.h"

int main(int argc, char** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<rclcpp::Node>("grid_map_node");

  RCLCPP_INFO(node->get_logger(), "Starting ESDF Grid Map Node...");

  // Create GridMap instance
  GridMap::Ptr grid_map_ptr = std::make_shared<GridMap>();

  // Initialize map with node
  grid_map_ptr->initMap(node);

  RCLCPP_INFO(node->get_logger(), "Grid Map initialized successfully!");

  // Spin
  rclcpp::spin(node);

  // Cleanup shared memory before shutdown
  grid_map_ptr->cleanupSharedMemory();

  // Shutdown
  rclcpp::shutdown();

  return 0;
}
