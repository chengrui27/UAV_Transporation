#ifndef ESDF_MAP_READER_H
#define ESDF_MAP_READER_H

#include <rclcpp/rclcpp.hpp>
#include <plan_env/msg/esdf_map.hpp>
#include <Eigen/Eigen>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>

class ESDFMapReader {
public:
  ESDFMapReader(rclcpp::Node::SharedPtr node);
  ~ESDFMapReader();

  bool isMapValid() const { return map_valid_; }

  double getDistance(const Eigen::Vector3d& pos);

  Eigen::Vector3d getGradient(const Eigen::Vector3d& pos);

  void getDistanceAndGradient(const Eigen::Vector3d& pos, double& dist, Eigen::Vector3d& grad);

  bool isInLocalBound(const Eigen::Vector3d& pos) const;

  Eigen::Vector3d getMapOrigin() const { return map_origin_; }
  double getResolution() const { return resolution_; }
  Eigen::Vector3i getMapVoxelNum() const { return map_voxel_num_; }

private:
  void metadataCallback(const plan_env::msg::ESDFMap::SharedPtr msg);

  bool openSharedMemory(const std::string& shm_name);
  void closeSharedMemory();

  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) const;
  inline int toAddress(const Eigen::Vector3i& id) const;
  inline int toAddress(int x, int y, int z) const;
  inline void boundIndex(Eigen::Vector3i& id) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<plan_env::msg::ESDFMap>::SharedPtr metadata_sub_;

  Eigen::Vector3d map_origin_;
  double resolution_;
  double resolution_inv_;
  Eigen::Vector3i map_voxel_num_;

  Eigen::Vector3i local_bound_min_;
  Eigen::Vector3i local_bound_max_;

  int shm_fd_;
  float* shm_ptr_;
  size_t shm_size_;
  std::string shm_name_;

  bool map_valid_;
};

inline void ESDFMapReader::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) const {
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - map_origin_(i)) * resolution_inv_);
}

inline int ESDFMapReader::toAddress(const Eigen::Vector3i& id) const {
  return id(0) * map_voxel_num_(1) * map_voxel_num_(2) +
         id(1) * map_voxel_num_(2) + id(2);
}

inline int ESDFMapReader::toAddress(int x, int y, int z) const {
  return x * map_voxel_num_(1) * map_voxel_num_(2) +
         y * map_voxel_num_(2) + z;
}

inline void ESDFMapReader::boundIndex(Eigen::Vector3i& id) const {
  id(0) = std::max(std::min(id(0), map_voxel_num_(0) - 1), 0);
  id(1) = std::max(std::min(id(1), map_voxel_num_(1) - 1), 0);
  id(2) = std::max(std::min(id(2), map_voxel_num_(2) - 1), 0);
}

#endif

