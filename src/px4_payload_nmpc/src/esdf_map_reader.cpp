#include "px4_payload_nmpc/esdf_map_reader.h"
#include <cstring>

ESDFMapReader::ESDFMapReader(rclcpp::Node::SharedPtr node)
  : node_(std::move(node)),
    shm_fd_(-1),
    shm_ptr_(nullptr),
    shm_size_(0),
    map_valid_(false),
    resolution_(0.1),
    resolution_inv_(10.0)
{
  map_origin_ = Eigen::Vector3d::Zero();
  map_voxel_num_ = Eigen::Vector3i::Zero();
  local_bound_min_ = Eigen::Vector3i::Zero();
  local_bound_max_ = Eigen::Vector3i::Zero();

  metadata_sub_ = node_->create_subscription<plan_env::msg::ESDFMap>(
    "/grid_map/esdf_metadata", 10,
    std::bind(&ESDFMapReader::metadataCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "ESDF Map Reader (payload) initialized, waiting for map metadata...");
}

ESDFMapReader::~ESDFMapReader() {
  closeSharedMemory();
}

void ESDFMapReader::metadataCallback(const plan_env::msg::ESDFMap::SharedPtr msg) {
  map_origin_(0) = msg->map_origin[0];
  map_origin_(1) = msg->map_origin[1];
  map_origin_(2) = msg->map_origin[2];

  resolution_ = msg->resolution;
  resolution_inv_ = 1.0 / resolution_;

  map_voxel_num_(0) = msg->map_voxel_num[0];
  map_voxel_num_(1) = msg->map_voxel_num[1];
  map_voxel_num_(2) = msg->map_voxel_num[2];

  local_bound_min_(0) = msg->local_bound_min[0];
  local_bound_min_(1) = msg->local_bound_min[1];
  local_bound_min_(2) = msg->local_bound_min[2];

  local_bound_max_(0) = msg->local_bound_max[0];
  local_bound_max_(1) = msg->local_bound_max[1];
  local_bound_max_(2) = msg->local_bound_max[2];

  if (shm_name_ != msg->shm_name) {
    closeSharedMemory();

    if (openSharedMemory(msg->shm_name)) {
      map_valid_ = true;
      RCLCPP_INFO(node_->get_logger(),
        "ESDF Map (payload) updated: origin=[%.1f, %.1f, %.1f], resolution=%.2f, voxels=[%d, %d, %d], local_bound=[%d:%d, %d:%d, %d:%d]",
        map_origin_(0), map_origin_(1), map_origin_(2),
        resolution_,
        map_voxel_num_(0), map_voxel_num_(1), map_voxel_num_(2),
        local_bound_min_(0), local_bound_max_(0),
        local_bound_min_(1), local_bound_max_(1),
        local_bound_min_(2), local_bound_max_(2));
    }
  }
}

bool ESDFMapReader::openSharedMemory(const std::string& shm_name) {
  shm_name_ = shm_name;
  // 4 floats per voxel: [distance, grad_x, grad_y, grad_z]
  shm_size_ = map_voxel_num_(0) * map_voxel_num_(1) * map_voxel_num_(2) * 4 * sizeof(float);

  shm_fd_ = shm_open(shm_name_.c_str(), O_RDONLY, 0666);
  if (shm_fd_ == -1) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open shared memory '%s': %s",
                 shm_name_.c_str(), strerror(errno));
    return false;
  }

  shm_ptr_ = static_cast<float*>(mmap(nullptr, shm_size_, PROT_READ, MAP_SHARED, shm_fd_, 0));
  if (shm_ptr_ == MAP_FAILED) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to map shared memory: %s", strerror(errno));
    close(shm_fd_);
    shm_fd_ = -1;
    shm_ptr_ = nullptr;
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Shared memory (payload) opened: %s, size: %.2f MB",
              shm_name_.c_str(), shm_size_ / 1024.0 / 1024.0);
  return true;
}

void ESDFMapReader::closeSharedMemory() {
  if (shm_ptr_ != nullptr && shm_ptr_ != MAP_FAILED) {
    munmap(shm_ptr_, shm_size_);
    shm_ptr_ = nullptr;
  }

  if (shm_fd_ != -1) {
    close(shm_fd_);
    shm_fd_ = -1;
  }

  map_valid_ = false;
}

double ESDFMapReader::getDistance(const Eigen::Vector3d& pos) {
  if (!map_valid_ || shm_ptr_ == nullptr) {
    return 10000.0;
  }

  Eigen::Vector3i id;
  posToIndex(pos, id);
  boundIndex(id);

  int idx = toAddress(id);
  // Read distance from shared memory (first element of 4-float block)
  return static_cast<double>(shm_ptr_[idx * 4]);
}

Eigen::Vector3d ESDFMapReader::getGradient(const Eigen::Vector3d& pos) {
  if (!map_valid_ || shm_ptr_ == nullptr) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3i id;
  posToIndex(pos, id);
  boundIndex(id);

  int idx = toAddress(id);
  // Read gradient from shared memory (elements 1,2,3 of 4-float block)
  Eigen::Vector3d grad;
  grad(0) = static_cast<double>(shm_ptr_[idx * 4 + 1]);
  grad(1) = static_cast<double>(shm_ptr_[idx * 4 + 2]);
  grad(2) = static_cast<double>(shm_ptr_[idx * 4 + 3]);
  return grad;
}

void ESDFMapReader::getDistanceAndGradient(const Eigen::Vector3d& pos,
                                            double& dist,
                                            Eigen::Vector3d& grad) {
  if (!map_valid_ || shm_ptr_ == nullptr) {
    dist = 10000.0;
    grad = Eigen::Vector3d::Zero();
    return;
  }

  Eigen::Vector3i id;
  posToIndex(pos, id);
  boundIndex(id);

  int idx = toAddress(id);
  // Read both distance and gradient in one memory access
  dist = static_cast<double>(shm_ptr_[idx * 4 + 0]);
  grad(0) = static_cast<double>(shm_ptr_[idx * 4 + 1]);
  grad(1) = static_cast<double>(shm_ptr_[idx * 4 + 2]);
  grad(2) = static_cast<double>(shm_ptr_[idx * 4 + 3]);
}

bool ESDFMapReader::isInLocalBound(const Eigen::Vector3d& pos) const {
  if (!map_valid_) {
    return false;
  }

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return (id(0) >= local_bound_min_(0) && id(0) <= local_bound_max_(0) &&
          id(1) >= local_bound_min_(1) && id(1) <= local_bound_max_(1) &&
          id(2) >= local_bound_min_(2) && id(2) <= local_bound_max_(2));
}

