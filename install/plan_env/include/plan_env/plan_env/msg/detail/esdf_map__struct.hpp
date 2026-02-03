// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from plan_env:msg/ESDFMap.idl
// generated code does not contain a copyright notice

#ifndef PLAN_ENV__MSG__DETAIL__ESDF_MAP__STRUCT_HPP_
#define PLAN_ENV__MSG__DETAIL__ESDF_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__plan_env__msg__ESDFMap __attribute__((deprecated))
#else
# define DEPRECATED__plan_env__msg__ESDFMap __declspec(deprecated)
#endif

namespace plan_env
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ESDFMap_
{
  using Type = ESDFMap_<ContainerAllocator>;

  explicit ESDFMap_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 3>::iterator, double>(this->map_origin.begin(), this->map_origin.end(), 0.0);
      this->resolution = 0.0;
      std::fill<typename std::array<int32_t, 3>::iterator, int32_t>(this->map_voxel_num.begin(), this->map_voxel_num.end(), 0l);
      std::fill<typename std::array<int32_t, 3>::iterator, int32_t>(this->local_bound_min.begin(), this->local_bound_min.end(), 0l);
      std::fill<typename std::array<int32_t, 3>::iterator, int32_t>(this->local_bound_max.begin(), this->local_bound_max.end(), 0l);
      this->shm_name = "";
    }
  }

  explicit ESDFMap_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    map_origin(_alloc),
    map_voxel_num(_alloc),
    local_bound_min(_alloc),
    local_bound_max(_alloc),
    shm_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 3>::iterator, double>(this->map_origin.begin(), this->map_origin.end(), 0.0);
      this->resolution = 0.0;
      std::fill<typename std::array<int32_t, 3>::iterator, int32_t>(this->map_voxel_num.begin(), this->map_voxel_num.end(), 0l);
      std::fill<typename std::array<int32_t, 3>::iterator, int32_t>(this->local_bound_min.begin(), this->local_bound_min.end(), 0l);
      std::fill<typename std::array<int32_t, 3>::iterator, int32_t>(this->local_bound_max.begin(), this->local_bound_max.end(), 0l);
      this->shm_name = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _map_origin_type =
    std::array<double, 3>;
  _map_origin_type map_origin;
  using _resolution_type =
    double;
  _resolution_type resolution;
  using _map_voxel_num_type =
    std::array<int32_t, 3>;
  _map_voxel_num_type map_voxel_num;
  using _local_bound_min_type =
    std::array<int32_t, 3>;
  _local_bound_min_type local_bound_min;
  using _local_bound_max_type =
    std::array<int32_t, 3>;
  _local_bound_max_type local_bound_max;
  using _shm_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _shm_name_type shm_name;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__map_origin(
    const std::array<double, 3> & _arg)
  {
    this->map_origin = _arg;
    return *this;
  }
  Type & set__resolution(
    const double & _arg)
  {
    this->resolution = _arg;
    return *this;
  }
  Type & set__map_voxel_num(
    const std::array<int32_t, 3> & _arg)
  {
    this->map_voxel_num = _arg;
    return *this;
  }
  Type & set__local_bound_min(
    const std::array<int32_t, 3> & _arg)
  {
    this->local_bound_min = _arg;
    return *this;
  }
  Type & set__local_bound_max(
    const std::array<int32_t, 3> & _arg)
  {
    this->local_bound_max = _arg;
    return *this;
  }
  Type & set__shm_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->shm_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    plan_env::msg::ESDFMap_<ContainerAllocator> *;
  using ConstRawPtr =
    const plan_env::msg::ESDFMap_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<plan_env::msg::ESDFMap_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<plan_env::msg::ESDFMap_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      plan_env::msg::ESDFMap_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<plan_env::msg::ESDFMap_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      plan_env::msg::ESDFMap_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<plan_env::msg::ESDFMap_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<plan_env::msg::ESDFMap_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<plan_env::msg::ESDFMap_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__plan_env__msg__ESDFMap
    std::shared_ptr<plan_env::msg::ESDFMap_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__plan_env__msg__ESDFMap
    std::shared_ptr<plan_env::msg::ESDFMap_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ESDFMap_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->map_origin != other.map_origin) {
      return false;
    }
    if (this->resolution != other.resolution) {
      return false;
    }
    if (this->map_voxel_num != other.map_voxel_num) {
      return false;
    }
    if (this->local_bound_min != other.local_bound_min) {
      return false;
    }
    if (this->local_bound_max != other.local_bound_max) {
      return false;
    }
    if (this->shm_name != other.shm_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const ESDFMap_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ESDFMap_

// alias to use template instance with default allocator
using ESDFMap =
  plan_env::msg::ESDFMap_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace plan_env

#endif  // PLAN_ENV__MSG__DETAIL__ESDF_MAP__STRUCT_HPP_
