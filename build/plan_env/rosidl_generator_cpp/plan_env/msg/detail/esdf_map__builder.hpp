// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from plan_env:msg/ESDFMap.idl
// generated code does not contain a copyright notice

#ifndef PLAN_ENV__MSG__DETAIL__ESDF_MAP__BUILDER_HPP_
#define PLAN_ENV__MSG__DETAIL__ESDF_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "plan_env/msg/detail/esdf_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace plan_env
{

namespace msg
{

namespace builder
{

class Init_ESDFMap_shm_name
{
public:
  explicit Init_ESDFMap_shm_name(::plan_env::msg::ESDFMap & msg)
  : msg_(msg)
  {}
  ::plan_env::msg::ESDFMap shm_name(::plan_env::msg::ESDFMap::_shm_name_type arg)
  {
    msg_.shm_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::plan_env::msg::ESDFMap msg_;
};

class Init_ESDFMap_local_bound_max
{
public:
  explicit Init_ESDFMap_local_bound_max(::plan_env::msg::ESDFMap & msg)
  : msg_(msg)
  {}
  Init_ESDFMap_shm_name local_bound_max(::plan_env::msg::ESDFMap::_local_bound_max_type arg)
  {
    msg_.local_bound_max = std::move(arg);
    return Init_ESDFMap_shm_name(msg_);
  }

private:
  ::plan_env::msg::ESDFMap msg_;
};

class Init_ESDFMap_local_bound_min
{
public:
  explicit Init_ESDFMap_local_bound_min(::plan_env::msg::ESDFMap & msg)
  : msg_(msg)
  {}
  Init_ESDFMap_local_bound_max local_bound_min(::plan_env::msg::ESDFMap::_local_bound_min_type arg)
  {
    msg_.local_bound_min = std::move(arg);
    return Init_ESDFMap_local_bound_max(msg_);
  }

private:
  ::plan_env::msg::ESDFMap msg_;
};

class Init_ESDFMap_map_voxel_num
{
public:
  explicit Init_ESDFMap_map_voxel_num(::plan_env::msg::ESDFMap & msg)
  : msg_(msg)
  {}
  Init_ESDFMap_local_bound_min map_voxel_num(::plan_env::msg::ESDFMap::_map_voxel_num_type arg)
  {
    msg_.map_voxel_num = std::move(arg);
    return Init_ESDFMap_local_bound_min(msg_);
  }

private:
  ::plan_env::msg::ESDFMap msg_;
};

class Init_ESDFMap_resolution
{
public:
  explicit Init_ESDFMap_resolution(::plan_env::msg::ESDFMap & msg)
  : msg_(msg)
  {}
  Init_ESDFMap_map_voxel_num resolution(::plan_env::msg::ESDFMap::_resolution_type arg)
  {
    msg_.resolution = std::move(arg);
    return Init_ESDFMap_map_voxel_num(msg_);
  }

private:
  ::plan_env::msg::ESDFMap msg_;
};

class Init_ESDFMap_map_origin
{
public:
  explicit Init_ESDFMap_map_origin(::plan_env::msg::ESDFMap & msg)
  : msg_(msg)
  {}
  Init_ESDFMap_resolution map_origin(::plan_env::msg::ESDFMap::_map_origin_type arg)
  {
    msg_.map_origin = std::move(arg);
    return Init_ESDFMap_resolution(msg_);
  }

private:
  ::plan_env::msg::ESDFMap msg_;
};

class Init_ESDFMap_header
{
public:
  Init_ESDFMap_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ESDFMap_map_origin header(::plan_env::msg::ESDFMap::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ESDFMap_map_origin(msg_);
  }

private:
  ::plan_env::msg::ESDFMap msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::plan_env::msg::ESDFMap>()
{
  return plan_env::msg::builder::Init_ESDFMap_header();
}

}  // namespace plan_env

#endif  // PLAN_ENV__MSG__DETAIL__ESDF_MAP__BUILDER_HPP_
