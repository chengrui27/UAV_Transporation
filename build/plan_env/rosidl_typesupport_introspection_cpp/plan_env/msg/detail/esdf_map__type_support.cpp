// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from plan_env:msg/ESDFMap.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "plan_env/msg/detail/esdf_map__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace plan_env
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ESDFMap_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) plan_env::msg::ESDFMap(_init);
}

void ESDFMap_fini_function(void * message_memory)
{
  auto typed_message = static_cast<plan_env::msg::ESDFMap *>(message_memory);
  typed_message->~ESDFMap();
}

size_t size_function__ESDFMap__map_origin(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__ESDFMap__map_origin(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__ESDFMap__map_origin(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__ESDFMap__map_origin(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__ESDFMap__map_origin(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__ESDFMap__map_origin(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__ESDFMap__map_origin(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__ESDFMap__map_voxel_num(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__ESDFMap__map_voxel_num(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int32_t, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__ESDFMap__map_voxel_num(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int32_t, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__ESDFMap__map_voxel_num(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__ESDFMap__map_voxel_num(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__ESDFMap__map_voxel_num(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__ESDFMap__map_voxel_num(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

size_t size_function__ESDFMap__local_bound_min(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__ESDFMap__local_bound_min(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int32_t, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__ESDFMap__local_bound_min(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int32_t, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__ESDFMap__local_bound_min(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__ESDFMap__local_bound_min(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__ESDFMap__local_bound_min(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__ESDFMap__local_bound_min(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

size_t size_function__ESDFMap__local_bound_max(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__ESDFMap__local_bound_max(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int32_t, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__ESDFMap__local_bound_max(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int32_t, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__ESDFMap__local_bound_max(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__ESDFMap__local_bound_max(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__ESDFMap__local_bound_max(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__ESDFMap__local_bound_max(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ESDFMap_message_member_array[7] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_env::msg::ESDFMap, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "map_origin",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(plan_env::msg::ESDFMap, map_origin),  // bytes offset in struct
    nullptr,  // default value
    size_function__ESDFMap__map_origin,  // size() function pointer
    get_const_function__ESDFMap__map_origin,  // get_const(index) function pointer
    get_function__ESDFMap__map_origin,  // get(index) function pointer
    fetch_function__ESDFMap__map_origin,  // fetch(index, &value) function pointer
    assign_function__ESDFMap__map_origin,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "resolution",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_env::msg::ESDFMap, resolution),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "map_voxel_num",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(plan_env::msg::ESDFMap, map_voxel_num),  // bytes offset in struct
    nullptr,  // default value
    size_function__ESDFMap__map_voxel_num,  // size() function pointer
    get_const_function__ESDFMap__map_voxel_num,  // get_const(index) function pointer
    get_function__ESDFMap__map_voxel_num,  // get(index) function pointer
    fetch_function__ESDFMap__map_voxel_num,  // fetch(index, &value) function pointer
    assign_function__ESDFMap__map_voxel_num,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "local_bound_min",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(plan_env::msg::ESDFMap, local_bound_min),  // bytes offset in struct
    nullptr,  // default value
    size_function__ESDFMap__local_bound_min,  // size() function pointer
    get_const_function__ESDFMap__local_bound_min,  // get_const(index) function pointer
    get_function__ESDFMap__local_bound_min,  // get(index) function pointer
    fetch_function__ESDFMap__local_bound_min,  // fetch(index, &value) function pointer
    assign_function__ESDFMap__local_bound_min,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "local_bound_max",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(plan_env::msg::ESDFMap, local_bound_max),  // bytes offset in struct
    nullptr,  // default value
    size_function__ESDFMap__local_bound_max,  // size() function pointer
    get_const_function__ESDFMap__local_bound_max,  // get_const(index) function pointer
    get_function__ESDFMap__local_bound_max,  // get(index) function pointer
    fetch_function__ESDFMap__local_bound_max,  // fetch(index, &value) function pointer
    assign_function__ESDFMap__local_bound_max,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "shm_name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_env::msg::ESDFMap, shm_name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ESDFMap_message_members = {
  "plan_env::msg",  // message namespace
  "ESDFMap",  // message name
  7,  // number of fields
  sizeof(plan_env::msg::ESDFMap),
  ESDFMap_message_member_array,  // message members
  ESDFMap_init_function,  // function to initialize message memory (memory has to be allocated)
  ESDFMap_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ESDFMap_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ESDFMap_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace plan_env


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<plan_env::msg::ESDFMap>()
{
  return &::plan_env::msg::rosidl_typesupport_introspection_cpp::ESDFMap_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, plan_env, msg, ESDFMap)() {
  return &::plan_env::msg::rosidl_typesupport_introspection_cpp::ESDFMap_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
