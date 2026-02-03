// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from plan_env:msg/ESDFMap.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "plan_env/msg/detail/esdf_map__rosidl_typesupport_introspection_c.h"
#include "plan_env/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "plan_env/msg/detail/esdf_map__functions.h"
#include "plan_env/msg/detail/esdf_map__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `shm_name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__ESDFMap_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  plan_env__msg__ESDFMap__init(message_memory);
}

void plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__ESDFMap_fini_function(void * message_memory)
{
  plan_env__msg__ESDFMap__fini(message_memory);
}

size_t plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__size_function__ESDFMap__map_origin(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_const_function__ESDFMap__map_origin(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_function__ESDFMap__map_origin(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__fetch_function__ESDFMap__map_origin(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_const_function__ESDFMap__map_origin(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__assign_function__ESDFMap__map_origin(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_function__ESDFMap__map_origin(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__size_function__ESDFMap__map_voxel_num(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_const_function__ESDFMap__map_voxel_num(
  const void * untyped_member, size_t index)
{
  const int32_t * member =
    (const int32_t *)(untyped_member);
  return &member[index];
}

void * plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_function__ESDFMap__map_voxel_num(
  void * untyped_member, size_t index)
{
  int32_t * member =
    (int32_t *)(untyped_member);
  return &member[index];
}

void plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__fetch_function__ESDFMap__map_voxel_num(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_const_function__ESDFMap__map_voxel_num(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__assign_function__ESDFMap__map_voxel_num(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_function__ESDFMap__map_voxel_num(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

size_t plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__size_function__ESDFMap__local_bound_min(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_const_function__ESDFMap__local_bound_min(
  const void * untyped_member, size_t index)
{
  const int32_t * member =
    (const int32_t *)(untyped_member);
  return &member[index];
}

void * plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_function__ESDFMap__local_bound_min(
  void * untyped_member, size_t index)
{
  int32_t * member =
    (int32_t *)(untyped_member);
  return &member[index];
}

void plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__fetch_function__ESDFMap__local_bound_min(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_const_function__ESDFMap__local_bound_min(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__assign_function__ESDFMap__local_bound_min(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_function__ESDFMap__local_bound_min(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

size_t plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__size_function__ESDFMap__local_bound_max(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_const_function__ESDFMap__local_bound_max(
  const void * untyped_member, size_t index)
{
  const int32_t * member =
    (const int32_t *)(untyped_member);
  return &member[index];
}

void * plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_function__ESDFMap__local_bound_max(
  void * untyped_member, size_t index)
{
  int32_t * member =
    (int32_t *)(untyped_member);
  return &member[index];
}

void plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__fetch_function__ESDFMap__local_bound_max(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_const_function__ESDFMap__local_bound_max(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__assign_function__ESDFMap__local_bound_max(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_function__ESDFMap__local_bound_max(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__ESDFMap_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_env__msg__ESDFMap, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "map_origin",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(plan_env__msg__ESDFMap, map_origin),  // bytes offset in struct
    NULL,  // default value
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__size_function__ESDFMap__map_origin,  // size() function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_const_function__ESDFMap__map_origin,  // get_const(index) function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_function__ESDFMap__map_origin,  // get(index) function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__fetch_function__ESDFMap__map_origin,  // fetch(index, &value) function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__assign_function__ESDFMap__map_origin,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "resolution",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_env__msg__ESDFMap, resolution),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "map_voxel_num",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(plan_env__msg__ESDFMap, map_voxel_num),  // bytes offset in struct
    NULL,  // default value
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__size_function__ESDFMap__map_voxel_num,  // size() function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_const_function__ESDFMap__map_voxel_num,  // get_const(index) function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_function__ESDFMap__map_voxel_num,  // get(index) function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__fetch_function__ESDFMap__map_voxel_num,  // fetch(index, &value) function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__assign_function__ESDFMap__map_voxel_num,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "local_bound_min",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(plan_env__msg__ESDFMap, local_bound_min),  // bytes offset in struct
    NULL,  // default value
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__size_function__ESDFMap__local_bound_min,  // size() function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_const_function__ESDFMap__local_bound_min,  // get_const(index) function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_function__ESDFMap__local_bound_min,  // get(index) function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__fetch_function__ESDFMap__local_bound_min,  // fetch(index, &value) function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__assign_function__ESDFMap__local_bound_min,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "local_bound_max",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(plan_env__msg__ESDFMap, local_bound_max),  // bytes offset in struct
    NULL,  // default value
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__size_function__ESDFMap__local_bound_max,  // size() function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_const_function__ESDFMap__local_bound_max,  // get_const(index) function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__get_function__ESDFMap__local_bound_max,  // get(index) function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__fetch_function__ESDFMap__local_bound_max,  // fetch(index, &value) function pointer
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__assign_function__ESDFMap__local_bound_max,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "shm_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(plan_env__msg__ESDFMap, shm_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__ESDFMap_message_members = {
  "plan_env__msg",  // message namespace
  "ESDFMap",  // message name
  7,  // number of fields
  sizeof(plan_env__msg__ESDFMap),
  plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__ESDFMap_message_member_array,  // message members
  plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__ESDFMap_init_function,  // function to initialize message memory (memory has to be allocated)
  plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__ESDFMap_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__ESDFMap_message_type_support_handle = {
  0,
  &plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__ESDFMap_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_plan_env
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, plan_env, msg, ESDFMap)() {
  plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__ESDFMap_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__ESDFMap_message_type_support_handle.typesupport_identifier) {
    plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__ESDFMap_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &plan_env__msg__ESDFMap__rosidl_typesupport_introspection_c__ESDFMap_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
