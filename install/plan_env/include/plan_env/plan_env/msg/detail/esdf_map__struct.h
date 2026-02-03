// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from plan_env:msg/ESDFMap.idl
// generated code does not contain a copyright notice

#ifndef PLAN_ENV__MSG__DETAIL__ESDF_MAP__STRUCT_H_
#define PLAN_ENV__MSG__DETAIL__ESDF_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'shm_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ESDFMap in the package plan_env.
typedef struct plan_env__msg__ESDFMap
{
  std_msgs__msg__Header header;
  /// Map parameters
  double map_origin[3];
  double resolution;
  int32_t map_voxel_num[3];
  /// Local bound (valid data range)
  int32_t local_bound_min[3];
  int32_t local_bound_max[3];
  /// Shared memory name
  rosidl_runtime_c__String shm_name;
} plan_env__msg__ESDFMap;

// Struct for a sequence of plan_env__msg__ESDFMap.
typedef struct plan_env__msg__ESDFMap__Sequence
{
  plan_env__msg__ESDFMap * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} plan_env__msg__ESDFMap__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLAN_ENV__MSG__DETAIL__ESDF_MAP__STRUCT_H_
