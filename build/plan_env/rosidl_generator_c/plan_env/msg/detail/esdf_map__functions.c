// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from plan_env:msg/ESDFMap.idl
// generated code does not contain a copyright notice
#include "plan_env/msg/detail/esdf_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `shm_name`
#include "rosidl_runtime_c/string_functions.h"

bool
plan_env__msg__ESDFMap__init(plan_env__msg__ESDFMap * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    plan_env__msg__ESDFMap__fini(msg);
    return false;
  }
  // map_origin
  // resolution
  // map_voxel_num
  // local_bound_min
  // local_bound_max
  // shm_name
  if (!rosidl_runtime_c__String__init(&msg->shm_name)) {
    plan_env__msg__ESDFMap__fini(msg);
    return false;
  }
  return true;
}

void
plan_env__msg__ESDFMap__fini(plan_env__msg__ESDFMap * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // map_origin
  // resolution
  // map_voxel_num
  // local_bound_min
  // local_bound_max
  // shm_name
  rosidl_runtime_c__String__fini(&msg->shm_name);
}

bool
plan_env__msg__ESDFMap__are_equal(const plan_env__msg__ESDFMap * lhs, const plan_env__msg__ESDFMap * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // map_origin
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->map_origin[i] != rhs->map_origin[i]) {
      return false;
    }
  }
  // resolution
  if (lhs->resolution != rhs->resolution) {
    return false;
  }
  // map_voxel_num
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->map_voxel_num[i] != rhs->map_voxel_num[i]) {
      return false;
    }
  }
  // local_bound_min
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->local_bound_min[i] != rhs->local_bound_min[i]) {
      return false;
    }
  }
  // local_bound_max
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->local_bound_max[i] != rhs->local_bound_max[i]) {
      return false;
    }
  }
  // shm_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->shm_name), &(rhs->shm_name)))
  {
    return false;
  }
  return true;
}

bool
plan_env__msg__ESDFMap__copy(
  const plan_env__msg__ESDFMap * input,
  plan_env__msg__ESDFMap * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // map_origin
  for (size_t i = 0; i < 3; ++i) {
    output->map_origin[i] = input->map_origin[i];
  }
  // resolution
  output->resolution = input->resolution;
  // map_voxel_num
  for (size_t i = 0; i < 3; ++i) {
    output->map_voxel_num[i] = input->map_voxel_num[i];
  }
  // local_bound_min
  for (size_t i = 0; i < 3; ++i) {
    output->local_bound_min[i] = input->local_bound_min[i];
  }
  // local_bound_max
  for (size_t i = 0; i < 3; ++i) {
    output->local_bound_max[i] = input->local_bound_max[i];
  }
  // shm_name
  if (!rosidl_runtime_c__String__copy(
      &(input->shm_name), &(output->shm_name)))
  {
    return false;
  }
  return true;
}

plan_env__msg__ESDFMap *
plan_env__msg__ESDFMap__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_env__msg__ESDFMap * msg = (plan_env__msg__ESDFMap *)allocator.allocate(sizeof(plan_env__msg__ESDFMap), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(plan_env__msg__ESDFMap));
  bool success = plan_env__msg__ESDFMap__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
plan_env__msg__ESDFMap__destroy(plan_env__msg__ESDFMap * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    plan_env__msg__ESDFMap__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
plan_env__msg__ESDFMap__Sequence__init(plan_env__msg__ESDFMap__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_env__msg__ESDFMap * data = NULL;

  if (size) {
    data = (plan_env__msg__ESDFMap *)allocator.zero_allocate(size, sizeof(plan_env__msg__ESDFMap), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = plan_env__msg__ESDFMap__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        plan_env__msg__ESDFMap__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
plan_env__msg__ESDFMap__Sequence__fini(plan_env__msg__ESDFMap__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      plan_env__msg__ESDFMap__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

plan_env__msg__ESDFMap__Sequence *
plan_env__msg__ESDFMap__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  plan_env__msg__ESDFMap__Sequence * array = (plan_env__msg__ESDFMap__Sequence *)allocator.allocate(sizeof(plan_env__msg__ESDFMap__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = plan_env__msg__ESDFMap__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
plan_env__msg__ESDFMap__Sequence__destroy(plan_env__msg__ESDFMap__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    plan_env__msg__ESDFMap__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
plan_env__msg__ESDFMap__Sequence__are_equal(const plan_env__msg__ESDFMap__Sequence * lhs, const plan_env__msg__ESDFMap__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!plan_env__msg__ESDFMap__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
plan_env__msg__ESDFMap__Sequence__copy(
  const plan_env__msg__ESDFMap__Sequence * input,
  plan_env__msg__ESDFMap__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(plan_env__msg__ESDFMap);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    plan_env__msg__ESDFMap * data =
      (plan_env__msg__ESDFMap *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!plan_env__msg__ESDFMap__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          plan_env__msg__ESDFMap__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!plan_env__msg__ESDFMap__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
