// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from plan_env:msg/ESDFMap.idl
// generated code does not contain a copyright notice
#include "plan_env/msg/detail/esdf_map__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "plan_env/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "plan_env/msg/detail/esdf_map__struct.h"
#include "plan_env/msg/detail/esdf_map__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // shm_name
#include "rosidl_runtime_c/string_functions.h"  // shm_name
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_plan_env
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_plan_env
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_plan_env
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _ESDFMap__ros_msg_type = plan_env__msg__ESDFMap;

static bool _ESDFMap__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ESDFMap__ros_msg_type * ros_message = static_cast<const _ESDFMap__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: map_origin
  {
    size_t size = 3;
    auto array_ptr = ros_message->map_origin;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: resolution
  {
    cdr << ros_message->resolution;
  }

  // Field name: map_voxel_num
  {
    size_t size = 3;
    auto array_ptr = ros_message->map_voxel_num;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: local_bound_min
  {
    size_t size = 3;
    auto array_ptr = ros_message->local_bound_min;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: local_bound_max
  {
    size_t size = 3;
    auto array_ptr = ros_message->local_bound_max;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: shm_name
  {
    const rosidl_runtime_c__String * str = &ros_message->shm_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _ESDFMap__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ESDFMap__ros_msg_type * ros_message = static_cast<_ESDFMap__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: map_origin
  {
    size_t size = 3;
    auto array_ptr = ros_message->map_origin;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: resolution
  {
    cdr >> ros_message->resolution;
  }

  // Field name: map_voxel_num
  {
    size_t size = 3;
    auto array_ptr = ros_message->map_voxel_num;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: local_bound_min
  {
    size_t size = 3;
    auto array_ptr = ros_message->local_bound_min;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: local_bound_max
  {
    size_t size = 3;
    auto array_ptr = ros_message->local_bound_max;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: shm_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->shm_name.data) {
      rosidl_runtime_c__String__init(&ros_message->shm_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->shm_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'shm_name'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_plan_env
size_t get_serialized_size_plan_env__msg__ESDFMap(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ESDFMap__ros_msg_type * ros_message = static_cast<const _ESDFMap__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name map_origin
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->map_origin;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name resolution
  {
    size_t item_size = sizeof(ros_message->resolution);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name map_voxel_num
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->map_voxel_num;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name local_bound_min
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->local_bound_min;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name local_bound_max
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->local_bound_max;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name shm_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->shm_name.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _ESDFMap__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_plan_env__msg__ESDFMap(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_plan_env
size_t max_serialized_size_plan_env__msg__ESDFMap(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: map_origin
  {
    size_t array_size = 3;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: resolution
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: map_voxel_num
  {
    size_t array_size = 3;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: local_bound_min
  {
    size_t array_size = 3;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: local_bound_max
  {
    size_t array_size = 3;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: shm_name
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = plan_env__msg__ESDFMap;
    is_plain =
      (
      offsetof(DataType, shm_name) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ESDFMap__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_plan_env__msg__ESDFMap(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ESDFMap = {
  "plan_env::msg",
  "ESDFMap",
  _ESDFMap__cdr_serialize,
  _ESDFMap__cdr_deserialize,
  _ESDFMap__get_serialized_size,
  _ESDFMap__max_serialized_size
};

static rosidl_message_type_support_t _ESDFMap__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ESDFMap,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, plan_env, msg, ESDFMap)() {
  return &_ESDFMap__type_support;
}

#if defined(__cplusplus)
}
#endif
