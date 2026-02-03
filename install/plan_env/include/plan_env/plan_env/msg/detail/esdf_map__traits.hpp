// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from plan_env:msg/ESDFMap.idl
// generated code does not contain a copyright notice

#ifndef PLAN_ENV__MSG__DETAIL__ESDF_MAP__TRAITS_HPP_
#define PLAN_ENV__MSG__DETAIL__ESDF_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "plan_env/msg/detail/esdf_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace plan_env
{

namespace msg
{

inline void to_flow_style_yaml(
  const ESDFMap & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: map_origin
  {
    if (msg.map_origin.size() == 0) {
      out << "map_origin: []";
    } else {
      out << "map_origin: [";
      size_t pending_items = msg.map_origin.size();
      for (auto item : msg.map_origin) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: resolution
  {
    out << "resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.resolution, out);
    out << ", ";
  }

  // member: map_voxel_num
  {
    if (msg.map_voxel_num.size() == 0) {
      out << "map_voxel_num: []";
    } else {
      out << "map_voxel_num: [";
      size_t pending_items = msg.map_voxel_num.size();
      for (auto item : msg.map_voxel_num) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: local_bound_min
  {
    if (msg.local_bound_min.size() == 0) {
      out << "local_bound_min: []";
    } else {
      out << "local_bound_min: [";
      size_t pending_items = msg.local_bound_min.size();
      for (auto item : msg.local_bound_min) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: local_bound_max
  {
    if (msg.local_bound_max.size() == 0) {
      out << "local_bound_max: []";
    } else {
      out << "local_bound_max: [";
      size_t pending_items = msg.local_bound_max.size();
      for (auto item : msg.local_bound_max) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: shm_name
  {
    out << "shm_name: ";
    rosidl_generator_traits::value_to_yaml(msg.shm_name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ESDFMap & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: map_origin
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.map_origin.size() == 0) {
      out << "map_origin: []\n";
    } else {
      out << "map_origin:\n";
      for (auto item : msg.map_origin) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: resolution
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.resolution, out);
    out << "\n";
  }

  // member: map_voxel_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.map_voxel_num.size() == 0) {
      out << "map_voxel_num: []\n";
    } else {
      out << "map_voxel_num:\n";
      for (auto item : msg.map_voxel_num) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: local_bound_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.local_bound_min.size() == 0) {
      out << "local_bound_min: []\n";
    } else {
      out << "local_bound_min:\n";
      for (auto item : msg.local_bound_min) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: local_bound_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.local_bound_max.size() == 0) {
      out << "local_bound_max: []\n";
    } else {
      out << "local_bound_max:\n";
      for (auto item : msg.local_bound_max) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: shm_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shm_name: ";
    rosidl_generator_traits::value_to_yaml(msg.shm_name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ESDFMap & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace plan_env

namespace rosidl_generator_traits
{

[[deprecated("use plan_env::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const plan_env::msg::ESDFMap & msg,
  std::ostream & out, size_t indentation = 0)
{
  plan_env::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use plan_env::msg::to_yaml() instead")]]
inline std::string to_yaml(const plan_env::msg::ESDFMap & msg)
{
  return plan_env::msg::to_yaml(msg);
}

template<>
inline const char * data_type<plan_env::msg::ESDFMap>()
{
  return "plan_env::msg::ESDFMap";
}

template<>
inline const char * name<plan_env::msg::ESDFMap>()
{
  return "plan_env/msg/ESDFMap";
}

template<>
struct has_fixed_size<plan_env::msg::ESDFMap>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<plan_env::msg::ESDFMap>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<plan_env::msg::ESDFMap>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PLAN_ENV__MSG__DETAIL__ESDF_MAP__TRAITS_HPP_
