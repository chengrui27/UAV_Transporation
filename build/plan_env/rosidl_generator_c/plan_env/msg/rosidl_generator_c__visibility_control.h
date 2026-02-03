// generated from rosidl_generator_c/resource/rosidl_generator_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef PLAN_ENV__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
#define PLAN_ENV__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_C_EXPORT_plan_env __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_C_IMPORT_plan_env __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_C_EXPORT_plan_env __declspec(dllexport)
    #define ROSIDL_GENERATOR_C_IMPORT_plan_env __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_C_BUILDING_DLL_plan_env
    #define ROSIDL_GENERATOR_C_PUBLIC_plan_env ROSIDL_GENERATOR_C_EXPORT_plan_env
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_plan_env ROSIDL_GENERATOR_C_IMPORT_plan_env
  #endif
#else
  #define ROSIDL_GENERATOR_C_EXPORT_plan_env __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_C_IMPORT_plan_env
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_C_PUBLIC_plan_env __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_plan_env
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // PLAN_ENV__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
