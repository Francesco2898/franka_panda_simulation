// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from debug_msg:msg/Debug.idl
// generated code does not contain a copyright notice

#ifndef DEBUG_MSG__MSG__DETAIL__DEBUG__STRUCT_H_
#define DEBUG_MSG__MSG__DETAIL__DEBUG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Debug in the package debug_msg.
typedef struct debug_msg__msg__Debug
{
  double commanded_torque[7];
  double stiffness_torque[7];
  double damping_torque[7];
  double coriolis_torque[7];
  double nullspace_torque[7];
  double impedance_force[6];
} debug_msg__msg__Debug;

// Struct for a sequence of debug_msg__msg__Debug.
typedef struct debug_msg__msg__Debug__Sequence
{
  debug_msg__msg__Debug * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} debug_msg__msg__Debug__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEBUG_MSG__MSG__DETAIL__DEBUG__STRUCT_H_
