// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from debug_msg:msg/Debug.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "debug_msg/msg/detail/debug__rosidl_typesupport_introspection_c.h"
#include "debug_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "debug_msg/msg/detail/debug__functions.h"
#include "debug_msg/msg/detail/debug__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void debug_msg__msg__Debug__rosidl_typesupport_introspection_c__Debug_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  debug_msg__msg__Debug__init(message_memory);
}

void debug_msg__msg__Debug__rosidl_typesupport_introspection_c__Debug_fini_function(void * message_memory)
{
  debug_msg__msg__Debug__fini(message_memory);
}

size_t debug_msg__msg__Debug__rosidl_typesupport_introspection_c__size_function__Debug__commanded_torque(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__commanded_torque(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__commanded_torque(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void debug_msg__msg__Debug__rosidl_typesupport_introspection_c__fetch_function__Debug__commanded_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__commanded_torque(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void debug_msg__msg__Debug__rosidl_typesupport_introspection_c__assign_function__Debug__commanded_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__commanded_torque(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t debug_msg__msg__Debug__rosidl_typesupport_introspection_c__size_function__Debug__stiffness_torque(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__stiffness_torque(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__stiffness_torque(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void debug_msg__msg__Debug__rosidl_typesupport_introspection_c__fetch_function__Debug__stiffness_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__stiffness_torque(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void debug_msg__msg__Debug__rosidl_typesupport_introspection_c__assign_function__Debug__stiffness_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__stiffness_torque(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t debug_msg__msg__Debug__rosidl_typesupport_introspection_c__size_function__Debug__damping_torque(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__damping_torque(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__damping_torque(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void debug_msg__msg__Debug__rosidl_typesupport_introspection_c__fetch_function__Debug__damping_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__damping_torque(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void debug_msg__msg__Debug__rosidl_typesupport_introspection_c__assign_function__Debug__damping_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__damping_torque(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t debug_msg__msg__Debug__rosidl_typesupport_introspection_c__size_function__Debug__coriolis_torque(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__coriolis_torque(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__coriolis_torque(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void debug_msg__msg__Debug__rosidl_typesupport_introspection_c__fetch_function__Debug__coriolis_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__coriolis_torque(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void debug_msg__msg__Debug__rosidl_typesupport_introspection_c__assign_function__Debug__coriolis_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__coriolis_torque(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t debug_msg__msg__Debug__rosidl_typesupport_introspection_c__size_function__Debug__nullspace_torque(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__nullspace_torque(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__nullspace_torque(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void debug_msg__msg__Debug__rosidl_typesupport_introspection_c__fetch_function__Debug__nullspace_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__nullspace_torque(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void debug_msg__msg__Debug__rosidl_typesupport_introspection_c__assign_function__Debug__nullspace_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__nullspace_torque(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t debug_msg__msg__Debug__rosidl_typesupport_introspection_c__size_function__Debug__impedance_force(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__impedance_force(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__impedance_force(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void debug_msg__msg__Debug__rosidl_typesupport_introspection_c__fetch_function__Debug__impedance_force(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__impedance_force(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void debug_msg__msg__Debug__rosidl_typesupport_introspection_c__assign_function__Debug__impedance_force(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__impedance_force(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember debug_msg__msg__Debug__rosidl_typesupport_introspection_c__Debug_message_member_array[6] = {
  {
    "commanded_torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(debug_msg__msg__Debug, commanded_torque),  // bytes offset in struct
    NULL,  // default value
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__size_function__Debug__commanded_torque,  // size() function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__commanded_torque,  // get_const(index) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__commanded_torque,  // get(index) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__fetch_function__Debug__commanded_torque,  // fetch(index, &value) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__assign_function__Debug__commanded_torque,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stiffness_torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(debug_msg__msg__Debug, stiffness_torque),  // bytes offset in struct
    NULL,  // default value
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__size_function__Debug__stiffness_torque,  // size() function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__stiffness_torque,  // get_const(index) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__stiffness_torque,  // get(index) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__fetch_function__Debug__stiffness_torque,  // fetch(index, &value) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__assign_function__Debug__stiffness_torque,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "damping_torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(debug_msg__msg__Debug, damping_torque),  // bytes offset in struct
    NULL,  // default value
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__size_function__Debug__damping_torque,  // size() function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__damping_torque,  // get_const(index) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__damping_torque,  // get(index) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__fetch_function__Debug__damping_torque,  // fetch(index, &value) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__assign_function__Debug__damping_torque,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "coriolis_torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(debug_msg__msg__Debug, coriolis_torque),  // bytes offset in struct
    NULL,  // default value
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__size_function__Debug__coriolis_torque,  // size() function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__coriolis_torque,  // get_const(index) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__coriolis_torque,  // get(index) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__fetch_function__Debug__coriolis_torque,  // fetch(index, &value) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__assign_function__Debug__coriolis_torque,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "nullspace_torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(debug_msg__msg__Debug, nullspace_torque),  // bytes offset in struct
    NULL,  // default value
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__size_function__Debug__nullspace_torque,  // size() function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__nullspace_torque,  // get_const(index) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__nullspace_torque,  // get(index) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__fetch_function__Debug__nullspace_torque,  // fetch(index, &value) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__assign_function__Debug__nullspace_torque,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "impedance_force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(debug_msg__msg__Debug, impedance_force),  // bytes offset in struct
    NULL,  // default value
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__size_function__Debug__impedance_force,  // size() function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_const_function__Debug__impedance_force,  // get_const(index) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__get_function__Debug__impedance_force,  // get(index) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__fetch_function__Debug__impedance_force,  // fetch(index, &value) function pointer
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__assign_function__Debug__impedance_force,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers debug_msg__msg__Debug__rosidl_typesupport_introspection_c__Debug_message_members = {
  "debug_msg__msg",  // message namespace
  "Debug",  // message name
  6,  // number of fields
  sizeof(debug_msg__msg__Debug),
  debug_msg__msg__Debug__rosidl_typesupport_introspection_c__Debug_message_member_array,  // message members
  debug_msg__msg__Debug__rosidl_typesupport_introspection_c__Debug_init_function,  // function to initialize message memory (memory has to be allocated)
  debug_msg__msg__Debug__rosidl_typesupport_introspection_c__Debug_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t debug_msg__msg__Debug__rosidl_typesupport_introspection_c__Debug_message_type_support_handle = {
  0,
  &debug_msg__msg__Debug__rosidl_typesupport_introspection_c__Debug_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_debug_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, debug_msg, msg, Debug)() {
  if (!debug_msg__msg__Debug__rosidl_typesupport_introspection_c__Debug_message_type_support_handle.typesupport_identifier) {
    debug_msg__msg__Debug__rosidl_typesupport_introspection_c__Debug_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &debug_msg__msg__Debug__rosidl_typesupport_introspection_c__Debug_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
