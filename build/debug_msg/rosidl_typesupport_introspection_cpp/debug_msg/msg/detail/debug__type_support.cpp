// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from debug_msg:msg/Debug.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "debug_msg/msg/detail/debug__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace debug_msg
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Debug_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) debug_msg::msg::Debug(_init);
}

void Debug_fini_function(void * message_memory)
{
  auto typed_message = static_cast<debug_msg::msg::Debug *>(message_memory);
  typed_message->~Debug();
}

size_t size_function__Debug__commanded_torque(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__Debug__commanded_torque(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__Debug__commanded_torque(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__Debug__commanded_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Debug__commanded_torque(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Debug__commanded_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Debug__commanded_torque(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__Debug__stiffness_torque(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__Debug__stiffness_torque(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__Debug__stiffness_torque(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__Debug__stiffness_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Debug__stiffness_torque(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Debug__stiffness_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Debug__stiffness_torque(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__Debug__damping_torque(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__Debug__damping_torque(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__Debug__damping_torque(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__Debug__damping_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Debug__damping_torque(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Debug__damping_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Debug__damping_torque(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__Debug__coriolis_torque(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__Debug__coriolis_torque(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__Debug__coriolis_torque(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__Debug__coriolis_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Debug__coriolis_torque(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Debug__coriolis_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Debug__coriolis_torque(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__Debug__nullspace_torque(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__Debug__nullspace_torque(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__Debug__nullspace_torque(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__Debug__nullspace_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Debug__nullspace_torque(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Debug__nullspace_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Debug__nullspace_torque(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__Debug__impedance_force(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__Debug__impedance_force(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__Debug__impedance_force(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__Debug__impedance_force(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Debug__impedance_force(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Debug__impedance_force(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Debug__impedance_force(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Debug_message_member_array[6] = {
  {
    "commanded_torque",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(debug_msg::msg::Debug, commanded_torque),  // bytes offset in struct
    nullptr,  // default value
    size_function__Debug__commanded_torque,  // size() function pointer
    get_const_function__Debug__commanded_torque,  // get_const(index) function pointer
    get_function__Debug__commanded_torque,  // get(index) function pointer
    fetch_function__Debug__commanded_torque,  // fetch(index, &value) function pointer
    assign_function__Debug__commanded_torque,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "stiffness_torque",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(debug_msg::msg::Debug, stiffness_torque),  // bytes offset in struct
    nullptr,  // default value
    size_function__Debug__stiffness_torque,  // size() function pointer
    get_const_function__Debug__stiffness_torque,  // get_const(index) function pointer
    get_function__Debug__stiffness_torque,  // get(index) function pointer
    fetch_function__Debug__stiffness_torque,  // fetch(index, &value) function pointer
    assign_function__Debug__stiffness_torque,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "damping_torque",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(debug_msg::msg::Debug, damping_torque),  // bytes offset in struct
    nullptr,  // default value
    size_function__Debug__damping_torque,  // size() function pointer
    get_const_function__Debug__damping_torque,  // get_const(index) function pointer
    get_function__Debug__damping_torque,  // get(index) function pointer
    fetch_function__Debug__damping_torque,  // fetch(index, &value) function pointer
    assign_function__Debug__damping_torque,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "coriolis_torque",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(debug_msg::msg::Debug, coriolis_torque),  // bytes offset in struct
    nullptr,  // default value
    size_function__Debug__coriolis_torque,  // size() function pointer
    get_const_function__Debug__coriolis_torque,  // get_const(index) function pointer
    get_function__Debug__coriolis_torque,  // get(index) function pointer
    fetch_function__Debug__coriolis_torque,  // fetch(index, &value) function pointer
    assign_function__Debug__coriolis_torque,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "nullspace_torque",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(debug_msg::msg::Debug, nullspace_torque),  // bytes offset in struct
    nullptr,  // default value
    size_function__Debug__nullspace_torque,  // size() function pointer
    get_const_function__Debug__nullspace_torque,  // get_const(index) function pointer
    get_function__Debug__nullspace_torque,  // get(index) function pointer
    fetch_function__Debug__nullspace_torque,  // fetch(index, &value) function pointer
    assign_function__Debug__nullspace_torque,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "impedance_force",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(debug_msg::msg::Debug, impedance_force),  // bytes offset in struct
    nullptr,  // default value
    size_function__Debug__impedance_force,  // size() function pointer
    get_const_function__Debug__impedance_force,  // get_const(index) function pointer
    get_function__Debug__impedance_force,  // get(index) function pointer
    fetch_function__Debug__impedance_force,  // fetch(index, &value) function pointer
    assign_function__Debug__impedance_force,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Debug_message_members = {
  "debug_msg::msg",  // message namespace
  "Debug",  // message name
  6,  // number of fields
  sizeof(debug_msg::msg::Debug),
  Debug_message_member_array,  // message members
  Debug_init_function,  // function to initialize message memory (memory has to be allocated)
  Debug_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Debug_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Debug_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace debug_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<debug_msg::msg::Debug>()
{
  return &::debug_msg::msg::rosidl_typesupport_introspection_cpp::Debug_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, debug_msg, msg, Debug)() {
  return &::debug_msg::msg::rosidl_typesupport_introspection_cpp::Debug_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
