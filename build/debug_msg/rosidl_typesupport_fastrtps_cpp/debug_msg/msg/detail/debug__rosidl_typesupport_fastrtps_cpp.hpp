// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from debug_msg:msg/Debug.idl
// generated code does not contain a copyright notice

#ifndef DEBUG_MSG__MSG__DETAIL__DEBUG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define DEBUG_MSG__MSG__DETAIL__DEBUG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "debug_msg/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "debug_msg/msg/detail/debug__struct.hpp"

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

#include "fastcdr/Cdr.h"

namespace debug_msg
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_debug_msg
cdr_serialize(
  const debug_msg::msg::Debug & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_debug_msg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  debug_msg::msg::Debug & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_debug_msg
get_serialized_size(
  const debug_msg::msg::Debug & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_debug_msg
max_serialized_size_Debug(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace debug_msg

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_debug_msg
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, debug_msg, msg, Debug)();

#ifdef __cplusplus
}
#endif

#endif  // DEBUG_MSG__MSG__DETAIL__DEBUG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
