// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from debug_msg:msg/Debug.idl
// generated code does not contain a copyright notice
#include "debug_msg/msg/detail/debug__rosidl_typesupport_fastrtps_cpp.hpp"
#include "debug_msg/msg/detail/debug__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

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
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: commanded_torque
  {
    cdr << ros_message.commanded_torque;
  }
  // Member: stiffness_torque
  {
    cdr << ros_message.stiffness_torque;
  }
  // Member: damping_torque
  {
    cdr << ros_message.damping_torque;
  }
  // Member: coriolis_torque
  {
    cdr << ros_message.coriolis_torque;
  }
  // Member: nullspace_torque
  {
    cdr << ros_message.nullspace_torque;
  }
  // Member: impedance_force
  {
    cdr << ros_message.impedance_force;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_debug_msg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  debug_msg::msg::Debug & ros_message)
{
  // Member: commanded_torque
  {
    cdr >> ros_message.commanded_torque;
  }

  // Member: stiffness_torque
  {
    cdr >> ros_message.stiffness_torque;
  }

  // Member: damping_torque
  {
    cdr >> ros_message.damping_torque;
  }

  // Member: coriolis_torque
  {
    cdr >> ros_message.coriolis_torque;
  }

  // Member: nullspace_torque
  {
    cdr >> ros_message.nullspace_torque;
  }

  // Member: impedance_force
  {
    cdr >> ros_message.impedance_force;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_debug_msg
get_serialized_size(
  const debug_msg::msg::Debug & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: commanded_torque
  {
    size_t array_size = 7;
    size_t item_size = sizeof(ros_message.commanded_torque[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: stiffness_torque
  {
    size_t array_size = 7;
    size_t item_size = sizeof(ros_message.stiffness_torque[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: damping_torque
  {
    size_t array_size = 7;
    size_t item_size = sizeof(ros_message.damping_torque[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: coriolis_torque
  {
    size_t array_size = 7;
    size_t item_size = sizeof(ros_message.coriolis_torque[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: nullspace_torque
  {
    size_t array_size = 7;
    size_t item_size = sizeof(ros_message.nullspace_torque[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: impedance_force
  {
    size_t array_size = 6;
    size_t item_size = sizeof(ros_message.impedance_force[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_debug_msg
max_serialized_size_Debug(
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


  // Member: commanded_torque
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: stiffness_torque
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: damping_torque
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: coriolis_torque
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: nullspace_torque
  {
    size_t array_size = 7;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: impedance_force
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = debug_msg::msg::Debug;
    is_plain =
      (
      offsetof(DataType, impedance_force) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Debug__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const debug_msg::msg::Debug *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Debug__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<debug_msg::msg::Debug *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Debug__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const debug_msg::msg::Debug *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Debug__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Debug(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Debug__callbacks = {
  "debug_msg::msg",
  "Debug",
  _Debug__cdr_serialize,
  _Debug__cdr_deserialize,
  _Debug__get_serialized_size,
  _Debug__max_serialized_size
};

static rosidl_message_type_support_t _Debug__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Debug__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace debug_msg

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_debug_msg
const rosidl_message_type_support_t *
get_message_type_support_handle<debug_msg::msg::Debug>()
{
  return &debug_msg::msg::typesupport_fastrtps_cpp::_Debug__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, debug_msg, msg, Debug)() {
  return &debug_msg::msg::typesupport_fastrtps_cpp::_Debug__handle;
}

#ifdef __cplusplus
}
#endif
