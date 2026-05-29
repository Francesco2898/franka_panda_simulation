// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from debug_msg:msg/Debug.idl
// generated code does not contain a copyright notice

#ifndef DEBUG_MSG__MSG__DETAIL__DEBUG__TRAITS_HPP_
#define DEBUG_MSG__MSG__DETAIL__DEBUG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "debug_msg/msg/detail/debug__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace debug_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Debug & msg,
  std::ostream & out)
{
  out << "{";
  // member: commanded_torque
  {
    if (msg.commanded_torque.size() == 0) {
      out << "commanded_torque: []";
    } else {
      out << "commanded_torque: [";
      size_t pending_items = msg.commanded_torque.size();
      for (auto item : msg.commanded_torque) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: stiffness_torque
  {
    if (msg.stiffness_torque.size() == 0) {
      out << "stiffness_torque: []";
    } else {
      out << "stiffness_torque: [";
      size_t pending_items = msg.stiffness_torque.size();
      for (auto item : msg.stiffness_torque) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: damping_torque
  {
    if (msg.damping_torque.size() == 0) {
      out << "damping_torque: []";
    } else {
      out << "damping_torque: [";
      size_t pending_items = msg.damping_torque.size();
      for (auto item : msg.damping_torque) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: coriolis_torque
  {
    if (msg.coriolis_torque.size() == 0) {
      out << "coriolis_torque: []";
    } else {
      out << "coriolis_torque: [";
      size_t pending_items = msg.coriolis_torque.size();
      for (auto item : msg.coriolis_torque) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: nullspace_torque
  {
    if (msg.nullspace_torque.size() == 0) {
      out << "nullspace_torque: []";
    } else {
      out << "nullspace_torque: [";
      size_t pending_items = msg.nullspace_torque.size();
      for (auto item : msg.nullspace_torque) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: impedance_force
  {
    if (msg.impedance_force.size() == 0) {
      out << "impedance_force: []";
    } else {
      out << "impedance_force: [";
      size_t pending_items = msg.impedance_force.size();
      for (auto item : msg.impedance_force) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Debug & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: commanded_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.commanded_torque.size() == 0) {
      out << "commanded_torque: []\n";
    } else {
      out << "commanded_torque:\n";
      for (auto item : msg.commanded_torque) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: stiffness_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.stiffness_torque.size() == 0) {
      out << "stiffness_torque: []\n";
    } else {
      out << "stiffness_torque:\n";
      for (auto item : msg.stiffness_torque) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: damping_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.damping_torque.size() == 0) {
      out << "damping_torque: []\n";
    } else {
      out << "damping_torque:\n";
      for (auto item : msg.damping_torque) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: coriolis_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.coriolis_torque.size() == 0) {
      out << "coriolis_torque: []\n";
    } else {
      out << "coriolis_torque:\n";
      for (auto item : msg.coriolis_torque) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: nullspace_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.nullspace_torque.size() == 0) {
      out << "nullspace_torque: []\n";
    } else {
      out << "nullspace_torque:\n";
      for (auto item : msg.nullspace_torque) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: impedance_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.impedance_force.size() == 0) {
      out << "impedance_force: []\n";
    } else {
      out << "impedance_force:\n";
      for (auto item : msg.impedance_force) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Debug & msg, bool use_flow_style = false)
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

}  // namespace debug_msg

namespace rosidl_generator_traits
{

[[deprecated("use debug_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const debug_msg::msg::Debug & msg,
  std::ostream & out, size_t indentation = 0)
{
  debug_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use debug_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const debug_msg::msg::Debug & msg)
{
  return debug_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<debug_msg::msg::Debug>()
{
  return "debug_msg::msg::Debug";
}

template<>
inline const char * name<debug_msg::msg::Debug>()
{
  return "debug_msg/msg/Debug";
}

template<>
struct has_fixed_size<debug_msg::msg::Debug>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<debug_msg::msg::Debug>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<debug_msg::msg::Debug>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DEBUG_MSG__MSG__DETAIL__DEBUG__TRAITS_HPP_
