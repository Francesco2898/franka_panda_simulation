// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from debug_msg:msg/Debug.idl
// generated code does not contain a copyright notice

#ifndef DEBUG_MSG__MSG__DETAIL__DEBUG__BUILDER_HPP_
#define DEBUG_MSG__MSG__DETAIL__DEBUG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "debug_msg/msg/detail/debug__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace debug_msg
{

namespace msg
{

namespace builder
{

class Init_Debug_impedance_force
{
public:
  explicit Init_Debug_impedance_force(::debug_msg::msg::Debug & msg)
  : msg_(msg)
  {}
  ::debug_msg::msg::Debug impedance_force(::debug_msg::msg::Debug::_impedance_force_type arg)
  {
    msg_.impedance_force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::debug_msg::msg::Debug msg_;
};

class Init_Debug_nullspace_torque
{
public:
  explicit Init_Debug_nullspace_torque(::debug_msg::msg::Debug & msg)
  : msg_(msg)
  {}
  Init_Debug_impedance_force nullspace_torque(::debug_msg::msg::Debug::_nullspace_torque_type arg)
  {
    msg_.nullspace_torque = std::move(arg);
    return Init_Debug_impedance_force(msg_);
  }

private:
  ::debug_msg::msg::Debug msg_;
};

class Init_Debug_coriolis_torque
{
public:
  explicit Init_Debug_coriolis_torque(::debug_msg::msg::Debug & msg)
  : msg_(msg)
  {}
  Init_Debug_nullspace_torque coriolis_torque(::debug_msg::msg::Debug::_coriolis_torque_type arg)
  {
    msg_.coriolis_torque = std::move(arg);
    return Init_Debug_nullspace_torque(msg_);
  }

private:
  ::debug_msg::msg::Debug msg_;
};

class Init_Debug_damping_torque
{
public:
  explicit Init_Debug_damping_torque(::debug_msg::msg::Debug & msg)
  : msg_(msg)
  {}
  Init_Debug_coriolis_torque damping_torque(::debug_msg::msg::Debug::_damping_torque_type arg)
  {
    msg_.damping_torque = std::move(arg);
    return Init_Debug_coriolis_torque(msg_);
  }

private:
  ::debug_msg::msg::Debug msg_;
};

class Init_Debug_stiffness_torque
{
public:
  explicit Init_Debug_stiffness_torque(::debug_msg::msg::Debug & msg)
  : msg_(msg)
  {}
  Init_Debug_damping_torque stiffness_torque(::debug_msg::msg::Debug::_stiffness_torque_type arg)
  {
    msg_.stiffness_torque = std::move(arg);
    return Init_Debug_damping_torque(msg_);
  }

private:
  ::debug_msg::msg::Debug msg_;
};

class Init_Debug_commanded_torque
{
public:
  Init_Debug_commanded_torque()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Debug_stiffness_torque commanded_torque(::debug_msg::msg::Debug::_commanded_torque_type arg)
  {
    msg_.commanded_torque = std::move(arg);
    return Init_Debug_stiffness_torque(msg_);
  }

private:
  ::debug_msg::msg::Debug msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::debug_msg::msg::Debug>()
{
  return debug_msg::msg::builder::Init_Debug_commanded_torque();
}

}  // namespace debug_msg

#endif  // DEBUG_MSG__MSG__DETAIL__DEBUG__BUILDER_HPP_
