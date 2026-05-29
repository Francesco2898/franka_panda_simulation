// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from debug_msg:msg/Debug.idl
// generated code does not contain a copyright notice

#ifndef DEBUG_MSG__MSG__DETAIL__DEBUG__STRUCT_HPP_
#define DEBUG_MSG__MSG__DETAIL__DEBUG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__debug_msg__msg__Debug __attribute__((deprecated))
#else
# define DEPRECATED__debug_msg__msg__Debug __declspec(deprecated)
#endif

namespace debug_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Debug_
{
  using Type = Debug_<ContainerAllocator>;

  explicit Debug_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 7>::iterator, double>(this->commanded_torque.begin(), this->commanded_torque.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->stiffness_torque.begin(), this->stiffness_torque.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->damping_torque.begin(), this->damping_torque.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->coriolis_torque.begin(), this->coriolis_torque.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->nullspace_torque.begin(), this->nullspace_torque.end(), 0.0);
      std::fill<typename std::array<double, 6>::iterator, double>(this->impedance_force.begin(), this->impedance_force.end(), 0.0);
    }
  }

  explicit Debug_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : commanded_torque(_alloc),
    stiffness_torque(_alloc),
    damping_torque(_alloc),
    coriolis_torque(_alloc),
    nullspace_torque(_alloc),
    impedance_force(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 7>::iterator, double>(this->commanded_torque.begin(), this->commanded_torque.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->stiffness_torque.begin(), this->stiffness_torque.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->damping_torque.begin(), this->damping_torque.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->coriolis_torque.begin(), this->coriolis_torque.end(), 0.0);
      std::fill<typename std::array<double, 7>::iterator, double>(this->nullspace_torque.begin(), this->nullspace_torque.end(), 0.0);
      std::fill<typename std::array<double, 6>::iterator, double>(this->impedance_force.begin(), this->impedance_force.end(), 0.0);
    }
  }

  // field types and members
  using _commanded_torque_type =
    std::array<double, 7>;
  _commanded_torque_type commanded_torque;
  using _stiffness_torque_type =
    std::array<double, 7>;
  _stiffness_torque_type stiffness_torque;
  using _damping_torque_type =
    std::array<double, 7>;
  _damping_torque_type damping_torque;
  using _coriolis_torque_type =
    std::array<double, 7>;
  _coriolis_torque_type coriolis_torque;
  using _nullspace_torque_type =
    std::array<double, 7>;
  _nullspace_torque_type nullspace_torque;
  using _impedance_force_type =
    std::array<double, 6>;
  _impedance_force_type impedance_force;

  // setters for named parameter idiom
  Type & set__commanded_torque(
    const std::array<double, 7> & _arg)
  {
    this->commanded_torque = _arg;
    return *this;
  }
  Type & set__stiffness_torque(
    const std::array<double, 7> & _arg)
  {
    this->stiffness_torque = _arg;
    return *this;
  }
  Type & set__damping_torque(
    const std::array<double, 7> & _arg)
  {
    this->damping_torque = _arg;
    return *this;
  }
  Type & set__coriolis_torque(
    const std::array<double, 7> & _arg)
  {
    this->coriolis_torque = _arg;
    return *this;
  }
  Type & set__nullspace_torque(
    const std::array<double, 7> & _arg)
  {
    this->nullspace_torque = _arg;
    return *this;
  }
  Type & set__impedance_force(
    const std::array<double, 6> & _arg)
  {
    this->impedance_force = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    debug_msg::msg::Debug_<ContainerAllocator> *;
  using ConstRawPtr =
    const debug_msg::msg::Debug_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<debug_msg::msg::Debug_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<debug_msg::msg::Debug_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      debug_msg::msg::Debug_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<debug_msg::msg::Debug_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      debug_msg::msg::Debug_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<debug_msg::msg::Debug_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<debug_msg::msg::Debug_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<debug_msg::msg::Debug_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__debug_msg__msg__Debug
    std::shared_ptr<debug_msg::msg::Debug_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__debug_msg__msg__Debug
    std::shared_ptr<debug_msg::msg::Debug_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Debug_ & other) const
  {
    if (this->commanded_torque != other.commanded_torque) {
      return false;
    }
    if (this->stiffness_torque != other.stiffness_torque) {
      return false;
    }
    if (this->damping_torque != other.damping_torque) {
      return false;
    }
    if (this->coriolis_torque != other.coriolis_torque) {
      return false;
    }
    if (this->nullspace_torque != other.nullspace_torque) {
      return false;
    }
    if (this->impedance_force != other.impedance_force) {
      return false;
    }
    return true;
  }
  bool operator!=(const Debug_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Debug_

// alias to use template instance with default allocator
using Debug =
  debug_msg::msg::Debug_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace debug_msg

#endif  // DEBUG_MSG__MSG__DETAIL__DEBUG__STRUCT_HPP_
