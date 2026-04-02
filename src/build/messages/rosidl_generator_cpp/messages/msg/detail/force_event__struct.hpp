// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from messages:msg/ForceEvent.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__FORCE_EVENT__STRUCT_HPP_
#define MESSAGES__MSG__DETAIL__FORCE_EVENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'timestamp_1_start'
// Member 'timestamp_1_end'
// Member 'timestamp_2_start'
// Member 'timestamp_2_end'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__messages__msg__ForceEvent __attribute__((deprecated))
#else
# define DEPRECATED__messages__msg__ForceEvent __declspec(deprecated)
#endif

namespace messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ForceEvent_
{
  using Type = ForceEvent_<ContainerAllocator>;

  explicit ForceEvent_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp_1_start(_init),
    timestamp_1_end(_init),
    timestamp_2_start(_init),
    timestamp_2_end(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->force_magnitude_1 = 0.0;
      this->force_magnitude_2 = 0.0;
    }
  }

  explicit ForceEvent_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp_1_start(_alloc, _init),
    timestamp_1_end(_alloc, _init),
    timestamp_2_start(_alloc, _init),
    timestamp_2_end(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->force_magnitude_1 = 0.0;
      this->force_magnitude_2 = 0.0;
    }
  }

  // field types and members
  using _force_magnitude_1_type =
    double;
  _force_magnitude_1_type force_magnitude_1;
  using _timestamp_1_start_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_1_start_type timestamp_1_start;
  using _timestamp_1_end_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_1_end_type timestamp_1_end;
  using _force_magnitude_2_type =
    double;
  _force_magnitude_2_type force_magnitude_2;
  using _timestamp_2_start_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_2_start_type timestamp_2_start;
  using _timestamp_2_end_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_2_end_type timestamp_2_end;

  // setters for named parameter idiom
  Type & set__force_magnitude_1(
    const double & _arg)
  {
    this->force_magnitude_1 = _arg;
    return *this;
  }
  Type & set__timestamp_1_start(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp_1_start = _arg;
    return *this;
  }
  Type & set__timestamp_1_end(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp_1_end = _arg;
    return *this;
  }
  Type & set__force_magnitude_2(
    const double & _arg)
  {
    this->force_magnitude_2 = _arg;
    return *this;
  }
  Type & set__timestamp_2_start(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp_2_start = _arg;
    return *this;
  }
  Type & set__timestamp_2_end(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp_2_end = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    messages::msg::ForceEvent_<ContainerAllocator> *;
  using ConstRawPtr =
    const messages::msg::ForceEvent_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<messages::msg::ForceEvent_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<messages::msg::ForceEvent_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      messages::msg::ForceEvent_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<messages::msg::ForceEvent_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      messages::msg::ForceEvent_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<messages::msg::ForceEvent_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<messages::msg::ForceEvent_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<messages::msg::ForceEvent_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__messages__msg__ForceEvent
    std::shared_ptr<messages::msg::ForceEvent_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__messages__msg__ForceEvent
    std::shared_ptr<messages::msg::ForceEvent_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ForceEvent_ & other) const
  {
    if (this->force_magnitude_1 != other.force_magnitude_1) {
      return false;
    }
    if (this->timestamp_1_start != other.timestamp_1_start) {
      return false;
    }
    if (this->timestamp_1_end != other.timestamp_1_end) {
      return false;
    }
    if (this->force_magnitude_2 != other.force_magnitude_2) {
      return false;
    }
    if (this->timestamp_2_start != other.timestamp_2_start) {
      return false;
    }
    if (this->timestamp_2_end != other.timestamp_2_end) {
      return false;
    }
    return true;
  }
  bool operator!=(const ForceEvent_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ForceEvent_

// alias to use template instance with default allocator
using ForceEvent =
  messages::msg::ForceEvent_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__FORCE_EVENT__STRUCT_HPP_
