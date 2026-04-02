// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages:msg/ForceEvent.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__FORCE_EVENT__BUILDER_HPP_
#define MESSAGES__MSG__DETAIL__FORCE_EVENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "messages/msg/detail/force_event__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace messages
{

namespace msg
{

namespace builder
{

class Init_ForceEvent_timestamp_2_end
{
public:
  explicit Init_ForceEvent_timestamp_2_end(::messages::msg::ForceEvent & msg)
  : msg_(msg)
  {}
  ::messages::msg::ForceEvent timestamp_2_end(::messages::msg::ForceEvent::_timestamp_2_end_type arg)
  {
    msg_.timestamp_2_end = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages::msg::ForceEvent msg_;
};

class Init_ForceEvent_timestamp_2_start
{
public:
  explicit Init_ForceEvent_timestamp_2_start(::messages::msg::ForceEvent & msg)
  : msg_(msg)
  {}
  Init_ForceEvent_timestamp_2_end timestamp_2_start(::messages::msg::ForceEvent::_timestamp_2_start_type arg)
  {
    msg_.timestamp_2_start = std::move(arg);
    return Init_ForceEvent_timestamp_2_end(msg_);
  }

private:
  ::messages::msg::ForceEvent msg_;
};

class Init_ForceEvent_force_magnitude_2
{
public:
  explicit Init_ForceEvent_force_magnitude_2(::messages::msg::ForceEvent & msg)
  : msg_(msg)
  {}
  Init_ForceEvent_timestamp_2_start force_magnitude_2(::messages::msg::ForceEvent::_force_magnitude_2_type arg)
  {
    msg_.force_magnitude_2 = std::move(arg);
    return Init_ForceEvent_timestamp_2_start(msg_);
  }

private:
  ::messages::msg::ForceEvent msg_;
};

class Init_ForceEvent_timestamp_1_end
{
public:
  explicit Init_ForceEvent_timestamp_1_end(::messages::msg::ForceEvent & msg)
  : msg_(msg)
  {}
  Init_ForceEvent_force_magnitude_2 timestamp_1_end(::messages::msg::ForceEvent::_timestamp_1_end_type arg)
  {
    msg_.timestamp_1_end = std::move(arg);
    return Init_ForceEvent_force_magnitude_2(msg_);
  }

private:
  ::messages::msg::ForceEvent msg_;
};

class Init_ForceEvent_timestamp_1_start
{
public:
  explicit Init_ForceEvent_timestamp_1_start(::messages::msg::ForceEvent & msg)
  : msg_(msg)
  {}
  Init_ForceEvent_timestamp_1_end timestamp_1_start(::messages::msg::ForceEvent::_timestamp_1_start_type arg)
  {
    msg_.timestamp_1_start = std::move(arg);
    return Init_ForceEvent_timestamp_1_end(msg_);
  }

private:
  ::messages::msg::ForceEvent msg_;
};

class Init_ForceEvent_force_magnitude_1
{
public:
  Init_ForceEvent_force_magnitude_1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ForceEvent_timestamp_1_start force_magnitude_1(::messages::msg::ForceEvent::_force_magnitude_1_type arg)
  {
    msg_.force_magnitude_1 = std::move(arg);
    return Init_ForceEvent_timestamp_1_start(msg_);
  }

private:
  ::messages::msg::ForceEvent msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages::msg::ForceEvent>()
{
  return messages::msg::builder::Init_ForceEvent_force_magnitude_1();
}

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__FORCE_EVENT__BUILDER_HPP_
