// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages:msg/ForceEvent.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__FORCE_EVENT__TRAITS_HPP_
#define MESSAGES__MSG__DETAIL__FORCE_EVENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "messages/msg/detail/force_event__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp_1_start'
// Member 'timestamp_1_end'
// Member 'timestamp_2_start'
// Member 'timestamp_2_end'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace messages
{

namespace msg
{

inline void to_flow_style_yaml(
  const ForceEvent & msg,
  std::ostream & out)
{
  out << "{";
  // member: force_magnitude_1
  {
    out << "force_magnitude_1: ";
    rosidl_generator_traits::value_to_yaml(msg.force_magnitude_1, out);
    out << ", ";
  }

  // member: timestamp_1_start
  {
    out << "timestamp_1_start: ";
    to_flow_style_yaml(msg.timestamp_1_start, out);
    out << ", ";
  }

  // member: timestamp_1_end
  {
    out << "timestamp_1_end: ";
    to_flow_style_yaml(msg.timestamp_1_end, out);
    out << ", ";
  }

  // member: force_magnitude_2
  {
    out << "force_magnitude_2: ";
    rosidl_generator_traits::value_to_yaml(msg.force_magnitude_2, out);
    out << ", ";
  }

  // member: timestamp_2_start
  {
    out << "timestamp_2_start: ";
    to_flow_style_yaml(msg.timestamp_2_start, out);
    out << ", ";
  }

  // member: timestamp_2_end
  {
    out << "timestamp_2_end: ";
    to_flow_style_yaml(msg.timestamp_2_end, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ForceEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: force_magnitude_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "force_magnitude_1: ";
    rosidl_generator_traits::value_to_yaml(msg.force_magnitude_1, out);
    out << "\n";
  }

  // member: timestamp_1_start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp_1_start:\n";
    to_block_style_yaml(msg.timestamp_1_start, out, indentation + 2);
  }

  // member: timestamp_1_end
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp_1_end:\n";
    to_block_style_yaml(msg.timestamp_1_end, out, indentation + 2);
  }

  // member: force_magnitude_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "force_magnitude_2: ";
    rosidl_generator_traits::value_to_yaml(msg.force_magnitude_2, out);
    out << "\n";
  }

  // member: timestamp_2_start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp_2_start:\n";
    to_block_style_yaml(msg.timestamp_2_start, out, indentation + 2);
  }

  // member: timestamp_2_end
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp_2_end:\n";
    to_block_style_yaml(msg.timestamp_2_end, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ForceEvent & msg, bool use_flow_style = false)
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

}  // namespace messages

namespace rosidl_generator_traits
{

[[deprecated("use messages::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const messages::msg::ForceEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  messages::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use messages::msg::to_yaml() instead")]]
inline std::string to_yaml(const messages::msg::ForceEvent & msg)
{
  return messages::msg::to_yaml(msg);
}

template<>
inline const char * data_type<messages::msg::ForceEvent>()
{
  return "messages::msg::ForceEvent";
}

template<>
inline const char * name<messages::msg::ForceEvent>()
{
  return "messages/msg/ForceEvent";
}

template<>
struct has_fixed_size<messages::msg::ForceEvent>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<messages::msg::ForceEvent>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<messages::msg::ForceEvent>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES__MSG__DETAIL__FORCE_EVENT__TRAITS_HPP_
