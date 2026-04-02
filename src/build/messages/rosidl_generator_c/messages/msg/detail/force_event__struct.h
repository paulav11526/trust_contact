// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from messages:msg/ForceEvent.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__FORCE_EVENT__STRUCT_H_
#define MESSAGES__MSG__DETAIL__FORCE_EVENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'timestamp_1_start'
// Member 'timestamp_1_end'
// Member 'timestamp_2_start'
// Member 'timestamp_2_end'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/ForceEvent in the package messages.
/**
  * ForceEvent.msg
 */
typedef struct messages__msg__ForceEvent
{
  double force_magnitude_1;
  builtin_interfaces__msg__Time timestamp_1_start;
  builtin_interfaces__msg__Time timestamp_1_end;
  double force_magnitude_2;
  builtin_interfaces__msg__Time timestamp_2_start;
  builtin_interfaces__msg__Time timestamp_2_end;
} messages__msg__ForceEvent;

// Struct for a sequence of messages__msg__ForceEvent.
typedef struct messages__msg__ForceEvent__Sequence
{
  messages__msg__ForceEvent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages__msg__ForceEvent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES__MSG__DETAIL__FORCE_EVENT__STRUCT_H_
