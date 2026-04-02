// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:msg/ForceEvent.idl
// generated code does not contain a copyright notice
#include "messages/msg/detail/force_event__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `timestamp_1_start`
// Member `timestamp_1_end`
// Member `timestamp_2_start`
// Member `timestamp_2_end`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
messages__msg__ForceEvent__init(messages__msg__ForceEvent * msg)
{
  if (!msg) {
    return false;
  }
  // force_magnitude_1
  // timestamp_1_start
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp_1_start)) {
    messages__msg__ForceEvent__fini(msg);
    return false;
  }
  // timestamp_1_end
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp_1_end)) {
    messages__msg__ForceEvent__fini(msg);
    return false;
  }
  // force_magnitude_2
  // timestamp_2_start
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp_2_start)) {
    messages__msg__ForceEvent__fini(msg);
    return false;
  }
  // timestamp_2_end
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp_2_end)) {
    messages__msg__ForceEvent__fini(msg);
    return false;
  }
  return true;
}

void
messages__msg__ForceEvent__fini(messages__msg__ForceEvent * msg)
{
  if (!msg) {
    return;
  }
  // force_magnitude_1
  // timestamp_1_start
  builtin_interfaces__msg__Time__fini(&msg->timestamp_1_start);
  // timestamp_1_end
  builtin_interfaces__msg__Time__fini(&msg->timestamp_1_end);
  // force_magnitude_2
  // timestamp_2_start
  builtin_interfaces__msg__Time__fini(&msg->timestamp_2_start);
  // timestamp_2_end
  builtin_interfaces__msg__Time__fini(&msg->timestamp_2_end);
}

bool
messages__msg__ForceEvent__are_equal(const messages__msg__ForceEvent * lhs, const messages__msg__ForceEvent * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // force_magnitude_1
  if (lhs->force_magnitude_1 != rhs->force_magnitude_1) {
    return false;
  }
  // timestamp_1_start
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp_1_start), &(rhs->timestamp_1_start)))
  {
    return false;
  }
  // timestamp_1_end
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp_1_end), &(rhs->timestamp_1_end)))
  {
    return false;
  }
  // force_magnitude_2
  if (lhs->force_magnitude_2 != rhs->force_magnitude_2) {
    return false;
  }
  // timestamp_2_start
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp_2_start), &(rhs->timestamp_2_start)))
  {
    return false;
  }
  // timestamp_2_end
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp_2_end), &(rhs->timestamp_2_end)))
  {
    return false;
  }
  return true;
}

bool
messages__msg__ForceEvent__copy(
  const messages__msg__ForceEvent * input,
  messages__msg__ForceEvent * output)
{
  if (!input || !output) {
    return false;
  }
  // force_magnitude_1
  output->force_magnitude_1 = input->force_magnitude_1;
  // timestamp_1_start
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp_1_start), &(output->timestamp_1_start)))
  {
    return false;
  }
  // timestamp_1_end
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp_1_end), &(output->timestamp_1_end)))
  {
    return false;
  }
  // force_magnitude_2
  output->force_magnitude_2 = input->force_magnitude_2;
  // timestamp_2_start
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp_2_start), &(output->timestamp_2_start)))
  {
    return false;
  }
  // timestamp_2_end
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp_2_end), &(output->timestamp_2_end)))
  {
    return false;
  }
  return true;
}

messages__msg__ForceEvent *
messages__msg__ForceEvent__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  messages__msg__ForceEvent * msg = (messages__msg__ForceEvent *)allocator.allocate(sizeof(messages__msg__ForceEvent), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__msg__ForceEvent));
  bool success = messages__msg__ForceEvent__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
messages__msg__ForceEvent__destroy(messages__msg__ForceEvent * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    messages__msg__ForceEvent__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
messages__msg__ForceEvent__Sequence__init(messages__msg__ForceEvent__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  messages__msg__ForceEvent * data = NULL;

  if (size) {
    data = (messages__msg__ForceEvent *)allocator.zero_allocate(size, sizeof(messages__msg__ForceEvent), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__msg__ForceEvent__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__msg__ForceEvent__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
messages__msg__ForceEvent__Sequence__fini(messages__msg__ForceEvent__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      messages__msg__ForceEvent__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

messages__msg__ForceEvent__Sequence *
messages__msg__ForceEvent__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  messages__msg__ForceEvent__Sequence * array = (messages__msg__ForceEvent__Sequence *)allocator.allocate(sizeof(messages__msg__ForceEvent__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = messages__msg__ForceEvent__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
messages__msg__ForceEvent__Sequence__destroy(messages__msg__ForceEvent__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    messages__msg__ForceEvent__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
messages__msg__ForceEvent__Sequence__are_equal(const messages__msg__ForceEvent__Sequence * lhs, const messages__msg__ForceEvent__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!messages__msg__ForceEvent__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
messages__msg__ForceEvent__Sequence__copy(
  const messages__msg__ForceEvent__Sequence * input,
  messages__msg__ForceEvent__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(messages__msg__ForceEvent);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    messages__msg__ForceEvent * data =
      (messages__msg__ForceEvent *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!messages__msg__ForceEvent__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          messages__msg__ForceEvent__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!messages__msg__ForceEvent__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
