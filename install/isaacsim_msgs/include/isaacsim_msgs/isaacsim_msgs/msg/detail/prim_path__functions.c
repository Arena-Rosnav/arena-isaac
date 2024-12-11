// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from isaacsim_msgs:msg/PrimPath.idl
// generated code does not contain a copyright notice
#include "isaacsim_msgs/msg/detail/prim_path__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `path`
#include "rosidl_runtime_c/string_functions.h"

bool
isaacsim_msgs__msg__PrimPath__init(isaacsim_msgs__msg__PrimPath * msg)
{
  if (!msg) {
    return false;
  }
  // path
  if (!rosidl_runtime_c__String__init(&msg->path)) {
    isaacsim_msgs__msg__PrimPath__fini(msg);
    return false;
  }
  return true;
}

void
isaacsim_msgs__msg__PrimPath__fini(isaacsim_msgs__msg__PrimPath * msg)
{
  if (!msg) {
    return;
  }
  // path
  rosidl_runtime_c__String__fini(&msg->path);
}

bool
isaacsim_msgs__msg__PrimPath__are_equal(const isaacsim_msgs__msg__PrimPath * lhs, const isaacsim_msgs__msg__PrimPath * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->path), &(rhs->path)))
  {
    return false;
  }
  return true;
}

bool
isaacsim_msgs__msg__PrimPath__copy(
  const isaacsim_msgs__msg__PrimPath * input,
  isaacsim_msgs__msg__PrimPath * output)
{
  if (!input || !output) {
    return false;
  }
  // path
  if (!rosidl_runtime_c__String__copy(
      &(input->path), &(output->path)))
  {
    return false;
  }
  return true;
}

isaacsim_msgs__msg__PrimPath *
isaacsim_msgs__msg__PrimPath__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  isaacsim_msgs__msg__PrimPath * msg = (isaacsim_msgs__msg__PrimPath *)allocator.allocate(sizeof(isaacsim_msgs__msg__PrimPath), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(isaacsim_msgs__msg__PrimPath));
  bool success = isaacsim_msgs__msg__PrimPath__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
isaacsim_msgs__msg__PrimPath__destroy(isaacsim_msgs__msg__PrimPath * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    isaacsim_msgs__msg__PrimPath__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
isaacsim_msgs__msg__PrimPath__Sequence__init(isaacsim_msgs__msg__PrimPath__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  isaacsim_msgs__msg__PrimPath * data = NULL;

  if (size) {
    data = (isaacsim_msgs__msg__PrimPath *)allocator.zero_allocate(size, sizeof(isaacsim_msgs__msg__PrimPath), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = isaacsim_msgs__msg__PrimPath__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        isaacsim_msgs__msg__PrimPath__fini(&data[i - 1]);
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
isaacsim_msgs__msg__PrimPath__Sequence__fini(isaacsim_msgs__msg__PrimPath__Sequence * array)
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
      isaacsim_msgs__msg__PrimPath__fini(&array->data[i]);
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

isaacsim_msgs__msg__PrimPath__Sequence *
isaacsim_msgs__msg__PrimPath__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  isaacsim_msgs__msg__PrimPath__Sequence * array = (isaacsim_msgs__msg__PrimPath__Sequence *)allocator.allocate(sizeof(isaacsim_msgs__msg__PrimPath__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = isaacsim_msgs__msg__PrimPath__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
isaacsim_msgs__msg__PrimPath__Sequence__destroy(isaacsim_msgs__msg__PrimPath__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    isaacsim_msgs__msg__PrimPath__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
isaacsim_msgs__msg__PrimPath__Sequence__are_equal(const isaacsim_msgs__msg__PrimPath__Sequence * lhs, const isaacsim_msgs__msg__PrimPath__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!isaacsim_msgs__msg__PrimPath__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
isaacsim_msgs__msg__PrimPath__Sequence__copy(
  const isaacsim_msgs__msg__PrimPath__Sequence * input,
  isaacsim_msgs__msg__PrimPath__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(isaacsim_msgs__msg__PrimPath);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    isaacsim_msgs__msg__PrimPath * data =
      (isaacsim_msgs__msg__PrimPath *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!isaacsim_msgs__msg__PrimPath__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          isaacsim_msgs__msg__PrimPath__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!isaacsim_msgs__msg__PrimPath__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}