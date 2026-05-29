// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from debug_msg:msg/Debug.idl
// generated code does not contain a copyright notice
#include "debug_msg/msg/detail/debug__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
debug_msg__msg__Debug__init(debug_msg__msg__Debug * msg)
{
  if (!msg) {
    return false;
  }
  // commanded_torque
  // stiffness_torque
  // damping_torque
  // coriolis_torque
  // nullspace_torque
  // impedance_force
  return true;
}

void
debug_msg__msg__Debug__fini(debug_msg__msg__Debug * msg)
{
  if (!msg) {
    return;
  }
  // commanded_torque
  // stiffness_torque
  // damping_torque
  // coriolis_torque
  // nullspace_torque
  // impedance_force
}

bool
debug_msg__msg__Debug__are_equal(const debug_msg__msg__Debug * lhs, const debug_msg__msg__Debug * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // commanded_torque
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->commanded_torque[i] != rhs->commanded_torque[i]) {
      return false;
    }
  }
  // stiffness_torque
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->stiffness_torque[i] != rhs->stiffness_torque[i]) {
      return false;
    }
  }
  // damping_torque
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->damping_torque[i] != rhs->damping_torque[i]) {
      return false;
    }
  }
  // coriolis_torque
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->coriolis_torque[i] != rhs->coriolis_torque[i]) {
      return false;
    }
  }
  // nullspace_torque
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->nullspace_torque[i] != rhs->nullspace_torque[i]) {
      return false;
    }
  }
  // impedance_force
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->impedance_force[i] != rhs->impedance_force[i]) {
      return false;
    }
  }
  return true;
}

bool
debug_msg__msg__Debug__copy(
  const debug_msg__msg__Debug * input,
  debug_msg__msg__Debug * output)
{
  if (!input || !output) {
    return false;
  }
  // commanded_torque
  for (size_t i = 0; i < 7; ++i) {
    output->commanded_torque[i] = input->commanded_torque[i];
  }
  // stiffness_torque
  for (size_t i = 0; i < 7; ++i) {
    output->stiffness_torque[i] = input->stiffness_torque[i];
  }
  // damping_torque
  for (size_t i = 0; i < 7; ++i) {
    output->damping_torque[i] = input->damping_torque[i];
  }
  // coriolis_torque
  for (size_t i = 0; i < 7; ++i) {
    output->coriolis_torque[i] = input->coriolis_torque[i];
  }
  // nullspace_torque
  for (size_t i = 0; i < 7; ++i) {
    output->nullspace_torque[i] = input->nullspace_torque[i];
  }
  // impedance_force
  for (size_t i = 0; i < 6; ++i) {
    output->impedance_force[i] = input->impedance_force[i];
  }
  return true;
}

debug_msg__msg__Debug *
debug_msg__msg__Debug__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  debug_msg__msg__Debug * msg = (debug_msg__msg__Debug *)allocator.allocate(sizeof(debug_msg__msg__Debug), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(debug_msg__msg__Debug));
  bool success = debug_msg__msg__Debug__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
debug_msg__msg__Debug__destroy(debug_msg__msg__Debug * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    debug_msg__msg__Debug__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
debug_msg__msg__Debug__Sequence__init(debug_msg__msg__Debug__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  debug_msg__msg__Debug * data = NULL;

  if (size) {
    data = (debug_msg__msg__Debug *)allocator.zero_allocate(size, sizeof(debug_msg__msg__Debug), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = debug_msg__msg__Debug__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        debug_msg__msg__Debug__fini(&data[i - 1]);
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
debug_msg__msg__Debug__Sequence__fini(debug_msg__msg__Debug__Sequence * array)
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
      debug_msg__msg__Debug__fini(&array->data[i]);
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

debug_msg__msg__Debug__Sequence *
debug_msg__msg__Debug__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  debug_msg__msg__Debug__Sequence * array = (debug_msg__msg__Debug__Sequence *)allocator.allocate(sizeof(debug_msg__msg__Debug__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = debug_msg__msg__Debug__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
debug_msg__msg__Debug__Sequence__destroy(debug_msg__msg__Debug__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    debug_msg__msg__Debug__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
debug_msg__msg__Debug__Sequence__are_equal(const debug_msg__msg__Debug__Sequence * lhs, const debug_msg__msg__Debug__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!debug_msg__msg__Debug__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
debug_msg__msg__Debug__Sequence__copy(
  const debug_msg__msg__Debug__Sequence * input,
  debug_msg__msg__Debug__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(debug_msg__msg__Debug);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    debug_msg__msg__Debug * data =
      (debug_msg__msg__Debug *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!debug_msg__msg__Debug__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          debug_msg__msg__Debug__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!debug_msg__msg__Debug__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
