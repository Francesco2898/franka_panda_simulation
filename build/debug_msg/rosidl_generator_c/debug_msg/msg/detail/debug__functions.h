// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from debug_msg:msg/Debug.idl
// generated code does not contain a copyright notice

#ifndef DEBUG_MSG__MSG__DETAIL__DEBUG__FUNCTIONS_H_
#define DEBUG_MSG__MSG__DETAIL__DEBUG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "debug_msg/msg/rosidl_generator_c__visibility_control.h"

#include "debug_msg/msg/detail/debug__struct.h"

/// Initialize msg/Debug message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * debug_msg__msg__Debug
 * )) before or use
 * debug_msg__msg__Debug__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_debug_msg
bool
debug_msg__msg__Debug__init(debug_msg__msg__Debug * msg);

/// Finalize msg/Debug message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_debug_msg
void
debug_msg__msg__Debug__fini(debug_msg__msg__Debug * msg);

/// Create msg/Debug message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * debug_msg__msg__Debug__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_debug_msg
debug_msg__msg__Debug *
debug_msg__msg__Debug__create();

/// Destroy msg/Debug message.
/**
 * It calls
 * debug_msg__msg__Debug__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_debug_msg
void
debug_msg__msg__Debug__destroy(debug_msg__msg__Debug * msg);

/// Check for msg/Debug message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_debug_msg
bool
debug_msg__msg__Debug__are_equal(const debug_msg__msg__Debug * lhs, const debug_msg__msg__Debug * rhs);

/// Copy a msg/Debug message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_debug_msg
bool
debug_msg__msg__Debug__copy(
  const debug_msg__msg__Debug * input,
  debug_msg__msg__Debug * output);

/// Initialize array of msg/Debug messages.
/**
 * It allocates the memory for the number of elements and calls
 * debug_msg__msg__Debug__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_debug_msg
bool
debug_msg__msg__Debug__Sequence__init(debug_msg__msg__Debug__Sequence * array, size_t size);

/// Finalize array of msg/Debug messages.
/**
 * It calls
 * debug_msg__msg__Debug__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_debug_msg
void
debug_msg__msg__Debug__Sequence__fini(debug_msg__msg__Debug__Sequence * array);

/// Create array of msg/Debug messages.
/**
 * It allocates the memory for the array and calls
 * debug_msg__msg__Debug__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_debug_msg
debug_msg__msg__Debug__Sequence *
debug_msg__msg__Debug__Sequence__create(size_t size);

/// Destroy array of msg/Debug messages.
/**
 * It calls
 * debug_msg__msg__Debug__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_debug_msg
void
debug_msg__msg__Debug__Sequence__destroy(debug_msg__msg__Debug__Sequence * array);

/// Check for msg/Debug message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_debug_msg
bool
debug_msg__msg__Debug__Sequence__are_equal(const debug_msg__msg__Debug__Sequence * lhs, const debug_msg__msg__Debug__Sequence * rhs);

/// Copy an array of msg/Debug messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_debug_msg
bool
debug_msg__msg__Debug__Sequence__copy(
  const debug_msg__msg__Debug__Sequence * input,
  debug_msg__msg__Debug__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DEBUG_MSG__MSG__DETAIL__DEBUG__FUNCTIONS_H_
