// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tutorial_interfaces:msg/Board.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__MSG__DETAIL__BOARD__STRUCT_H_
#define TUTORIAL_INTERFACES__MSG__DETAIL__BOARD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Board in the package tutorial_interfaces.
typedef struct tutorial_interfaces__msg__Board
{
  int32_t key_up;
  int32_t key_down;
  int32_t key_left;
  int32_t key_right;
  int32_t key_left_shift;
  int32_t key_left_ctrl;
  int32_t w;
  int32_t a;
  int32_t s;
  int32_t d;
  int32_t q;
  int32_t e;
  int32_t c;
  int32_t x;
  int32_t z;
  int32_t j;
} tutorial_interfaces__msg__Board;

// Struct for a sequence of tutorial_interfaces__msg__Board.
typedef struct tutorial_interfaces__msg__Board__Sequence
{
  tutorial_interfaces__msg__Board * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tutorial_interfaces__msg__Board__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TUTORIAL_INTERFACES__MSG__DETAIL__BOARD__STRUCT_H_
