// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tutorial_interfaces:msg/Board.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__MSG__DETAIL__BOARD__TRAITS_HPP_
#define TUTORIAL_INTERFACES__MSG__DETAIL__BOARD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tutorial_interfaces/msg/detail/board__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tutorial_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Board & msg,
  std::ostream & out)
{
  out << "{";
  // member: key_up
  {
    out << "key_up: ";
    rosidl_generator_traits::value_to_yaml(msg.key_up, out);
    out << ", ";
  }

  // member: key_down
  {
    out << "key_down: ";
    rosidl_generator_traits::value_to_yaml(msg.key_down, out);
    out << ", ";
  }

  // member: key_left
  {
    out << "key_left: ";
    rosidl_generator_traits::value_to_yaml(msg.key_left, out);
    out << ", ";
  }

  // member: key_right
  {
    out << "key_right: ";
    rosidl_generator_traits::value_to_yaml(msg.key_right, out);
    out << ", ";
  }

  // member: key_left_shift
  {
    out << "key_left_shift: ";
    rosidl_generator_traits::value_to_yaml(msg.key_left_shift, out);
    out << ", ";
  }

  // member: key_left_ctrl
  {
    out << "key_left_ctrl: ";
    rosidl_generator_traits::value_to_yaml(msg.key_left_ctrl, out);
    out << ", ";
  }

  // member: w
  {
    out << "w: ";
    rosidl_generator_traits::value_to_yaml(msg.w, out);
    out << ", ";
  }

  // member: a
  {
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << ", ";
  }

  // member: s
  {
    out << "s: ";
    rosidl_generator_traits::value_to_yaml(msg.s, out);
    out << ", ";
  }

  // member: d
  {
    out << "d: ";
    rosidl_generator_traits::value_to_yaml(msg.d, out);
    out << ", ";
  }

  // member: q
  {
    out << "q: ";
    rosidl_generator_traits::value_to_yaml(msg.q, out);
    out << ", ";
  }

  // member: e
  {
    out << "e: ";
    rosidl_generator_traits::value_to_yaml(msg.e, out);
    out << ", ";
  }

  // member: c
  {
    out << "c: ";
    rosidl_generator_traits::value_to_yaml(msg.c, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: j
  {
    out << "j: ";
    rosidl_generator_traits::value_to_yaml(msg.j, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Board & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: key_up
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "key_up: ";
    rosidl_generator_traits::value_to_yaml(msg.key_up, out);
    out << "\n";
  }

  // member: key_down
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "key_down: ";
    rosidl_generator_traits::value_to_yaml(msg.key_down, out);
    out << "\n";
  }

  // member: key_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "key_left: ";
    rosidl_generator_traits::value_to_yaml(msg.key_left, out);
    out << "\n";
  }

  // member: key_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "key_right: ";
    rosidl_generator_traits::value_to_yaml(msg.key_right, out);
    out << "\n";
  }

  // member: key_left_shift
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "key_left_shift: ";
    rosidl_generator_traits::value_to_yaml(msg.key_left_shift, out);
    out << "\n";
  }

  // member: key_left_ctrl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "key_left_ctrl: ";
    rosidl_generator_traits::value_to_yaml(msg.key_left_ctrl, out);
    out << "\n";
  }

  // member: w
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "w: ";
    rosidl_generator_traits::value_to_yaml(msg.w, out);
    out << "\n";
  }

  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << "\n";
  }

  // member: s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "s: ";
    rosidl_generator_traits::value_to_yaml(msg.s, out);
    out << "\n";
  }

  // member: d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d: ";
    rosidl_generator_traits::value_to_yaml(msg.d, out);
    out << "\n";
  }

  // member: q
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q: ";
    rosidl_generator_traits::value_to_yaml(msg.q, out);
    out << "\n";
  }

  // member: e
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "e: ";
    rosidl_generator_traits::value_to_yaml(msg.e, out);
    out << "\n";
  }

  // member: c
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "c: ";
    rosidl_generator_traits::value_to_yaml(msg.c, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: j
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "j: ";
    rosidl_generator_traits::value_to_yaml(msg.j, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Board & msg, bool use_flow_style = false)
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

}  // namespace tutorial_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use tutorial_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tutorial_interfaces::msg::Board & msg,
  std::ostream & out, size_t indentation = 0)
{
  tutorial_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tutorial_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const tutorial_interfaces::msg::Board & msg)
{
  return tutorial_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tutorial_interfaces::msg::Board>()
{
  return "tutorial_interfaces::msg::Board";
}

template<>
inline const char * name<tutorial_interfaces::msg::Board>()
{
  return "tutorial_interfaces/msg/Board";
}

template<>
struct has_fixed_size<tutorial_interfaces::msg::Board>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<tutorial_interfaces::msg::Board>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<tutorial_interfaces::msg::Board>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TUTORIAL_INTERFACES__MSG__DETAIL__BOARD__TRAITS_HPP_
