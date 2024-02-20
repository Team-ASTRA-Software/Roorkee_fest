// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from action_commands:action/Command.idl
// generated code does not contain a copyright notice

#ifndef ACTION_COMMANDS__ACTION__DETAIL__COMMAND__TRAITS_HPP_
#define ACTION_COMMANDS__ACTION__DETAIL__COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "action_commands/action/detail/command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace action_commands
{

namespace action
{

inline void to_flow_style_yaml(
  const Command_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: param1
  {
    out << "param1: ";
    rosidl_generator_traits::value_to_yaml(msg.param1, out);
    out << ", ";
  }

  // member: param2
  {
    out << "param2: ";
    rosidl_generator_traits::value_to_yaml(msg.param2, out);
    out << ", ";
  }

  // member: param3
  {
    out << "param3: ";
    rosidl_generator_traits::value_to_yaml(msg.param3, out);
    out << ", ";
  }

  // member: param4
  {
    out << "param4: ";
    rosidl_generator_traits::value_to_yaml(msg.param4, out);
    out << ", ";
  }

  // member: param5
  {
    out << "param5: ";
    rosidl_generator_traits::value_to_yaml(msg.param5, out);
    out << ", ";
  }

  // member: param6
  {
    out << "param6: ";
    rosidl_generator_traits::value_to_yaml(msg.param6, out);
    out << ", ";
  }

  // member: param7
  {
    out << "param7: ";
    rosidl_generator_traits::value_to_yaml(msg.param7, out);
    out << ", ";
  }

  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: target_system
  {
    out << "target_system: ";
    rosidl_generator_traits::value_to_yaml(msg.target_system, out);
    out << ", ";
  }

  // member: target_component
  {
    out << "target_component: ";
    rosidl_generator_traits::value_to_yaml(msg.target_component, out);
    out << ", ";
  }

  // member: source_system
  {
    out << "source_system: ";
    rosidl_generator_traits::value_to_yaml(msg.source_system, out);
    out << ", ";
  }

  // member: source_component
  {
    out << "source_component: ";
    rosidl_generator_traits::value_to_yaml(msg.source_component, out);
    out << ", ";
  }

  // member: confirmation
  {
    out << "confirmation: ";
    rosidl_generator_traits::value_to_yaml(msg.confirmation, out);
    out << ", ";
  }

  // member: from_external
  {
    out << "from_external: ";
    rosidl_generator_traits::value_to_yaml(msg.from_external, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Command_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: param1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param1: ";
    rosidl_generator_traits::value_to_yaml(msg.param1, out);
    out << "\n";
  }

  // member: param2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param2: ";
    rosidl_generator_traits::value_to_yaml(msg.param2, out);
    out << "\n";
  }

  // member: param3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param3: ";
    rosidl_generator_traits::value_to_yaml(msg.param3, out);
    out << "\n";
  }

  // member: param4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param4: ";
    rosidl_generator_traits::value_to_yaml(msg.param4, out);
    out << "\n";
  }

  // member: param5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param5: ";
    rosidl_generator_traits::value_to_yaml(msg.param5, out);
    out << "\n";
  }

  // member: param6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param6: ";
    rosidl_generator_traits::value_to_yaml(msg.param6, out);
    out << "\n";
  }

  // member: param7
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param7: ";
    rosidl_generator_traits::value_to_yaml(msg.param7, out);
    out << "\n";
  }

  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: target_system
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_system: ";
    rosidl_generator_traits::value_to_yaml(msg.target_system, out);
    out << "\n";
  }

  // member: target_component
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_component: ";
    rosidl_generator_traits::value_to_yaml(msg.target_component, out);
    out << "\n";
  }

  // member: source_system
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "source_system: ";
    rosidl_generator_traits::value_to_yaml(msg.source_system, out);
    out << "\n";
  }

  // member: source_component
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "source_component: ";
    rosidl_generator_traits::value_to_yaml(msg.source_component, out);
    out << "\n";
  }

  // member: confirmation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confirmation: ";
    rosidl_generator_traits::value_to_yaml(msg.confirmation, out);
    out << "\n";
  }

  // member: from_external
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "from_external: ";
    rosidl_generator_traits::value_to_yaml(msg.from_external, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Command_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_commands

namespace rosidl_generator_traits
{

[[deprecated("use action_commands::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_commands::action::Command_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::Command_Goal & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::Command_Goal>()
{
  return "action_commands::action::Command_Goal";
}

template<>
inline const char * name<action_commands::action::Command_Goal>()
{
  return "action_commands/action/Command_Goal";
}

template<>
struct has_fixed_size<action_commands::action::Command_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<action_commands::action::Command_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<action_commands::action::Command_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace action_commands
{

namespace action
{

inline void to_flow_style_yaml(
  const Command_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: arming_state
  {
    out << "arming_state: ";
    rosidl_generator_traits::value_to_yaml(msg.arming_state, out);
    out << ", ";
  }

  // member: nav_state
  {
    out << "nav_state: ";
    rosidl_generator_traits::value_to_yaml(msg.nav_state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Command_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: arming_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "arming_state: ";
    rosidl_generator_traits::value_to_yaml(msg.arming_state, out);
    out << "\n";
  }

  // member: nav_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nav_state: ";
    rosidl_generator_traits::value_to_yaml(msg.nav_state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Command_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_commands

namespace rosidl_generator_traits
{

[[deprecated("use action_commands::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_commands::action::Command_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::Command_Result & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::Command_Result>()
{
  return "action_commands::action::Command_Result";
}

template<>
inline const char * name<action_commands::action::Command_Result>()
{
  return "action_commands/action/Command_Result";
}

template<>
struct has_fixed_size<action_commands::action::Command_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<action_commands::action::Command_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<action_commands::action::Command_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace action_commands
{

namespace action
{

inline void to_flow_style_yaml(
  const Command_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: arming_state_present
  {
    out << "arming_state_present: ";
    rosidl_generator_traits::value_to_yaml(msg.arming_state_present, out);
    out << ", ";
  }

  // member: nav_state_present
  {
    out << "nav_state_present: ";
    rosidl_generator_traits::value_to_yaml(msg.nav_state_present, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Command_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: arming_state_present
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "arming_state_present: ";
    rosidl_generator_traits::value_to_yaml(msg.arming_state_present, out);
    out << "\n";
  }

  // member: nav_state_present
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nav_state_present: ";
    rosidl_generator_traits::value_to_yaml(msg.nav_state_present, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Command_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_commands

namespace rosidl_generator_traits
{

[[deprecated("use action_commands::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_commands::action::Command_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::Command_Feedback & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::Command_Feedback>()
{
  return "action_commands::action::Command_Feedback";
}

template<>
inline const char * name<action_commands::action::Command_Feedback>()
{
  return "action_commands/action/Command_Feedback";
}

template<>
struct has_fixed_size<action_commands::action::Command_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<action_commands::action::Command_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<action_commands::action::Command_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "action_commands/action/detail/command__traits.hpp"

namespace action_commands
{

namespace action
{

inline void to_flow_style_yaml(
  const Command_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Command_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Command_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_commands

namespace rosidl_generator_traits
{

[[deprecated("use action_commands::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_commands::action::Command_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::Command_SendGoal_Request & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::Command_SendGoal_Request>()
{
  return "action_commands::action::Command_SendGoal_Request";
}

template<>
inline const char * name<action_commands::action::Command_SendGoal_Request>()
{
  return "action_commands/action/Command_SendGoal_Request";
}

template<>
struct has_fixed_size<action_commands::action::Command_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<action_commands::action::Command_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<action_commands::action::Command_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<action_commands::action::Command_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<action_commands::action::Command_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace action_commands
{

namespace action
{

inline void to_flow_style_yaml(
  const Command_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Command_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Command_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_commands

namespace rosidl_generator_traits
{

[[deprecated("use action_commands::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_commands::action::Command_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::Command_SendGoal_Response & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::Command_SendGoal_Response>()
{
  return "action_commands::action::Command_SendGoal_Response";
}

template<>
inline const char * name<action_commands::action::Command_SendGoal_Response>()
{
  return "action_commands/action/Command_SendGoal_Response";
}

template<>
struct has_fixed_size<action_commands::action::Command_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<action_commands::action::Command_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<action_commands::action::Command_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<action_commands::action::Command_SendGoal>()
{
  return "action_commands::action::Command_SendGoal";
}

template<>
inline const char * name<action_commands::action::Command_SendGoal>()
{
  return "action_commands/action/Command_SendGoal";
}

template<>
struct has_fixed_size<action_commands::action::Command_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<action_commands::action::Command_SendGoal_Request>::value &&
    has_fixed_size<action_commands::action::Command_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<action_commands::action::Command_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<action_commands::action::Command_SendGoal_Request>::value &&
    has_bounded_size<action_commands::action::Command_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<action_commands::action::Command_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<action_commands::action::Command_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<action_commands::action::Command_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace action_commands
{

namespace action
{

inline void to_flow_style_yaml(
  const Command_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Command_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Command_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_commands

namespace rosidl_generator_traits
{

[[deprecated("use action_commands::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_commands::action::Command_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::Command_GetResult_Request & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::Command_GetResult_Request>()
{
  return "action_commands::action::Command_GetResult_Request";
}

template<>
inline const char * name<action_commands::action::Command_GetResult_Request>()
{
  return "action_commands/action/Command_GetResult_Request";
}

template<>
struct has_fixed_size<action_commands::action::Command_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<action_commands::action::Command_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<action_commands::action::Command_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "action_commands/action/detail/command__traits.hpp"

namespace action_commands
{

namespace action
{

inline void to_flow_style_yaml(
  const Command_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Command_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Command_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_commands

namespace rosidl_generator_traits
{

[[deprecated("use action_commands::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_commands::action::Command_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::Command_GetResult_Response & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::Command_GetResult_Response>()
{
  return "action_commands::action::Command_GetResult_Response";
}

template<>
inline const char * name<action_commands::action::Command_GetResult_Response>()
{
  return "action_commands/action/Command_GetResult_Response";
}

template<>
struct has_fixed_size<action_commands::action::Command_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<action_commands::action::Command_Result>::value> {};

template<>
struct has_bounded_size<action_commands::action::Command_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<action_commands::action::Command_Result>::value> {};

template<>
struct is_message<action_commands::action::Command_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<action_commands::action::Command_GetResult>()
{
  return "action_commands::action::Command_GetResult";
}

template<>
inline const char * name<action_commands::action::Command_GetResult>()
{
  return "action_commands/action/Command_GetResult";
}

template<>
struct has_fixed_size<action_commands::action::Command_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<action_commands::action::Command_GetResult_Request>::value &&
    has_fixed_size<action_commands::action::Command_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<action_commands::action::Command_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<action_commands::action::Command_GetResult_Request>::value &&
    has_bounded_size<action_commands::action::Command_GetResult_Response>::value
  >
{
};

template<>
struct is_service<action_commands::action::Command_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<action_commands::action::Command_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<action_commands::action::Command_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "action_commands/action/detail/command__traits.hpp"

namespace action_commands
{

namespace action
{

inline void to_flow_style_yaml(
  const Command_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Command_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Command_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace action_commands

namespace rosidl_generator_traits
{

[[deprecated("use action_commands::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_commands::action::Command_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::Command_FeedbackMessage & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::Command_FeedbackMessage>()
{
  return "action_commands::action::Command_FeedbackMessage";
}

template<>
inline const char * name<action_commands::action::Command_FeedbackMessage>()
{
  return "action_commands/action/Command_FeedbackMessage";
}

template<>
struct has_fixed_size<action_commands::action::Command_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<action_commands::action::Command_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<action_commands::action::Command_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<action_commands::action::Command_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<action_commands::action::Command_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<action_commands::action::Command>
  : std::true_type
{
};

template<>
struct is_action_goal<action_commands::action::Command_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<action_commands::action::Command_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<action_commands::action::Command_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // ACTION_COMMANDS__ACTION__DETAIL__COMMAND__TRAITS_HPP_
