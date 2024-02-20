// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from action_commands:action/FollowGPSWaypoints.idl
// generated code does not contain a copyright notice

#ifndef ACTION_COMMANDS__ACTION__DETAIL__FOLLOW_GPS_WAYPOINTS__TRAITS_HPP_
#define ACTION_COMMANDS__ACTION__DETAIL__FOLLOW_GPS_WAYPOINTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "action_commands/action/detail/follow_gps_waypoints__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'gps_poses'
#include "geometry_msgs/msg/detail/geo_pose__traits.hpp"

namespace action_commands
{

namespace action
{

inline void to_flow_style_yaml(
  const FollowGPSWaypoints_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: number_of_loops
  {
    out << "number_of_loops: ";
    rosidl_generator_traits::value_to_yaml(msg.number_of_loops, out);
    out << ", ";
  }

  // member: goal_index
  {
    out << "goal_index: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_index, out);
    out << ", ";
  }

  // member: gps_poses
  {
    if (msg.gps_poses.size() == 0) {
      out << "gps_poses: []";
    } else {
      out << "gps_poses: [";
      size_t pending_items = msg.gps_poses.size();
      for (auto item : msg.gps_poses) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FollowGPSWaypoints_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: number_of_loops
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "number_of_loops: ";
    rosidl_generator_traits::value_to_yaml(msg.number_of_loops, out);
    out << "\n";
  }

  // member: goal_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_index: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_index, out);
    out << "\n";
  }

  // member: gps_poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gps_poses.size() == 0) {
      out << "gps_poses: []\n";
    } else {
      out << "gps_poses:\n";
      for (auto item : msg.gps_poses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FollowGPSWaypoints_Goal & msg, bool use_flow_style = false)
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
  const action_commands::action::FollowGPSWaypoints_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::FollowGPSWaypoints_Goal & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::FollowGPSWaypoints_Goal>()
{
  return "action_commands::action::FollowGPSWaypoints_Goal";
}

template<>
inline const char * name<action_commands::action::FollowGPSWaypoints_Goal>()
{
  return "action_commands/action/FollowGPSWaypoints_Goal";
}

template<>
struct has_fixed_size<action_commands::action::FollowGPSWaypoints_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<action_commands::action::FollowGPSWaypoints_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<action_commands::action::FollowGPSWaypoints_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'missed_waypoints'
#include "action_commands/msg/detail/missed_waypoint__traits.hpp"

namespace action_commands
{

namespace action
{

inline void to_flow_style_yaml(
  const FollowGPSWaypoints_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: missed_waypoints
  {
    if (msg.missed_waypoints.size() == 0) {
      out << "missed_waypoints: []";
    } else {
      out << "missed_waypoints: [";
      size_t pending_items = msg.missed_waypoints.size();
      for (auto item : msg.missed_waypoints) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FollowGPSWaypoints_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: missed_waypoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.missed_waypoints.size() == 0) {
      out << "missed_waypoints: []\n";
    } else {
      out << "missed_waypoints:\n";
      for (auto item : msg.missed_waypoints) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FollowGPSWaypoints_Result & msg, bool use_flow_style = false)
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
  const action_commands::action::FollowGPSWaypoints_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::FollowGPSWaypoints_Result & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::FollowGPSWaypoints_Result>()
{
  return "action_commands::action::FollowGPSWaypoints_Result";
}

template<>
inline const char * name<action_commands::action::FollowGPSWaypoints_Result>()
{
  return "action_commands/action/FollowGPSWaypoints_Result";
}

template<>
struct has_fixed_size<action_commands::action::FollowGPSWaypoints_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<action_commands::action::FollowGPSWaypoints_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<action_commands::action::FollowGPSWaypoints_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace action_commands
{

namespace action
{

inline void to_flow_style_yaml(
  const FollowGPSWaypoints_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_waypoint
  {
    out << "current_waypoint: ";
    rosidl_generator_traits::value_to_yaml(msg.current_waypoint, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FollowGPSWaypoints_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_waypoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_waypoint: ";
    rosidl_generator_traits::value_to_yaml(msg.current_waypoint, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FollowGPSWaypoints_Feedback & msg, bool use_flow_style = false)
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
  const action_commands::action::FollowGPSWaypoints_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::FollowGPSWaypoints_Feedback & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::FollowGPSWaypoints_Feedback>()
{
  return "action_commands::action::FollowGPSWaypoints_Feedback";
}

template<>
inline const char * name<action_commands::action::FollowGPSWaypoints_Feedback>()
{
  return "action_commands/action/FollowGPSWaypoints_Feedback";
}

template<>
struct has_fixed_size<action_commands::action::FollowGPSWaypoints_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<action_commands::action::FollowGPSWaypoints_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<action_commands::action::FollowGPSWaypoints_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "action_commands/action/detail/follow_gps_waypoints__traits.hpp"

namespace action_commands
{

namespace action
{

inline void to_flow_style_yaml(
  const FollowGPSWaypoints_SendGoal_Request & msg,
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
  const FollowGPSWaypoints_SendGoal_Request & msg,
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

inline std::string to_yaml(const FollowGPSWaypoints_SendGoal_Request & msg, bool use_flow_style = false)
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
  const action_commands::action::FollowGPSWaypoints_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::FollowGPSWaypoints_SendGoal_Request & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::FollowGPSWaypoints_SendGoal_Request>()
{
  return "action_commands::action::FollowGPSWaypoints_SendGoal_Request";
}

template<>
inline const char * name<action_commands::action::FollowGPSWaypoints_SendGoal_Request>()
{
  return "action_commands/action/FollowGPSWaypoints_SendGoal_Request";
}

template<>
struct has_fixed_size<action_commands::action::FollowGPSWaypoints_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<action_commands::action::FollowGPSWaypoints_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<action_commands::action::FollowGPSWaypoints_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<action_commands::action::FollowGPSWaypoints_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<action_commands::action::FollowGPSWaypoints_SendGoal_Request>
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
  const FollowGPSWaypoints_SendGoal_Response & msg,
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
  const FollowGPSWaypoints_SendGoal_Response & msg,
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

inline std::string to_yaml(const FollowGPSWaypoints_SendGoal_Response & msg, bool use_flow_style = false)
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
  const action_commands::action::FollowGPSWaypoints_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::FollowGPSWaypoints_SendGoal_Response & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::FollowGPSWaypoints_SendGoal_Response>()
{
  return "action_commands::action::FollowGPSWaypoints_SendGoal_Response";
}

template<>
inline const char * name<action_commands::action::FollowGPSWaypoints_SendGoal_Response>()
{
  return "action_commands/action/FollowGPSWaypoints_SendGoal_Response";
}

template<>
struct has_fixed_size<action_commands::action::FollowGPSWaypoints_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<action_commands::action::FollowGPSWaypoints_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<action_commands::action::FollowGPSWaypoints_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<action_commands::action::FollowGPSWaypoints_SendGoal>()
{
  return "action_commands::action::FollowGPSWaypoints_SendGoal";
}

template<>
inline const char * name<action_commands::action::FollowGPSWaypoints_SendGoal>()
{
  return "action_commands/action/FollowGPSWaypoints_SendGoal";
}

template<>
struct has_fixed_size<action_commands::action::FollowGPSWaypoints_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<action_commands::action::FollowGPSWaypoints_SendGoal_Request>::value &&
    has_fixed_size<action_commands::action::FollowGPSWaypoints_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<action_commands::action::FollowGPSWaypoints_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<action_commands::action::FollowGPSWaypoints_SendGoal_Request>::value &&
    has_bounded_size<action_commands::action::FollowGPSWaypoints_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<action_commands::action::FollowGPSWaypoints_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<action_commands::action::FollowGPSWaypoints_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<action_commands::action::FollowGPSWaypoints_SendGoal_Response>
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
  const FollowGPSWaypoints_GetResult_Request & msg,
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
  const FollowGPSWaypoints_GetResult_Request & msg,
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

inline std::string to_yaml(const FollowGPSWaypoints_GetResult_Request & msg, bool use_flow_style = false)
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
  const action_commands::action::FollowGPSWaypoints_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::FollowGPSWaypoints_GetResult_Request & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::FollowGPSWaypoints_GetResult_Request>()
{
  return "action_commands::action::FollowGPSWaypoints_GetResult_Request";
}

template<>
inline const char * name<action_commands::action::FollowGPSWaypoints_GetResult_Request>()
{
  return "action_commands/action/FollowGPSWaypoints_GetResult_Request";
}

template<>
struct has_fixed_size<action_commands::action::FollowGPSWaypoints_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<action_commands::action::FollowGPSWaypoints_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<action_commands::action::FollowGPSWaypoints_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "action_commands/action/detail/follow_gps_waypoints__traits.hpp"

namespace action_commands
{

namespace action
{

inline void to_flow_style_yaml(
  const FollowGPSWaypoints_GetResult_Response & msg,
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
  const FollowGPSWaypoints_GetResult_Response & msg,
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

inline std::string to_yaml(const FollowGPSWaypoints_GetResult_Response & msg, bool use_flow_style = false)
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
  const action_commands::action::FollowGPSWaypoints_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::FollowGPSWaypoints_GetResult_Response & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::FollowGPSWaypoints_GetResult_Response>()
{
  return "action_commands::action::FollowGPSWaypoints_GetResult_Response";
}

template<>
inline const char * name<action_commands::action::FollowGPSWaypoints_GetResult_Response>()
{
  return "action_commands/action/FollowGPSWaypoints_GetResult_Response";
}

template<>
struct has_fixed_size<action_commands::action::FollowGPSWaypoints_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<action_commands::action::FollowGPSWaypoints_Result>::value> {};

template<>
struct has_bounded_size<action_commands::action::FollowGPSWaypoints_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<action_commands::action::FollowGPSWaypoints_Result>::value> {};

template<>
struct is_message<action_commands::action::FollowGPSWaypoints_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<action_commands::action::FollowGPSWaypoints_GetResult>()
{
  return "action_commands::action::FollowGPSWaypoints_GetResult";
}

template<>
inline const char * name<action_commands::action::FollowGPSWaypoints_GetResult>()
{
  return "action_commands/action/FollowGPSWaypoints_GetResult";
}

template<>
struct has_fixed_size<action_commands::action::FollowGPSWaypoints_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<action_commands::action::FollowGPSWaypoints_GetResult_Request>::value &&
    has_fixed_size<action_commands::action::FollowGPSWaypoints_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<action_commands::action::FollowGPSWaypoints_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<action_commands::action::FollowGPSWaypoints_GetResult_Request>::value &&
    has_bounded_size<action_commands::action::FollowGPSWaypoints_GetResult_Response>::value
  >
{
};

template<>
struct is_service<action_commands::action::FollowGPSWaypoints_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<action_commands::action::FollowGPSWaypoints_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<action_commands::action::FollowGPSWaypoints_GetResult_Response>
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
// #include "action_commands/action/detail/follow_gps_waypoints__traits.hpp"

namespace action_commands
{

namespace action
{

inline void to_flow_style_yaml(
  const FollowGPSWaypoints_FeedbackMessage & msg,
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
  const FollowGPSWaypoints_FeedbackMessage & msg,
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

inline std::string to_yaml(const FollowGPSWaypoints_FeedbackMessage & msg, bool use_flow_style = false)
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
  const action_commands::action::FollowGPSWaypoints_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_commands::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_commands::action::to_yaml() instead")]]
inline std::string to_yaml(const action_commands::action::FollowGPSWaypoints_FeedbackMessage & msg)
{
  return action_commands::action::to_yaml(msg);
}

template<>
inline const char * data_type<action_commands::action::FollowGPSWaypoints_FeedbackMessage>()
{
  return "action_commands::action::FollowGPSWaypoints_FeedbackMessage";
}

template<>
inline const char * name<action_commands::action::FollowGPSWaypoints_FeedbackMessage>()
{
  return "action_commands/action/FollowGPSWaypoints_FeedbackMessage";
}

template<>
struct has_fixed_size<action_commands::action::FollowGPSWaypoints_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<action_commands::action::FollowGPSWaypoints_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<action_commands::action::FollowGPSWaypoints_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<action_commands::action::FollowGPSWaypoints_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<action_commands::action::FollowGPSWaypoints_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<action_commands::action::FollowGPSWaypoints>
  : std::true_type
{
};

template<>
struct is_action_goal<action_commands::action::FollowGPSWaypoints_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<action_commands::action::FollowGPSWaypoints_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<action_commands::action::FollowGPSWaypoints_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // ACTION_COMMANDS__ACTION__DETAIL__FOLLOW_GPS_WAYPOINTS__TRAITS_HPP_
