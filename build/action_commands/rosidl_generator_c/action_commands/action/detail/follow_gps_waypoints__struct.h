// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from action_commands:action/FollowGPSWaypoints.idl
// generated code does not contain a copyright notice

#ifndef ACTION_COMMANDS__ACTION__DETAIL__FOLLOW_GPS_WAYPOINTS__STRUCT_H_
#define ACTION_COMMANDS__ACTION__DETAIL__FOLLOW_GPS_WAYPOINTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'gps_poses'
#include "geometry_msgs/msg/detail/geo_pose__struct.h"

/// Struct defined in action/FollowGPSWaypoints in the package action_commands.
typedef struct action_commands__action__FollowGPSWaypoints_Goal
{
  uint32_t number_of_loops;
  uint32_t goal_index;
  geometry_msgs__msg__GeoPose__Sequence gps_poses;
} action_commands__action__FollowGPSWaypoints_Goal;

// Struct for a sequence of action_commands__action__FollowGPSWaypoints_Goal.
typedef struct action_commands__action__FollowGPSWaypoints_Goal__Sequence
{
  action_commands__action__FollowGPSWaypoints_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__FollowGPSWaypoints_Goal__Sequence;


// Constants defined in the message

/// Constant 'NONE'.
/**
  * Error codes
  * Note: The expected priority order of the errors should match the message order
 */
enum
{
  action_commands__action__FollowGPSWaypoints_Result__NONE = 0
};

/// Constant 'UNKNOWN'.
enum
{
  action_commands__action__FollowGPSWaypoints_Result__UNKNOWN = 600
};

/// Constant 'TASK_EXECUTOR_FAILED'.
enum
{
  action_commands__action__FollowGPSWaypoints_Result__TASK_EXECUTOR_FAILED = 601
};

// Include directives for member types
// Member 'missed_waypoints'
#include "action_commands/msg/detail/missed_waypoint__struct.h"

/// Struct defined in action/FollowGPSWaypoints in the package action_commands.
typedef struct action_commands__action__FollowGPSWaypoints_Result
{
  action_commands__msg__MissedWaypoint__Sequence missed_waypoints;
} action_commands__action__FollowGPSWaypoints_Result;

// Struct for a sequence of action_commands__action__FollowGPSWaypoints_Result.
typedef struct action_commands__action__FollowGPSWaypoints_Result__Sequence
{
  action_commands__action__FollowGPSWaypoints_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__FollowGPSWaypoints_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/FollowGPSWaypoints in the package action_commands.
typedef struct action_commands__action__FollowGPSWaypoints_Feedback
{
  uint32_t current_waypoint;
} action_commands__action__FollowGPSWaypoints_Feedback;

// Struct for a sequence of action_commands__action__FollowGPSWaypoints_Feedback.
typedef struct action_commands__action__FollowGPSWaypoints_Feedback__Sequence
{
  action_commands__action__FollowGPSWaypoints_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__FollowGPSWaypoints_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "action_commands/action/detail/follow_gps_waypoints__struct.h"

/// Struct defined in action/FollowGPSWaypoints in the package action_commands.
typedef struct action_commands__action__FollowGPSWaypoints_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_commands__action__FollowGPSWaypoints_Goal goal;
} action_commands__action__FollowGPSWaypoints_SendGoal_Request;

// Struct for a sequence of action_commands__action__FollowGPSWaypoints_SendGoal_Request.
typedef struct action_commands__action__FollowGPSWaypoints_SendGoal_Request__Sequence
{
  action_commands__action__FollowGPSWaypoints_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__FollowGPSWaypoints_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/FollowGPSWaypoints in the package action_commands.
typedef struct action_commands__action__FollowGPSWaypoints_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} action_commands__action__FollowGPSWaypoints_SendGoal_Response;

// Struct for a sequence of action_commands__action__FollowGPSWaypoints_SendGoal_Response.
typedef struct action_commands__action__FollowGPSWaypoints_SendGoal_Response__Sequence
{
  action_commands__action__FollowGPSWaypoints_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__FollowGPSWaypoints_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/FollowGPSWaypoints in the package action_commands.
typedef struct action_commands__action__FollowGPSWaypoints_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} action_commands__action__FollowGPSWaypoints_GetResult_Request;

// Struct for a sequence of action_commands__action__FollowGPSWaypoints_GetResult_Request.
typedef struct action_commands__action__FollowGPSWaypoints_GetResult_Request__Sequence
{
  action_commands__action__FollowGPSWaypoints_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__FollowGPSWaypoints_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "action_commands/action/detail/follow_gps_waypoints__struct.h"

/// Struct defined in action/FollowGPSWaypoints in the package action_commands.
typedef struct action_commands__action__FollowGPSWaypoints_GetResult_Response
{
  int8_t status;
  action_commands__action__FollowGPSWaypoints_Result result;
} action_commands__action__FollowGPSWaypoints_GetResult_Response;

// Struct for a sequence of action_commands__action__FollowGPSWaypoints_GetResult_Response.
typedef struct action_commands__action__FollowGPSWaypoints_GetResult_Response__Sequence
{
  action_commands__action__FollowGPSWaypoints_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__FollowGPSWaypoints_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "action_commands/action/detail/follow_gps_waypoints__struct.h"

/// Struct defined in action/FollowGPSWaypoints in the package action_commands.
typedef struct action_commands__action__FollowGPSWaypoints_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_commands__action__FollowGPSWaypoints_Feedback feedback;
} action_commands__action__FollowGPSWaypoints_FeedbackMessage;

// Struct for a sequence of action_commands__action__FollowGPSWaypoints_FeedbackMessage.
typedef struct action_commands__action__FollowGPSWaypoints_FeedbackMessage__Sequence
{
  action_commands__action__FollowGPSWaypoints_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__FollowGPSWaypoints_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ACTION_COMMANDS__ACTION__DETAIL__FOLLOW_GPS_WAYPOINTS__STRUCT_H_
