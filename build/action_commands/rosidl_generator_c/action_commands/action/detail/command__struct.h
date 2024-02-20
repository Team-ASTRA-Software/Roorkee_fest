// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from action_commands:action/Command.idl
// generated code does not contain a copyright notice

#ifndef ACTION_COMMANDS__ACTION__DETAIL__COMMAND__STRUCT_H_
#define ACTION_COMMANDS__ACTION__DETAIL__COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/Command in the package action_commands.
typedef struct action_commands__action__Command_Goal
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// Parameter 1, as defined by MAVLink uint16 VEHICLE_CMD enum.
  float param1;
  /// Parameter 2, as defined by MAVLink uint16 VEHICLE_CMD enum.
  float param2;
  /// Parameter 3, as defined by MAVLink uint16 VEHICLE_CMD enum.
  float param3;
  /// Parameter 4, as defined by MAVLink uint16 VEHICLE_CMD enum.
  float param4;
  /// Parameter 5, as defined by MAVLink uint16 VEHICLE_CMD enum.
  double param5;
  /// Parameter 6, as defined by MAVLink uint16 VEHICLE_CMD enum.
  double param6;
  /// Parameter 7, as defined by MAVLink uint16 VEHICLE_CMD enum.
  float param7;
  /// Command ID
  uint32_t command;
  /// System which should execute the command
  uint8_t target_system;
  /// Component which should execute the command, 0 for all components
  uint8_t target_component;
  /// System sending the command
  uint8_t source_system;
  /// Component / mode executor sending the command
  uint16_t source_component;
  /// 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
  uint8_t confirmation;
  bool from_external;
} action_commands__action__Command_Goal;

// Struct for a sequence of action_commands__action__Command_Goal.
typedef struct action_commands__action__Command_Goal__Sequence
{
  action_commands__action__Command_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__Command_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/Command in the package action_commands.
typedef struct action_commands__action__Command_Result
{
  uint8_t arming_state;
  uint8_t nav_state;
} action_commands__action__Command_Result;

// Struct for a sequence of action_commands__action__Command_Result.
typedef struct action_commands__action__Command_Result__Sequence
{
  action_commands__action__Command_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__Command_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/Command in the package action_commands.
typedef struct action_commands__action__Command_Feedback
{
  uint8_t arming_state_present;
  uint8_t nav_state_present;
} action_commands__action__Command_Feedback;

// Struct for a sequence of action_commands__action__Command_Feedback.
typedef struct action_commands__action__Command_Feedback__Sequence
{
  action_commands__action__Command_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__Command_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "action_commands/action/detail/command__struct.h"

/// Struct defined in action/Command in the package action_commands.
typedef struct action_commands__action__Command_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_commands__action__Command_Goal goal;
} action_commands__action__Command_SendGoal_Request;

// Struct for a sequence of action_commands__action__Command_SendGoal_Request.
typedef struct action_commands__action__Command_SendGoal_Request__Sequence
{
  action_commands__action__Command_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__Command_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Command in the package action_commands.
typedef struct action_commands__action__Command_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} action_commands__action__Command_SendGoal_Response;

// Struct for a sequence of action_commands__action__Command_SendGoal_Response.
typedef struct action_commands__action__Command_SendGoal_Response__Sequence
{
  action_commands__action__Command_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__Command_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Command in the package action_commands.
typedef struct action_commands__action__Command_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} action_commands__action__Command_GetResult_Request;

// Struct for a sequence of action_commands__action__Command_GetResult_Request.
typedef struct action_commands__action__Command_GetResult_Request__Sequence
{
  action_commands__action__Command_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__Command_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "action_commands/action/detail/command__struct.h"

/// Struct defined in action/Command in the package action_commands.
typedef struct action_commands__action__Command_GetResult_Response
{
  int8_t status;
  action_commands__action__Command_Result result;
} action_commands__action__Command_GetResult_Response;

// Struct for a sequence of action_commands__action__Command_GetResult_Response.
typedef struct action_commands__action__Command_GetResult_Response__Sequence
{
  action_commands__action__Command_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__Command_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "action_commands/action/detail/command__struct.h"

/// Struct defined in action/Command in the package action_commands.
typedef struct action_commands__action__Command_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_commands__action__Command_Feedback feedback;
} action_commands__action__Command_FeedbackMessage;

// Struct for a sequence of action_commands__action__Command_FeedbackMessage.
typedef struct action_commands__action__Command_FeedbackMessage__Sequence
{
  action_commands__action__Command_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_commands__action__Command_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ACTION_COMMANDS__ACTION__DETAIL__COMMAND__STRUCT_H_
